#!/usr/bin/env python3
import re
import time
import socket
import threading
import subprocess
from typing import Any, Dict, Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Optional MQTT
try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None


def kv(st: DiagnosticStatus, k: str, v: Any):
    st.values.append(KeyValue(key=k, value=str(v)))


def now_s() -> float:
    return time.time()


def _run_cmd(cmd: List[str], timeout: float) -> Tuple[int, str, str]:
    """
    Run a command, always return (rc, stdout, stderr). Never raises.
    """
    try:
        p = subprocess.run(
            cmd, capture_output=True, text=True, timeout=max(0.1, timeout)
        )
        return p.returncode, p.stdout or "", p.stderr or ""
    except subprocess.TimeoutExpired as e:
        return 124, e.stdout or "", e.stderr or "timeout"
    except FileNotFoundError as e:
        return 127, "", str(e)
    except Exception as e:
        return 125, "", str(e)


def _auto_wifi_iface() -> Optional[str]:
    # Try "iw dev"
    rc, out, _ = _run_cmd(["iw", "dev"], timeout=1.5)
    if rc == 0:
        m = re.search(r"Interface\s+(\S+)", out)
        if m:
            return m.group(1)
    # Fallback: first interface starting with wl* from `ip -o link`
    rc, out, _ = _run_cmd(["ip", "-o", "link"], timeout=1.0)
    if rc == 0:
        for line in out.splitlines():
            m = re.search(r"\d+:\s+([^:]+):", line)
            if m and m.group(1).startswith("wl"):
                return m.group(1)
    return None


class CheckState:
    """Thread-safe status cache for one check."""
    def __init__(self, name: str, stale_s: float):
        self.name = name
        self.level = DiagnosticStatus.STALE
        self.message = "no data"
        self.kvs: List[Tuple[str, Any]] = []
        self.stamp_s: float = 0.0
        self.stale_s: float = float(stale_s)
        self._lock = threading.Lock()

    def set(self, level: int, message: str, kv_pairs: Optional[List[Tuple[str, Any]]] = None):
        with self._lock:
            self.level = level
            self.message = message
            self.kvs = list(kv_pairs) if kv_pairs else []
            self.stamp_s = now_s()

    def snapshot(self) -> Tuple[int, str, List[Tuple[str, Any]], float, float]:
        with self._lock:
            return self.level, self.message, list(self.kvs), self.stamp_s, self.stale_s


class NetworkDiagnosticsYaml(Node):
    def __init__(self):
        super().__init__("network_diagnostics_yaml")

        # Load YAML
        self.declare_parameter("config_file", "")
        cfg_path = self.get_parameter("config_file").get_parameter_value().string_value
        if not cfg_path:
            self.get_logger().fatal("Set -p config_file:=/abs/path/to/network_diag.yaml")
            raise SystemExit(1)

        import yaml
        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f)["network_diag"]

        self.hw_id = cfg.get("hardware_id", "network")
        self.publish_hz = float(cfg.get("publish_hz", 2.0))
        depth = int(cfg.get("qos_depth", 50))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
        )
        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", qos)
        self.create_timer(max(0.01, 1.0 / self.publish_hz), self._publish)

        self.states: Dict[str, CheckState] = {}
        self.threads: List[threading.Thread] = []
        self._stop = threading.Event()

        # INTERNET
        inet = cfg.get("internet", {})
        if inet.get("enabled", False):
            s = CheckState("Internet Connectivity", stale_s=inet.get("stale_s", 5.0))
            self.states["internet"] = s
            t = threading.Thread(target=self._run_internet, args=(s, inet), daemon=True)
            t.start(); self.threads.append(t)

        # MQTT
        mqtt_cfg = cfg.get("mqtt", {})
        if mqtt_cfg.get("enabled", False):
            if mqtt is None:
                self.get_logger().error("paho-mqtt not installed but MQTT check enabled; disabling MQTT check")
            else:
                s = CheckState("App/MQTT Connectivity", stale_s=mqtt_cfg.get("stale_s", 8.0))
                self.states["mqtt"] = s
                t = threading.Thread(target=self._run_mqtt, args=(s, mqtt_cfg), daemon=True)
                t.start(); self.threads.append(t)

        # Wi-Fi signal
        wifi = cfg.get("wifi_signal", {})
        if wifi.get("enabled", False):
            s = CheckState("Wi-Fi Signal Strength", stale_s=wifi.get("stale_s", 4.0))
            self.states["wifi_signal"] = s
            t = threading.Thread(target=self._run_wifi_signal, args=(s, wifi), daemon=True)
            t.start(); self.threads.append(t)

        # Neighbors
        neigh = cfg.get("neighbors", {})
        if neigh.get("enabled", False):
            s = CheckState("Local Network Devices", stale_s=neigh.get("stale_s", 12.0))
            self.states["neighbors"] = s
            t = threading.Thread(target=self._run_neighbors, args=(s, neigh), daemon=True)
            t.start(); self.threads.append(t)

        # Driver info
        drv = cfg.get("driver", {})
        if drv.get("enabled", False):
            s = CheckState("Network Driver Versions", stale_s=drv.get("stale_s", 90.0))
            self.states["driver"] = s
            t = threading.Thread(target=self._run_driver, args=(s, drv), daemon=True)
            t.start(); self.threads.append(t)

    # ----------------------- helpers -----------------------

    def _sleep_loop(self, interval_s: float) -> bool:
        deadline = now_s() + max(0.05, interval_s)
        while now_s() < deadline:
            if self._stop.is_set():
                return False
            time.sleep(0.05)
        return not self._stop.is_set()

    # ----------------------- Checks -----------------------

    def _run_internet(self, state: CheckState, cfg: Dict[str, Any]):
        host = cfg.get("host", "8.8.8.8")
        interval = float(cfg.get("interval_s", 2.0))
        timeout = float(cfg.get("timeout_s", 1.0))
        warn_ms = cfg.get("latency_warn_ms", None)
        err_ms = cfg.get("latency_error_ms", None)
        count = int(cfg.get("count", 1))
        count = max(1, min(count, 5))

        while not self._stop.is_set():
            try:
                cmd = ["ping", "-n", "-q", "-c", str(count), "-W", str(int(max(1, round(timeout)))), host]
                rc, out, err = _run_cmd(cmd, timeout=timeout + count + 0.5)
                if rc == 0:
                    # packet loss
                    loss = None
                    mloss = re.search(r"(\d+)% packet loss", out)
                    if mloss:
                        loss = int(mloss.group(1))
                    # rtt stats
                    rtt_avg = None
                    m = re.search(r"rtt [^=]+= ([\d\.]+)/([\d\.]+)/([\d\.]+)/([\d\.]+) ms", out)
                    if m:
                        rtt_min, rtt_avg, rtt_max, rtt_mdev = [float(x) for x in m.groups()]
                    else:
                        # single ping: try to parse "time=xx.x ms"
                        m2 = re.search(r"time[=<]([\d\.]+)\s*ms", out)
                        rtt_avg = float(m2.group(1)) if m2 else None

                    level = DiagnosticStatus.OK
                    msg = "reachable"
                    kvs = [("host", host)]
                    if loss is not None:
                        kvs.append(("loss_percent", loss))
                        if loss > 0:
                            level = max(level, DiagnosticStatus.WARN)
                            msg = "packet loss"
                    if rtt_avg is not None:
                        kvs.append(("rtt_ms", f"{rtt_avg:.1f}"))
                        if err_ms is not None and rtt_avg >= float(err_ms):
                            level = DiagnosticStatus.ERROR; msg = "high latency"
                        elif warn_ms is not None and rtt_avg >= float(warn_ms):
                            level = max(level, DiagnosticStatus.WARN); msg = "latency warning"
                    state.set(level, msg, kvs)
                else:
                    state.set(DiagnosticStatus.ERROR, "unreachable", [("host", host), ("rc", rc), ("stderr", err.strip())])
            except Exception as e:
                state.set(DiagnosticStatus.ERROR, "ping failed", [("error", str(e))])

            if not self._sleep_loop(interval):
                break

    def _run_mqtt(self, state: CheckState, cfg: Dict[str, Any]):
        host = cfg.get("host", "localhost")
        port = int(cfg.get("port", 1883))
        keepalive = int(cfg.get("keepalive_s", 3))
        interval = float(cfg.get("interval_s", 3.0))
        timeout = float(cfg.get("timeout_s", 2.0))

        transport = str(cfg.get("transport", "")).lower().strip()
        if not transport:
            # Heuristic: port 9001 is usually websockets
            transport = "websockets" if port == 9001 else "tcp"

        while not self._stop.is_set():
            try:
                client = mqtt.Client(transport=transport)
                client._transport = transport  # paho quirk
                client.connect_async(host, port, keepalive=keepalive)
                client.loop_start()
                start = now_s()
                connected = False
                while now_s() - start < timeout:
                    if client.is_connected():
                        connected = True
                        break
                    time.sleep(0.05)
                client.loop_stop()
                try:
                    client.disconnect()
                except Exception:
                    pass

                if connected:
                    state.set(DiagnosticStatus.OK, "connected",
                              [("broker", f"{host}:{port}"), ("transport", transport)])
                else:
                    state.set(DiagnosticStatus.ERROR, "failed",
                              [("broker", f"{host}:{port}"), ("transport", transport)])
            except Exception as e:
                state.set(DiagnosticStatus.ERROR, "failed",
                          [("broker", f"{host}:{port}"), ("transport", transport), ("error", str(e))])

            if not self._sleep_loop(interval):
                break

    def _parse_rssi(self, text: str) -> Optional[int]:
        m = re.search(r"signal[:=]\s*(-?\d+)\s*dBm", text, flags=re.IGNORECASE)
        if m: return int(m.group(1))
        m = re.search(r"Signal level[=]\s*(-?\d+)\s*dBm", text, flags=re.IGNORECASE)
        if m: return int(m.group(1))
        return None

    def _run_wifi_signal(self, state: CheckState, cfg: Dict[str, Any]):
        iface = str(cfg.get("iface", "wlan0"))
        if not iface or iface.lower() == "auto":
            ai = _auto_wifi_iface()
            if ai:
                iface = ai

        interval = float(cfg.get("interval_s", 1.0))
        warn_dbm = cfg.get("rssi_warn_dbm", None)
        err_dbm = cfg.get("rssi_error_dbm", None)

        while not self._stop.is_set():
            try:
                # Prefer 'iw dev IF link'
                rc, out, err = _run_cmd(["iw", "dev", iface, "link"], timeout=1.0)
                if rc != 0 or "Not connected." in out:
                    # fallback to iwconfig
                    rc2, out2, err2 = _run_cmd(["iwconfig", iface], timeout=1.0)
                    out = out2; err = err2; rc = rc2

                if rc != 0:
                    msg = "iface missing or down"
                    if "No such device" in err:
                        msg = "iface not found"
                    state.set(DiagnosticStatus.ERROR, msg, [("iface", iface), ("error", err.strip())])
                else:
                    rssi = self._parse_rssi(out)
                    if rssi is None:
                        state.set(DiagnosticStatus.ERROR, "no RSSI info", [("iface", iface)])
                    else:
                        lvl = DiagnosticStatus.OK
                        msg = "good"
                        if err_dbm is not None and rssi <= int(err_dbm):
                            lvl = DiagnosticStatus.ERROR; msg = "poor"
                        elif warn_dbm is not None and rssi <= int(warn_dbm):
                            lvl = DiagnosticStatus.WARN; msg = "fair"
                        state.set(lvl, msg, [("iface", iface), ("rssi_dbm", rssi)])
            except Exception as e:
                state.set(DiagnosticStatus.ERROR, "failed", [("iface", iface), ("error", str(e))])

            if not self._sleep_loop(interval):
                break

    def _run_neighbors(self, state: CheckState, cfg: Dict[str, Any]):
        method = cfg.get("method", "ip_neigh")
        interval = float(cfg.get("interval_s", 5.0))
        warn_low = cfg.get("warn_low", None)
        error_low = cfg.get("error_low", None)

        while not self._stop.is_set():
            try:
                if method == "arp":
                    rc, out, _ = _run_cmd(["arp", "-an"], timeout=1.5)
                    lines = [l for l in out.splitlines() if l.strip()] if rc == 0 else []
                else:
                    rc, out, _ = _run_cmd(["ip", "neigh", "show"], timeout=1.5)
                    lines = [l for l in out.splitlines() if l.strip()] if rc == 0 else []
                count = len(lines)

                lvl = DiagnosticStatus.OK
                msg = f"{count} devices"
                if error_low is not None and count <= int(error_low):
                    lvl = DiagnosticStatus.ERROR; msg = "too few neighbors"
                elif warn_low is not None and count <= int(warn_low):
                    lvl = DiagnosticStatus.WARN; msg = "low neighbors"
                state.set(lvl, msg, [("device_count", count)])
            except Exception as e:
                state.set(DiagnosticStatus.ERROR, "failed", [("error", str(e))])

            if not self._sleep_loop(interval):
                break

    def _run_driver(self, state: CheckState, cfg: Dict[str, Any]):
        iface = cfg.get("iface", "wlan0")
        interval = float(cfg.get("interval_s", 30.0))

        while not self._stop.is_set():
            try:
                rc, out, err = _run_cmd(["ethtool", "-i", iface], timeout=2.0)
                if rc == 0:
                    kvs: List[Tuple[str, Any]] = []
                    for line in out.splitlines():
                        if ":" in line:
                            k, v = [s.strip() for s in line.split(":", 1)]
                            if k in ("driver", "version", "firmware-version", "bus-info"):
                                kvs.append((k, v))
                    state.set(DiagnosticStatus.OK, "queried", kvs or [("iface", iface)])
                else:
                    # rc 71 often means Operation not supported — downgrade to WARN with hint.
                    level = DiagnosticStatus.WARN if rc == 71 else DiagnosticStatus.ERROR
                    msg = "not supported" if rc == 71 else "failed"
                    state.set(level, msg, [("iface", iface), ("rc", rc), ("error", err.strip())])
            except Exception as e:
                state.set(DiagnosticStatus.ERROR, "failed", [("iface", iface), ("error", str(e))])

            if not self._sleep_loop(interval):
                break

    # ----------------------- Publisher (2 Hz) -----------------------

    def _publish(self):
        da = DiagnosticArray()
        da.header.stamp = self.get_clock().now().to_msg()

        for key, state in self.states.items():
            level, message, kvs, stamp_s, stale_s = state.snapshot()

            st = DiagnosticStatus()
            st.name = state.name
            st.hardware_id = self.hw_id

            age = now_s() - stamp_s if stamp_s > 0 else float("inf")
            kv(st, "age_s", f"{age:.2f}")

            if not (age < float("inf")):
                st.level = DiagnosticStatus.STALE
                st.message = "no data"
            elif age > max(0.0, stale_s):
                st.level = DiagnosticStatus.STALE
                st.message = f"stale ({age:.2f}s)"
            else:
                st.level = level
                st.message = message

            for k, v in kvs:
                kv(st, k, v)

            da.status.append(st)

        self.pub.publish(da)

    # ----------------------- Shutdown -----------------------
    def destroy_node(self):
        self._stop.set()
        for t in self.threads:
            try:
                t.join(timeout=1.0)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NetworkDiagnosticsYaml()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
