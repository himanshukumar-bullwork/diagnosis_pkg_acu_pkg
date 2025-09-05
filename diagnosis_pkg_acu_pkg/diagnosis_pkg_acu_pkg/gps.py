#!/usr/bin/env python3
# Hardened diagnostics: condition first, compare as ints, assign level as bytes
import math, os
from glob import glob
from typing import Any, Dict, Optional, Tuple, List, Type

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from ublox_ubx_msgs.msg import UBXNavPVT, UBXNavRelPosNED
try:
    from ublox_ubx_msgs.msg import UBXRxmRTCM
except Exception:
    UBXRxmRTCM = None

TYPE_MAP: Dict[str, Optional[Type]] = {
    "NavPVT": UBXNavPVT,
    "NavRELPOSNED": UBXNavRelPosNED,
    "RxmRTCM": UBXRxmRTCM,
}

# ---- severity ints (internal)
LVL_OK, LVL_WARN, LVL_ERROR, LVL_STALE = 0, 1, 2, 3

# ---- coercers (same logic as your working echo)
def s_int(x, default: Optional[int] = None) -> Optional[int]:
    if isinstance(x, bool): return 1 if x else 0
    if isinstance(x, int): return x
    if isinstance(x, (bytes, bytearray)):
        if len(x) == 1: return int(x[0])
        try: return int(x.decode('ascii').strip())
        except Exception:
            try: return int.from_bytes(bytes(x), 'little', signed=False)
            except Exception: return default
    try:
        import numpy as np
        if isinstance(x, np.integer):  # type: ignore[attr-defined]
            return int(x.item())
    except Exception:
        pass
    try:
        if isinstance(x, str): return int(x.strip())
        return int(x)
    except Exception:
        return default

def s_float(x, default: Optional[float] = None) -> Optional[float]:
    if isinstance(x, (float, int, bool)): return float(x)
    iv = s_int(x, None)
    if iv is not None: return float(iv)
    if isinstance(x, str):
        try: return float(x.strip())
        except Exception: return default
    return default

def s_str(x) -> str:
    if isinstance(x, (bytes, bytearray)) and len(x) == 1:
        return str(int(x[0]))
    return str(x)

def kv(st: DiagnosticStatus, k: str, v: Any):
    st.values.append(KeyValue(key=s_str(k), value=s_str(v)))

def set_level(st: DiagnosticStatus, lvl_int: int):
    """Assign level compatibly (ROS builds where level is byte expect bytes len=1)."""
    i = max(0, min(255, int(lvl_int)))
    try:
        st.level = bytes([i])    # ROS1-style 'byte'
    except Exception:
        st.level = i             # ROS2 'uint8'

def _try_get(obj, path: str):
    cur = obj
    for part in path.split('.'):
        if not hasattr(cur, part):
            return None
        cur = getattr(cur, part)
    return cur

def get_attr_any(obj, expr: Optional[Any]):
    if expr is None: return None
    if isinstance(expr, (list, tuple)):
        candidates = [str(x) for x in expr]
    elif isinstance(expr, str) and '|' in expr:
        candidates = [p.strip() for p in expr.split('|') if p.strip()]
    else:
        candidates = [str(expr)]
    for p in candidates:
        v = _try_get(obj, p)
        if v is not None:
            return v
    return None

def _make_qos_sensor(depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=max(5, depth),
    )

def _make_qos_reliable(depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=max(10, depth),
    )

def _merge_level(cur: Any, cand: Optional[Any]) -> int:
    ci = s_int(cur, 0) or 0
    if cand is None: return ci
    return max(ci, s_int(cand, 0) or 0)

class TopicMon:
    def __init__(self, node: Node, topic: str, msg_type: Type, qos: QoSProfile):
        self.node = node
        self.topic = topic
        self.last_msg = None
        self.last_stamp = None
        self.sub = node.create_subscription(msg_type, topic, self._cb, qos)
        node.get_logger().info(
            f"[gps_diag] subscribe {topic} "
            f"(reliability={'BEST_EFFORT' if qos.reliability == ReliabilityPolicy.BEST_EFFORT else 'RELIABLE'}, "
            f"depth={qos.depth})"
        )
    def _cb(self, msg): self.last_msg = msg; self.last_stamp = self.node.get_clock().now()
    def age_s(self) -> float:
        if self.last_stamp is None: return math.inf
        return (self.node.get_clock().now() - self.last_stamp).nanoseconds * 1e-9

class GpsDiagnosticsYaml(Node):
    def __init__(self):
        super().__init__('gps_diagnostics_yaml')
        self.declare_parameter('config_file', '')
        cfg_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not cfg_path:
            self.get_logger().fatal('Set param: -p config_file:=/abs/path/to/gps_diag.yaml')
            raise SystemExit(1)

        import yaml
        with open(cfg_path, 'r') as f:
            cfg_all = yaml.safe_load(f) or {}
        cfg = cfg_all.get('gps_diag', {}) or {}

        self.publish_hz = float(cfg.get('publish_hz', 2.0))
        self.compute_hz = 5.0
        self.stale_factor = float(cfg.get('stale_factor', 1.5))
        self.default_stale_s = float(cfg.get('default_stale_s', 1.5))
        self.hw_prefix = s_str(cfg.get('hardware_id_prefix', 'GPS'))
        default_depth = int(s_int(cfg.get('qos_depth', 10), 10))

        self.devices_cfg: Dict[str, Dict[str, str]] = cfg.get('devices', {}) or {}
        self.usb_rescan_period_s = float(cfg.get('usb_rescan_period_s', 3.0))
        self._usb_last_scan_secs = 0.0
        self._usb_last_result: Dict[str, bool] = {}

        self.comps = cfg.get('components', {}) or {}
        self.monitors: Dict[str, TopicMon] = {}
        for cname, c in self.comps.items():
            topic = s_str(c['topic'])
            ttype = s_str(c['type'])
            msg_cls = TYPE_MAP.get(ttype)
            if msg_cls is None:
                self.get_logger().warn(f"[gps_diag] component '{cname}': type '{ttype}' not available; skipping")
                continue
            qos_str = s_str(c.get('qos', 'sensor')).strip().lower()
            depth = int(s_int(c.get('qos_depth', default_depth), default_depth))
            qos = _make_qos_sensor(depth) if qos_str in ('sensor', 'best_effort', 'be') else _make_qos_reliable(depth)
            self.monitors[cname] = TopicMon(self, topic, msg_cls, qos)

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', _make_qos_reliable(10))
        self._last_status: List[DiagnosticStatus] = []
        self.create_timer(max(0.01, 1.0 / self.compute_hz), self._compute_once_safe)
        self.create_timer(max(0.01, 1.0 / self.publish_hz), self._publish_last_safe)

    # ---- grading (use ONLY floats/ints)
    @staticmethod
    def _grade_thresholds(val: Optional[float], th: Dict[str, Any]) -> Optional[int]:
        if val is None or not th: return None
        try:
            eh = s_float(th.get('error_high', None), None)
            el = s_float(th.get('error_low',  None), None)
            wh = s_float(th.get('warn_high',  None), None)
            wl = s_float(th.get('warn_low',   None), None)
            if el is not None and val <= el: return LVL_ERROR
            if eh is not None and val >= eh: return LVL_ERROR
            warned = False
            if wl is not None and val <= wl: warned = True
            if wh is not None and val >= wh: warned = True
            return LVL_WARN if warned else None
        except Exception:
            return None

    @staticmethod
    def _enum_level(v: Any, enum_levels: Optional[Dict[str, str]]) -> Optional[int]:
        if not enum_levels: return None
        key = s_int(v, None)
        key_s = str(key) if key is not None else s_str(v)
        lbl = enum_levels.get(key_s)
        if not lbl: return None
        return {"OK": LVL_OK, "WARN": LVL_WARN, "ERROR": LVL_ERROR, "STALE": LVL_STALE}.get(lbl.upper(), None)

    # ---- USB presence
    def _scan_usb_serials(self, vendor: str, product: str) -> List[str]:
        found: List[str] = []
        for devdir in glob('/sys/bus/usb/devices/*'):
            if ':' in os.path.basename(devdir): continue
            idv, idp, ser = os.path.join(devdir, 'idVendor'), os.path.join(devdir, 'idProduct'), os.path.join(devdir, 'serial')
            try:
                if not (os.path.exists(idv) and os.path.exists(idp)): continue
                with open(idv) as f: v = f.read().strip().lower()
                with open(idp) as f: p = f.read().strip().lower()
                if v != vendor.lower() or p != product.lower(): continue
                if os.path.exists(ser):
                    with open(ser) as f: s = f.read().strip()
                    if s: found.append(s)
            except Exception:
                continue
        return found

    def _usb_presence_check(self) -> Dict[str, bool]:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._usb_last_scan_secs) < self.usb_rescan_period_s and self._usb_last_result:
            return self._usb_last_result
        result: Dict[str, bool] = {}
        vp_groups: Dict[Tuple[str, str], List[Tuple[str, str]]] = {}
        for label, d in self.devices_cfg.items():
            vid = s_str(d.get('vendor_id', '')).lower()
            pid = s_str(d.get('product_id', '')).lower()
            ser = s_str(d.get('serial', ''))
            vp_groups.setdefault((vid, pid), []).append((label, ser))
        for (vid, pid), entries in vp_groups.items():
            serials = self._scan_usb_serials(vid, pid)
            for label, expected_ser in entries:
                present = expected_ser in serials if expected_ser else (len(serials) > 0)
                result[label] = present
        self._usb_last_scan_secs = now
        self._usb_last_result = result
        return result

    # ---- component evaluation (compute in ints/floats; assign level at end)
    def _eval_component(self, cname: str, comp: Dict[str, Any], mon: TopicMon) -> DiagnosticStatus:
        st = DiagnosticStatus()
        st.name = s_str(comp.get('name', cname))
        st.hardware_id = s_str(comp.get('hardware_id', f"{self.hw_prefix}"))

        expected_hz = s_float(comp.get('expected_hz', 0.0), 0.0) or 0.0
        stale_s = s_float(comp.get('stale_s', 0.0), 0.0) or 0.0
        if stale_s <= 0.0:
            stale_s = (1.0 / expected_hz) * self.stale_factor if expected_hz > 0 else self.default_stale_s

        msg = mon.last_msg
        age = mon.age_s()
        if math.isinf(age):
            set_level(st, LVL_STALE); st.message = 'no data'; return st
        kv(st, 'age_s', f'{age:.2f}')
        if age > stale_s:
            set_level(st, LVL_STALE); st.message = f'stale ({age:.2f}s)'; return st

        lvl_accum = LVL_OK
        st.message = 'OK'

        for fd in (comp.get('fields') or []):
            try:
                key    = s_str(fd.get('key', 'field'))
                expr   = fd.get('expr') or fd.get('expr_any')
                scale  = s_float(fd.get('scale', 1.0), 1.0) or 1.0
                offset = s_float(fd.get('offset', 0.0), 0.0) or 0.0
                units  = s_str(fd.get('units', ''))
                th     = fd.get('thresholds', {}) or {}
                enums  = fd.get('enum_levels', None)
                labels = fd.get('enum_labels', None)

                raw = get_attr_any(msg, expr) if expr else None
                if raw is None:
                    kv(st, key, 'unavailable')
                    if req:
                        lvl_accum = _merge_level(lvl_accum, LVL_ERROR if str(req).lower() != 'warn' else LVL_WARN)
                        continue

                num = s_float(raw, None)
                val = num * scale + offset if num is not None else raw

                if isinstance(val, float):
                    kv(st, key, f'{val:.6f}')
                else:
                    u8 = s_int(raw, None)
                    kv(st, key, (str(u8) if u8 is not None else s_str(val)))
                if units: kv(st, f'{key}_units', units)

                if labels is not None:
                    k8 = s_int(raw, None)
                    lab = labels.get(str(k8) if k8 is not None else s_str(raw))
                    if lab is not None: kv(st, f'{key}_label', lab)

                enum_sev = self._enum_level(raw, enums)
                lvl_accum = _merge_level(lvl_accum, enum_sev)

                thr_val = (num * scale + offset) if num is not None else None
                thr_sev = self._grade_thresholds(thr_val, th)
                lvl_accum = _merge_level(lvl_accum, thr_sev)

            except Exception as e:
                lvl_accum = _merge_level(lvl_accum, LVL_WARN)
                if st.message == 'OK': st.message = 'field parse warning'
                kv(st, 'field_error', f"{key}: {e!s}")

        if lvl_accum >= LVL_ERROR:
            st.message = 'threshold error'
            set_level(st, LVL_ERROR)
        elif lvl_accum >= LVL_WARN:
            st.message = 'threshold warn'
            set_level(st, LVL_WARN)
        else:
            set_level(st, LVL_OK)

        return st

    # ---- compute/publish with guards
    def _compute_once_safe(self):
        try:
            status_list: List[DiagnosticStatus] = []
            for cname, comp in self.comps.items():
                mon = self.monitors.get(cname)
                if mon is None: continue
                try:
                    status_list.append(self._eval_component(cname, comp, mon))
                except Exception as e:
                    st = DiagnosticStatus()
                    st.name = s_str(comp.get('name', cname))
                    st.hardware_id = s_str(comp.get('hardware_id', f"{self.hw_prefix}"))
                    st.message = f'component exception: {e}'
                    set_level(st, LVL_ERROR)
                    kv(st, 'exception_type', type(e).__name__)
                    status_list.append(st)

            # USB presence
            if self.devices_cfg:
                presence = self._usb_presence_check()
                for label in ('base', 'rover'):
                    if label in self.devices_cfg:
                        d = self.devices_cfg[label]
                        expected_ser = s_str(d.get('serial', ''))
                        vid = s_str(d.get('vendor_id', ''))
                        pid = s_str(d.get('product_id', ''))
                        ok = bool(presence.get(label, False))

                        st = DiagnosticStatus()
                        st.name = f"GPS Device ({label})"
                        st.hardware_id = "GPS_USB"
                        st.message = "connected" if ok else "device not connected"
                        set_level(st, LVL_OK if ok else LVL_ERROR)
                        kv(st, 'vendor_id', vid); kv(st, 'product_id', pid)
                        if expected_ser: kv(st, 'expected_serial', expected_ser)
                        kv(st, 'present', ok)
                        status_list.append(st)

                if any(not presence.get(lbl, True) for lbl in self.devices_cfg.keys()):
                    st = DiagnosticStatus()
                    st.name = "GPS Devices Overall"
                    st.hardware_id = "GPS_USB"
                    st.message = "one or more GPS devices missing"
                    set_level(st, LVL_ERROR)
                    for lbl, ok in presence.items():
                        kv(st, f"{lbl}_present", ok)
                    status_list.append(st)

            # Final sanitize of strings (levels already set via set_level)
            for st in status_list:
                st.name = s_str(st.name)
                st.message = s_str(st.message)
                st.hardware_id = s_str(st.hardware_id)
                for kvp in st.values:
                    kvp.key = s_str(kvp.key); kvp.value = s_str(kvp.value)

            self._last_status = status_list

        except Exception as e:
            self.get_logger().error(f'compute exception (caught): {e}')
            st = DiagnosticStatus()
            st.name = "Diagnostics Compute"; st.hardware_id = "GPS_DIAG"
            st.message = f"compute exception: {e}"
            set_level(st, LVL_ERROR)
            self._last_status = [st]

    def _publish_last_safe(self):
        try:
            da = DiagnosticArray()
            da.header.stamp = self.get_clock().now().to_msg()
            da.status = list(self._last_status)
            self.pub.publish(da)
        except Exception as e:
            self.get_logger().error(f'publish exception (caught): {e}')

# ---- safe main
def _safe_shutdown():
    try:
        ctx = rclpy.get_default_context()
        if ctx is not None and ctx.ok():
            rclpy.shutdown()
    except Exception:
        pass

def main(args=None):
    ret = 0
    rclpy.init(args=args)
    node = GpsDiagnosticsYaml()
    exec_ = SingleThreadedExecutor()
    try:
        exec_.add_node(node); exec_.spin()
    except KeyboardInterrupt:
        node.get_logger().info('GPS diagnostics: Ctrl-C received, shutting down cleanly...')
        ret = 0
    except Exception as e:
        node.get_logger().error(f'GPS diagnostics: unhandled exception: {e}')
        ret = 1
    finally:
        try: exec_.remove_node(node)
        except Exception: pass
        try: node.destroy_node()
        except Exception: pass
        _safe_shutdown()
    import sys; sys.exit(ret)

if __name__ == '__main__':
    main()
