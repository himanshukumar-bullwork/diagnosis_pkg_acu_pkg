#!/usr/bin/env python3
import math
import threading
from typing import Dict, Any, Optional, Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import yaml
import can  # python-can


def kv(st: DiagnosticStatus, key: str, value: Any):
    st.values.append(KeyValue(key=key, value=str(value)))


def parse_id(s: str) -> int:
    s = str(s).strip()
    return int(s, 16) if s.lower().startswith("0x") else int(s, 10)


def _to_int(x) -> Optional[int]:
    try:
        if isinstance(x, str):
            x = x.strip()
            if x.lower().startswith("0x"):
                return int(x, 16)
            return int(x)
        return int(x)
    except Exception:
        return None


def _to_int_list(xs: Iterable) -> Optional[list]:
    if xs is None:
        return None
    out = []
    for x in (xs if isinstance(xs, (list, tuple)) else [xs]):
        v = _to_int(x)
        if v is None:
            return None
        out.append(v)
    return out


class _CanListener(can.Listener):
    """python-can listener updating shared caches for only the IDs we care about."""
    def __init__(self, node: Node, wanted_ids: set, cache_data: Dict[int, bytes],
                 cache_stamp: Dict[int, rclpy.time.Time], lock: threading.Lock):
        self.node = node
        self.wanted_ids = wanted_ids
        self.cache_data = cache_data
        self.cache_stamp = cache_stamp
        self.lock = lock

    def on_message_received(self, msg: can.Message):
        arb_id = msg.arbitration_id
        if arb_id in self.wanted_ids:
            with self.lock:
                self.cache_data[arb_id] = bytes(msg.data)
                self.cache_stamp[arb_id] = self.node.get_clock().now()


class CanDiagnosticsPythonCan(Node):
    def __init__(self):
        super().__init__('can_diagnostics_pythoncan')

        # ---- params / config ----
        self.declare_parameter('config_file', '')
        cfg_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not cfg_path:
            self.get_logger().fatal('Set param: -p config_file:=/abs/path/to/can_diag.yaml')
            raise SystemExit(1)

        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)['can_diag']

        bus_cfg = cfg.get('bus', {})
        self.interface = bus_cfg.get('interface', 'socketcan')
        self.channel = bus_cfg.get('channel', 'can0')
        self.bitrate = bus_cfg.get('bitrate', None)  # usually set via `ip link` for socketcan

        self.publish_hz = float(cfg.get('publish_hz', 2.0))
        self.stale_factor = float(cfg.get('stale_factor', 1.5))
        hw_prefix = cfg.get('hardware_id_prefix', 'CAN')
        self.hardware_id = f"{hw_prefix}:{self.channel}"

        # IDs table
        self.id_cfg: Dict[int, Dict[str, Any]] = {}
        for k, v in (cfg.get('ids') or {}).items():
            self.id_cfg[parse_id(k)] = v
        self.wanted_ids = set(self.id_cfg.keys())

        # caches
        self.latest_data: Dict[int, Optional[bytes]] = {cid: None for cid in self.wanted_ids}
        self.latest_stamp: Dict[int, Optional[rclpy.time.Time]] = {cid: None for cid in self.wanted_ids}
        self.lock = threading.Lock()

        # ---- CAN bus ----
        self.bus = None
        try:
            bus_kwargs = dict(bustype=self.interface, channel=self.channel)
            if self.bitrate and self.interface not in ('socketcan',):
                bus_kwargs['bitrate'] = int(self.bitrate)
            self.bus = can.Bus(**bus_kwargs)
            try:
                filters = [{'can_id': cid, 'can_mask': 0x7FF, 'extended': False} for cid in self.wanted_ids]
                if filters:
                    self.bus.set_filters(filters)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f'CAN open failed on {self.interface}:{self.channel} — {e}')

        self.notifier = None
        if self.bus is not None:
            self.notifier = can.Notifier(
                self.bus,
                [_CanListener(self, self.wanted_ids, self.latest_data, self.latest_stamp, self.lock)],
                timeout=0.2
            )

        # ---- ROS pub/timer ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', qos)
        self.timer = self.create_timer(max(0.001, 1.0 / self.publish_hz), self._publish)

    # --------- value decoding helpers ----------
    @staticmethod
    def _extract(data: bytes, start: int, length: int, endian: str, signed: bool) -> Optional[int]:
        if data is None:
            return None
        if start < 0 or length <= 0 or start + length > len(data):
            return None
        order = 'little' if str(endian).lower().startswith('l') else 'big'
        try:
            return int.from_bytes(data[start:start + length], byteorder=order, signed=bool(signed))
        except Exception:
            return None

    @staticmethod
    def _apply_scale(raw: Optional[int], scale: float, offset: float) -> Optional[float]:
        if raw is None:
            return None
        return raw * float(scale) + float(offset)

    def _grade_thresholds(
        self,
        val: Optional[float],
        th: Any,
        raw: Optional[int] = None
    ) -> Optional[int]:
        """
        Evaluate thresholds; return DiagnosticStatus level override or None.

        Supported keys (all optional):
          - error_low / error_high / warn_low / warn_high
          - error_mask_any / warn_mask_any       (bitwise check on RAW value)
          - error_equals / warn_equals           (exact match on RAW or scaled; uses RAW if present)
          - error_not_in / warn_not_in           (RAW or scaled in allowed-set; raise if not in)

        Notes:
          * 'error_*' dominates 'warn_*'
          * If a key is malformed, it's ignored (no crash).
        """
        if th is None or not isinstance(th, dict):
            return None

        # --- mask checks on RAW ---
        if raw is not None:
            m_err = _to_int(th.get('error_mask_any'))
            if m_err is not None and (raw & m_err) != 0:
                return DiagnosticStatus.ERROR
            m_warn = _to_int(th.get('warn_mask_any'))
            if m_warn is not None and (raw & m_warn) != 0:
                return DiagnosticStatus.WARN

            # equals / not-in on RAW first (if provided)
            err_eq = _to_int_list(th.get('error_equals'))
            if err_eq is not None and raw in err_eq:
                return DiagnosticStatus.ERROR
            warn_eq = _to_int_list(th.get('warn_equals'))
            if warn_eq is not None and raw in warn_eq:
                return DiagnosticStatus.WARN

            err_notin = _to_int_list(th.get('error_not_in'))
            if err_notin is not None and raw not in err_notin:
                return DiagnosticStatus.ERROR
            warn_notin = _to_int_list(th.get('warn_not_in'))
            if warn_notin is not None and raw not in warn_notin:
                return DiagnosticStatus.WARN

        # --- numeric low/high on SCALED value ---
        if val is None:
            return None

        try:
            # ERROR dominates
            lo = th.get('error_low');  hi = th.get('error_high')
            if lo is not None and val <= float(lo):   return DiagnosticStatus.ERROR
            if hi is not None and val >= float(hi):   return DiagnosticStatus.ERROR

            warned = False
            lo = th.get('warn_low');   hi = th.get('warn_high')
            if lo is not None and val <= float(lo):   warned = True
            if hi is not None and val >= float(hi):   warned = True
            return DiagnosticStatus.WARN if warned else None
        except Exception:
            # Malformed numbers → ignore
            return None

    # ------------- main publish tick -------------
    def _publish(self):
        now = self.get_clock().now()
        da = DiagnosticArray()
        da.header.stamp = now.to_msg()

        # Bus health
        bus_st = DiagnosticStatus()
        bus_st.hardware_id = self.hardware_id
        bus_st.name = 'can_bus'
        if self.bus is None:
            bus_st.level = DiagnosticStatus.ERROR
            bus_st.message = 'bus not open'
        else:
            bus_st.level = DiagnosticStatus.OK
            bus_st.message = 'bus open'
            try:
                state = getattr(self.bus, 'state', None)
                if state is not None:
                    kv(bus_st, 'bus_state', state)
            except Exception:
                pass
        da.status.append(bus_st)

        # Snapshot caches
        with self.lock:
            snap_d = dict(self.latest_data)
            snap_t = dict(self.latest_stamp)

        # Per-ID statuses
        for cid, meta in self.id_cfg.items():
            name = meta.get('name', f'id_{cid}')
            expected_hz = float(meta.get('expected_hz', 1.0))
            period = 1.0 / max(1e-6, expected_hz)
            stale_after = period * self.stale_factor

            st = DiagnosticStatus()
            st.hardware_id = self.hardware_id
            st.name = name
            st.level = DiagnosticStatus.OK
            st.message = 'OK'

            data = snap_d.get(cid)
            stamp = snap_t.get(cid)

            kv(st, 'id', f'0x{cid:03X}')
            kv(st, 'expected_hz', expected_hz)

            if self.bus is None:
                st.level = DiagnosticStatus.ERROR
                st.message = 'bus closed'
                da.status.append(st)
                continue

            if data is None or stamp is None:
                st.level = DiagnosticStatus.WARN
                st.message = 'no data yet'
                da.status.append(st)
                continue

            age = (now - stamp).nanoseconds * 1e-9
            kv(st, 'age_s', f'{age:.3f}')

            if age > stale_after:
                st.level = DiagnosticStatus.STALE
                st.message = f'stale ({age:.2f}s)'
                da.status.append(st)
                continue

            # Decode configured fields (if any)
            fields = meta.get('fields', [])
            worst = 0  # 0=OK, 1=WARN, 2=ERROR
            if fields:
                for fd in fields:
                    key    = fd.get('key', 'field')
                    start  = int(fd.get('start', 0))   # 0-based
                    length = int(fd.get('length', 1))
                    endian = fd.get('endian', 'little')
                    signed = bool(fd.get('signed', False))
                    scale  = float(fd.get('scale', 1.0))
                    offset = float(fd.get('offset', 0.0))
                    units  = fd.get('units', '')

                    raw = self._extract(data, start, length, endian, signed)
                    val = self._apply_scale(raw, scale, offset) if raw is not None else None

                    if raw is None:
                        kv(st, key, 'decode_error')
                        continue  # DO NOT change severity if no threshold & decode fails
                    else:
                        kv(st, f'{key}_raw', raw)
                        if isinstance(val, float):
                            kv(st, key, f'{val:.3f}')
                        else:
                            kv(st, key, val)
                        if units:
                            kv(st, f'{key}_units', units)

                    # Thresholds (optional)
                    th = fd.get('thresholds', {})
                    override = self._grade_thresholds(
                        val if isinstance(val, (int, float)) else None,
                        th,
                        raw=raw
                    )
                    if override is not None:
                        # Ensure integer and compare safely
                        try:
                            worst = max(worst, int(override))
                        except Exception:
                            pass
            else:
                # No fields defined: still provide raw bytes for visibility
                kv(st, 'raw', ' '.join(f'0x{b:02X}' for b in data))

            # Apply per-field severity (only if not stale)
            if worst == DiagnosticStatus.ERROR:
                st.level = DiagnosticStatus.ERROR
                if st.message == 'OK':
                    st.message = 'field threshold error'
            elif worst == DiagnosticStatus.WARN:
                st.level = max(st.level, DiagnosticStatus.WARN)
                if st.message == 'OK':
                    st.message = 'field threshold warn'

            da.status.append(st)

        self.pub.publish(da)

    def destroy_node(self):
        try:
            if self.notifier is not None:
                self.notifier.stop()
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanDiagnosticsPythonCan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
