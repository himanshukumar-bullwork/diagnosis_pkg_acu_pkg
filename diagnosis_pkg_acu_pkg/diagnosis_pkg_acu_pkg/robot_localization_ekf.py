#!/usr/bin/env python3
# ekf_diagnostics.py (accepts ekf_diag.nodes schema)

import math
import time
from collections import deque
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

import tf2_ros
import tf_transformations as tft
import yaml


def kv(status: DiagnosticStatus, key: str, value):
    status.values.append(KeyValue(key=key, value=str(value)))


def quat_to_yaw_deg(q):
    yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    return math.degrees(yaw)


class TopicRateMonitor:
    """Track rate and age for a topic. Optionally keep last msg for snapshot."""
    def __init__(self, node: Node, topic: str, msg_type, header_based=True, keep_last=False, window=50):
        self.node = node
        self.topic = topic
        self.header_based = header_based
        self.keep_last = keep_last
        self.stamps = deque(maxlen=window)
        self.last_age = None
        self.last_msg = None
        self.sub = node.create_subscription(msg_type, topic, self._cb, 10)

    def _cb(self, msg):
        now = self.node.get_clock().now()
        if self.header_based and hasattr(msg, 'header'):
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        else:
            stamp = now
        self.stamps.append(stamp)
        self.last_age = (now - stamp).nanoseconds * 1e-9
        if self.keep_last:
            self.last_msg = msg

    def rate_hz(self):
        if len(self.stamps) < 2:
            return 0.0
        xs = list(self.stamps)[-10:] if len(self.stamps) > 10 else list(self.stamps)
        dt = (xs[-1] - xs[0]).nanoseconds * 1e-9
        if dt <= 0:
            return 0.0
        return (len(xs) - 1) / dt

    def age_s(self):
        return self.last_age if self.last_age is not None else math.inf


class TFMonitor:
    def __init__(self, node: Node):
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.listener = tf2_ros.TransformListener(self.buf, node)

    def age(self, target: str, source: str):
        """Return (age_seconds, ok). If TF missing, (inf, False)."""
        try:
            tf = self.buf.lookup_transform(target, source, rclpy.time.Time())
            now = rclpy.time.Time()
            age = (now - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            return max(0.0, age), True
        except Exception:
            return math.inf, False


class EkfDiagnostics(Node):
    def __init__(self):
        super().__init__('ekf_diagnostics')

        # --- Load YAML ---
        self.declare_parameter('config_file', '')
        cfg_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not cfg_path:
            self.get_logger().fatal('Set param: config_file:=/abs/path/to/ekf_diag.yaml')
            raise SystemExit(1)

        with open(cfg_path, 'r') as f:
            root = yaml.safe_load(f) or {}
        self.cfg: Dict[str, Any] = root.get('ekf_diag', {}) or {}

        # --- Root thresholds & publish rate ---
        self.publish_hz = float(self.cfg.get('publish_hz', 2.0))
        self.tf_warn_age = float(self.cfg.get('tf_warn_s', 0.3))
        self.tf_err_age  = float(self.cfg.get('tf_error_s', 0.8))

        # Defaults used when node-level thresholds are missing
        self.default_rate_warn = 10.0
        self.default_rate_err  = 2.0
        self.default_age_stale = 0.5

        # Extra (optional) uncertainty / jump thresholds
        th = self.cfg.get('thresholds', {})
        self.std_xy_warn  = float(th.get('std_xy_warn_m', 1.0))
        self.std_xy_err   = float(th.get('std_xy_error_m', 3.0))
        self.std_yaw_warn = float(th.get('std_yaw_warn_deg', 5.0))
        self.std_yaw_err  = float(th.get('std_yaw_error_deg', 15.0))
        self.max_xy_jump  = float(th.get('max_xy_jump_m', 1.0))
        self.max_yaw_jump = float(th.get('max_yaw_jump_deg', 20.0))

        self.debounce_s = float(self.cfg.get('debounce_s', 1.0))

        self.tfmon = TFMonitor(self)
        self.topic_mon = {}    # (comp,key)->monitor
        self.last_levels = {}  # comp->(level, since_ts)
        self.prev_pose  = {}   # comp->(x, y, yaw_deg)

        # --- Accept either 'components:' (old) or 'nodes:' (your YAML) ---
        components = self.cfg.get('components')
        if components is None:
            nodes = self.cfg.get('nodes', {})
            components = self._convert_nodes_to_components(nodes)

        if not components:
            self.get_logger().fatal("No components found under 'ekf_diag.components' or 'ekf_diag.nodes'.")
            raise SystemExit(1)

        self.components = components

        # Build subscriptions
        self._setup_component_monitors()

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(max(0.1, 1.0 / self.publish_hz), self._tick)

    # ---------- schema adapter ----------
    def _convert_nodes_to_components(self, nodes: Dict[str, Any]) -> Dict[str, Any]:
        comps: Dict[str, Any] = {}
        for name, cfg in (nodes or {}).items():
            topics = cfg.get('topics', {}) or {}
            thr    = cfg.get('thresholds', {}) or {}
            if name == 'navsat_transform':
                comps[name] = {
                    'kind': 'navsat',
                    'hardware_id': cfg.get('hardware_id', 'localization'),
                    'topics': {
                        'gps_filtered': topics.get('gps_filtered', ''),  # NavSatFix
                        'odom_gps':    topics.get('odometry/gps', topics.get('odom_gps', '')),  # Odometry
                    },
                    'thresholds': {
                        'rate_warn_hz': float(thr.get('rate_warn_hz', self.default_rate_warn/2)),
                        'rate_error_hz': float(thr.get('rate_error_hz', self.default_rate_err/2)),
                        'age_stale_s': float(thr.get('age_stale_s', 1.0)),
                    }
                }
            else:
                # Treat as EKF filter node (odom output)
                odom_topic = topics.get('odometry/local', topics.get('odometry/global', topics.get('odom_filtered', '')))
                comps[name] = {
                    'kind': 'ekf',
                    'hardware_id': cfg.get('hardware_id', 'localization'),
                    'odom_topic': odom_topic,
                    'tf_required': list(cfg.get('tf_required', []) or []),
                    'thresholds': {
                        'rate_warn_hz': float(thr.get('rate_warn_hz', self.default_rate_warn)),
                        'rate_error_hz': float(thr.get('rate_error_hz', self.default_rate_err)),
                        'age_stale_s':  float(thr.get('age_stale_s',  self.default_age_stale)),
                    }
                }
        return comps

    # ---------- helpers ----------
    @staticmethod
    def _lvl_name(level):
        return {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}.get(level, 'UNKNOWN')

    def _debounce(self, comp: str, new_level: int):
        now = time.time()
        prev = self.last_levels.get(comp)
        if prev is None or new_level > prev[0]:
            self.last_levels[comp] = (new_level, now)
            return new_level
        prev_level, since = prev
        if new_level < prev_level and (now - since) < self.debounce_s:
            return prev_level
        self.last_levels[comp] = (new_level, now)
        return new_level

    # ---------- subscriptions ----------
    def _setup_component_monitors(self):
        for comp_name, comp in self.components.items():
            kind = comp.get('kind', 'ekf')
            if kind == 'ekf':
                t = comp.get('odom_topic', '')
                if t:
                    self.topic_mon[(comp_name, 'odom')] = TopicRateMonitor(self, t, Odometry, header_based=True, keep_last=True)
                # Optionally add inputs here if you extend your YAML (imu/odom/pose)

            elif kind == 'navsat':
                topics = comp.get('topics', {})
                gps = topics.get('gps_filtered', '')
                og  = topics.get('odom_gps', '')
                if gps:
                    self.topic_mon[(comp_name, 'gps_filtered')] = TopicRateMonitor(self, gps, NavSatFix, header_based=True, keep_last=False)
                if og:
                    self.topic_mon[(comp_name, 'odom_gps')] = TopicRateMonitor(self, og, Odometry, header_based=True, keep_last=False)

    # ---------- evaluations ----------
    def _eval_ekf(self, comp_name: str, comp: dict) -> DiagnosticStatus:
        st = DiagnosticStatus(name=comp_name, hardware_id=comp.get('hardware_id', 'localization'))
        level = DiagnosticStatus.OK
        msg = 'OK'

        thr = comp.get('thresholds', {})
        rate_warn = float(thr.get('rate_warn_hz', self.default_rate_warn))
        rate_err  = float(thr.get('rate_error_hz',  self.default_rate_err))
        age_stale = float(thr.get('age_stale_s',    self.default_age_stale))

        mon = self.topic_mon.get((comp_name, 'odom'))
        if mon is None or mon.last_msg is None:
            st.level = self._debounce(comp_name, DiagnosticStatus.ERROR)
            st.message = 'no odom output'
            return st

        age = mon.age_s()
        rate = mon.rate_hz()
        kv(st, 'odom_age_s',  f'{age:.3f}')
        kv(st, 'odom_rate_hz', f'{rate:.2f}')

        if age > age_stale:
            level, msg = DiagnosticStatus.STALE, 'odom stale'
        elif rate < rate_err:
            level, msg = DiagnosticStatus.ERROR, 'odom rate too low'
        elif rate < rate_warn:
            level, msg = DiagnosticStatus.WARN, 'odom rate low'

        # Snapshot pose/vel/yaw and simple quality checks (optional)
        od = mon.last_msg
        x = od.pose.pose.position.x
        y = od.pose.pose.position.y
        z = od.pose.pose.position.z
        yaw_deg = quat_to_yaw_deg(od.pose.pose.orientation)
        vx = od.twist.twist.linear.x
        vy = od.twist.twist.linear.y
        wz = od.twist.twist.angular.z
        kv(st, 'pos_x', f'{x:.3f}'); kv(st, 'pos_y', f'{y:.3f}'); kv(st, 'pos_z', f'{z:.3f}')
        kv(st, 'yaw_deg', f'{yaw_deg:.2f}')
        kv(st, 'lin_vel_x', f'{vx:.3f}'); kv(st, 'lin_vel_y', f'{vy:.3f}'); kv(st, 'ang_vel_z', f'{wz:.3f}')

        # Uncertainty from covariance (pose)
        pcov = od.pose.covariance
        try:
            std_x = math.sqrt(max(pcov[0], 0.0))
            std_y = math.sqrt(max(pcov[7], 0.0))
            std_xy = max(std_x, std_y)
            std_yaw_deg = math.degrees(math.sqrt(max(pcov[35], 0.0)))
        except Exception:
            std_xy = float('nan')
            std_yaw_deg = float('nan')
        kv(st, 'std_xy_m', f'{std_xy:.3f}')
        kv(st, 'std_yaw_deg', f'{std_yaw_deg:.2f}')

        if not math.isnan(std_xy):
            if std_xy > self.std_xy_err:
                level, msg = max(level, DiagnosticStatus.ERROR), 'position uncertainty high'
            elif std_xy > self.std_xy_warn:
                level, msg = max(level, DiagnosticStatus.WARN), 'position uncertainty rising'
        if not math.isnan(std_yaw_deg):
            if std_yaw_deg > self.std_yaw_err:
                level, msg = max(level, DiagnosticStatus.ERROR), 'yaw uncertainty high'
            elif std_yaw_deg > self.std_yaw_warn:
                level, msg = max(level, DiagnosticStatus.WARN), 'yaw uncertainty rising'

        # TF checks
        for pair in comp.get('tf_required', []) or []:
            try:
                tgt, src = [p.strip() for p in pair.split('->')]
            except Exception:
                continue
            tf_age, ok = self.tfmon.age(tgt, src)
            kv(st, f'tf_{tgt}_{src}_age_s', 'missing' if not ok else f'{tf_age:.3f}')
            if not ok or tf_age > self.tf_err_age:
                level, msg = max(level, DiagnosticStatus.ERROR), 'tf missing/too old'
            elif tf_age > self.tf_warn_age:
                level, msg = max(level, DiagnosticStatus.WARN), 'tf aging'

        # Jump detection
        prev = self.prev_pose.get(comp_name)
        if prev is not None:
            dx = math.hypot(x - prev[0], y - prev[1])
            dyaw = abs((yaw_deg - prev[2] + 180) % 360 - 180)
            if dx > self.max_xy_jump:
                kv(st, 'jump_xy_m', f'{dx:.2f}')
                level = max(level, DiagnosticStatus.WARN)
                msg = 'pose jump'
            if dyaw > self.max_yaw_jump:
                kv(st, 'jump_yaw_deg', f'{dyaw:.1f}')
                level = max(level, DiagnosticStatus.WARN)
                msg = 'yaw jump'
        self.prev_pose[comp_name] = (x, y, yaw_deg)

        st.level = self._debounce(comp_name, level)
        st.message = msg
        return st

    def _eval_navsat(self, comp_name: str, comp: dict) -> DiagnosticStatus:
        st = DiagnosticStatus(name=comp_name, hardware_id=comp.get('hardware_id', 'localization'))
        level = DiagnosticStatus.OK
        msg = 'OK'

        thr = comp.get('thresholds', {})
        rate_warn = float(thr.get('rate_warn_hz', self.default_rate_warn/2))
        rate_err  = float(thr.get('rate_error_hz', self.default_rate_err/2))
        age_stale = float(thr.get('age_stale_s', 1.0))

        # gps_filtered (NavSatFix)
        mon_gps = self.topic_mon.get((comp_name, 'gps_filtered'))
        if mon_gps:
            rate, age = mon_gps.rate_hz(), mon_gps.age_s()
            kv(st, 'gps_filtered_rate_hz', f'{rate:.2f}')
            kv(st, 'gps_filtered_age_s',  f'{age:.2f}')
            if math.isinf(age):
                level, msg = max(level, DiagnosticStatus.ERROR), 'gps_filtered missing'
            elif age > age_stale:
                level, msg = max(level, DiagnosticStatus.STALE), 'gps_filtered stale'
            elif rate < rate_err:
                level, msg = max(level, DiagnosticStatus.ERROR), 'gps_filtered rate too low'
            elif rate < rate_warn:
                level, msg = max(level, DiagnosticStatus.WARN), 'gps_filtered rate low'

        # odom_gps (Odometry)
        mon_odom = self.topic_mon.get((comp_name, 'odom_gps'))
        if mon_odom:
            rate, age = mon_odom.rate_hz(), mon_odom.age_s()
            kv(st, 'odom_gps_rate_hz', f'{rate:.2f}')
            kv(st, 'odom_gps_age_s',  f'{age:.2f}')
            if math.isinf(age):
                level, msg = max(level, DiagnosticStatus.ERROR), 'odom_gps missing'
            elif age > age_stale:
                level, msg = max(level, DiagnosticStatus.STALE), 'odom_gps stale'
            elif rate < rate_err:
                level, msg = max(level, DiagnosticStatus.ERROR), 'odom_gps rate too low'
            elif rate < rate_warn:
                level, msg = max(level, DiagnosticStatus.WARN), 'odom_gps rate low'

        st.level = self._debounce(comp_name, level)
        st.message = msg
        return st

    # ---------- tick ----------
    def _tick(self):
        da = DiagnosticArray()
        da.header.stamp = self.get_clock().now().to_msg()

        statuses = []
        for comp_name, comp in self.components.items():
            kind = comp.get('kind', 'ekf')
            if kind == 'navsat':
                statuses.append(self._eval_navsat(comp_name, comp))
            else:
                statuses.append(self._eval_ekf(comp_name, comp))

        roll = DiagnosticStatus(name='localization_overall', hardware_id='localization')
        if statuses:
            worst = max(s.level for s in statuses)
            roll.level = worst
            roll.message = self._lvl_name(worst)
        else:
            roll.level = DiagnosticStatus.WARN
            roll.message = 'no components configured'
        statuses.append(roll)

        da.status = statuses
        self.pub.publish(da)


def main(args=None):
    rclpy.init(args=args)
    node = EkfDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
