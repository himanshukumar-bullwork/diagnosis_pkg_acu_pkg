#!/usr/bin/env python3
import math, time
from collections import deque
from typing import Deque, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from sensor_msgs.msg import PointCloud2, Imu

def kv(st: DiagnosticStatus, k: str, v: Any):
    st.values.append(KeyValue(key=str(k), value=str(v)))

def sensor_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )

def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )

def secs_between(node: Node, stamp) -> float:
    """
    Age = node_clock_now - stamp in seconds.
    Returns +inf if stamp missing or zero. Avoids Time-subtraction clock mismatches.
    """
    try:
        s  = int(getattr(stamp, "sec", 0))
        ns = int(getattr(stamp, "nanosec", 0))
        if s == 0 and ns == 0:
            return float("inf")
        now_ns = node.get_clock().now().nanoseconds
        msg_ns = s * 1_000_000_000 + ns
        return max(0.0, (now_ns - msg_ns) / 1e9)
    except Exception:
        return float("inf")

class RateWindow:
    def __init__(self, horizon_s: float = 5.0):
        self.hz_window: Deque[float] = deque(maxlen=512)
        self.horizon_s = horizon_s

    def tick(self, tnow: float):
        self.hz_window.append(tnow)
        while self.hz_window and (tnow - self.hz_window[0]) > self.horizon_s:
            self.hz_window.popleft()

    def rate(self) -> float:
        if len(self.hz_window) < 2:
            return 0.0
        dt = self.hz_window[-1] - self.hz_window[0]
        return (len(self.hz_window)-1)/dt if dt > 0 else 0.0

# Safely convert sequences / numpy arrays to a Python list
def to_list(x) -> list:
    if x is None:
        return []
    try:
        import numpy as np  # type: ignore
        if isinstance(x, np.ndarray):
            return x.tolist()
    except Exception:
        pass
    try:
        return list(x)
    except Exception:
        return []

class ZedMinDiag(Node):
    def __init__(self):
        super().__init__("zed_min_diag")

        self.declare_parameter("config_file", "")
        cfg_path = self.get_parameter("config_file").get_parameter_value().string_value
        if not cfg_path:
            self.get_logger().fatal("Set -p config_file:=/abs/path/to/zed_min_diag.yaml")
            raise SystemExit(1)

        import yaml
        with open(cfg_path, "r") as f:
            root = yaml.safe_load(f) or {}
        cfg = root.get("zed_min_diag", {}) or {}

        self.hw_id = cfg.get("hardware_id", "zed")

        # ---- pointcloud ----
        pcfg = cfg.get("pointcloud", {}) or {}
        self.pc_topic = pcfg.get("topic", "/pointcloud/point_registered")
        self.pc_warn = float(pcfg.get("warn_rate_hz", 10.0))
        self.pc_err = float(pcfg.get("error_rate_hz", 3.0))
        self.pc_stale = float(pcfg.get("stale_age_s", 0.5))
        self.pc_header_warn = float(pcfg.get("header_age_warn_s", self.pc_stale))
        self.pc_header_err  = float(pcfg.get("header_age_error_s", max(self.pc_stale*2, 1.0)))
        self.pc_min_points_warn = int(pcfg.get("min_points_warn", 1000))
        self.pc_min_points_err  = int(pcfg.get("min_points_error", 1))
        self.pc_qos = pcfg.get("qos", "sensor").lower()
        self.pc_frame = pcfg.get("required_frame", "")

        self.pc_last_wall = 0.0
        self.pc_last_msg: Optional[PointCloud2] = None
        self.pc_rate = RateWindow(5.0)

        qos_pc = sensor_qos() if self.pc_qos in ("sensor","best_effort","be") else reliable_qos()
        self.create_subscription(PointCloud2, self.pc_topic, self._pc_cb, qos_pc)

        # ---- imu ----
        icfg = cfg.get("imu", {}) or {}
        self.imu_topic = icfg.get("topic", "/zed/zed_node/imu/data")
        self.imu_warn = float(icfg.get("warn_rate_hz", 150.0))
        self.imu_err = float(icfg.get("error_rate_hz", 50.0))
        self.imu_stale = float(icfg.get("stale_age_s", 0.15))
        self.imu_header_warn = float(icfg.get("header_age_warn_s", self.imu_stale))
        self.imu_header_err  = float(icfg.get("header_age_error_s", max(self.imu_stale*2, 0.3)))
        self.imu_qos = icfg.get("qos", "sensor").lower()
        self.quat_tol = float(icfg.get("quat_norm_tol", 0.02))
        self.min_cov_warn = float(icfg.get("min_cov_warn", 1e-6))

        self.imu_last_wall = 0.0
        self.imu_last_msg: Optional[Imu] = None
        self.imu_rate = RateWindow(5.0)

        qos_imu = sensor_qos() if self.imu_qos in ("sensor","best_effort","be") else reliable_qos()
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, qos_imu)

        # publisher 2 Hz
        pub_qos = reliable_qos(10)
        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", pub_qos)
        self.create_timer(0.5, self._publish)

    # ---------- callbacks ----------
    def _pc_cb(self, msg: PointCloud2):
        self.pc_last_msg = msg
        self.pc_last_wall = time.time()
        self.pc_rate.tick(self.pc_last_wall)

    def _imu_cb(self, msg: Imu):
        self.imu_last_msg = msg
        self.imu_last_wall = time.time()
        self.imu_rate.tick(self.imu_last_wall)

    # ---------- builders ----------
    def _status_pointcloud(self) -> DiagnosticStatus:
        st = DiagnosticStatus()
        st.name = "ZED: PointCloud"
        st.hardware_id = self.hw_id

        now = time.time()
        age = now - self.pc_last_wall if self.pc_last_wall > 0 else float("inf")
        kv(st, "age_s", f"{age:.3f}")
        rate = self.pc_rate.rate()
        kv(st, "rate_hz", f"{rate:.2f}")
        kv(st, "topic", self.pc_topic)

        if age > self.pc_stale:
            st.level = DiagnosticStatus.STALE; st.message = f"stale ({age:.2f}s)"
            return st

        msg = self.pc_last_msg
        if msg is None:
            st.level = DiagnosticStatus.STALE; st.message = "no data"
            return st

        h_age = secs_between(self, msg.header.stamp)
        kv(st, "header_age_s", "inf" if math.isinf(h_age) else f"{h_age:.3f}")
        if math.isinf(h_age):
            st.level = max(st.level, DiagnosticStatus.WARN)
            if not st.message: st.message = "header age unknown"
        elif h_age >= self.pc_header_err:
            st.level = DiagnosticStatus.ERROR; st.message = "old header timestamp"
        elif h_age >= self.pc_header_warn:
            st.level = max(st.level, DiagnosticStatus.WARN); st.message = "header timestamp lag"

        if self.pc_frame and msg.header.frame_id != self.pc_frame:
            st.level = max(st.level, DiagnosticStatus.WARN)
            kv(st, "frame_mismatch", f"{msg.header.frame_id} != {self.pc_frame}")

        pts = int(msg.width) * int(msg.height)
        kv(st, "points", pts)
        if pts <= self.pc_min_points_err:
            st.level = DiagnosticStatus.ERROR; st.message = "no / too few points"
        elif pts <= self.pc_min_points_warn:
            st.level = max(st.level, DiagnosticStatus.WARN)
            if not st.message: st.message = "few points"

        if rate <= self.pc_err:
            st.level = DiagnosticStatus.ERROR; st.message = "low rate"
        elif rate <= self.pc_warn:
            st.level = max(st.level, DiagnosticStatus.WARN)
            if not st.message: st.message = "rate warning"

        if st.level == DiagnosticStatus.OK:
            st.message = "OK"
        return st

    def _status_imu(self) -> DiagnosticStatus:
        st = DiagnosticStatus()
        st.name = "ZED: IMU"
        st.hardware_id = self.hw_id

        now = time.time()
        age = now - self.imu_last_wall if self.imu_last_wall > 0 else float("inf")
        kv(st, "age_s", f"{age:.3f}")
        rate = self.imu_rate.rate()
        kv(st, "rate_hz", f"{rate:.1f}")
        kv(st, "topic", self.imu_topic)

        if age > self.imu_stale:
            st.level = DiagnosticStatus.STALE; st.message = f"stale ({age:.2f}s)"
            return st

        msg = self.imu_last_msg
        if msg is None:
            st.level = DiagnosticStatus.STALE; st.message = "no data"
            return st

        h_age = secs_between(self, msg.header.stamp)
        kv(st, "header_age_s", "inf" if math.isinf(h_age) else f"{h_age:.3f}")
        if math.isinf(h_age):
            st.level = max(st.level, DiagnosticStatus.WARN)
            if not st.message: st.message = "header age unknown"
        elif h_age >= self.imu_header_err:
            st.level = DiagnosticStatus.ERROR; st.message = "old header timestamp"
        elif h_age >= self.imu_header_warn:
            st.level = max(st.level, DiagnosticStatus.WARN); st.message = "header timestamp lag"

        # Quaternion sanity
        q = msg.orientation
        qn = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if qn > 0.0:
            kv(st, "quat_norm", f"{qn:.5f}")
            if abs(1.0 - qn) > self.quat_tol:
                st.level = max(st.level, DiagnosticStatus.WARN)
                kv(st, "quat_warn", f"|1-norm|>{self.quat_tol}")

        # Covariance check — convert to list; don't boolean-test arrays
        cov = to_list(getattr(msg, "orientation_covariance", None))
        if len(cov) >= 9:
            diag = [cov[0], cov[4], cov[8]]
            # many drivers use -1 diag to mean “unknown”; skip warn in that case
            if not any(abs(d + 1.0) < 1e-9 for d in diag):
                if all(abs(c) < self.min_cov_warn for c in cov):
                    st.level = max(st.level, DiagnosticStatus.WARN)
                    kv(st, "cov_warn", "orientation_covariance ~0")

        if rate <= self.imu_err:
            st.level = DiagnosticStatus.ERROR; st.message = "low rate"
        elif rate <= self.imu_warn:
            st.level = max(st.level, DiagnosticStatus.WARN)
            if not st.message: st.message = "rate warning"

        if st.level == DiagnosticStatus.OK:
            st.message = "OK"
        return st

    def _publish(self):
        da = DiagnosticArray()
        da.header.stamp = self.get_clock().now().to_msg()
        da.status.append(self._status_pointcloud())
        da.status.append(self._status_imu())
        self.pub.publish(da)

def main(args=None):
    rclpy.init(args=args)
    node = ZedMinDiag()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
