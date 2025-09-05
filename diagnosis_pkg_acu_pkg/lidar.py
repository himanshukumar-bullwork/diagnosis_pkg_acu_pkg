#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

LIDAR_FREQ_HZ = 10.0
STALE_FACTOR = 1.5
DEVICE_PATH = '/dev/lidar'

class LidarDiagnostics(Node):
    def __init__(self):
        super().__init__('lidar_diagnostics')
        self.last_scan_stamp = None

        # QoS: RELIABLE / VOLATILE / KEEP_LAST depth 50
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', qos)

        # Make publish rate configurable; default 2 Hz
        self.declare_parameter('publish_hz', 2.0)
        hz = float(self.get_parameter('publish_hz').value) or 2.0
        self.create_timer(1.0 / hz, self.publish_diagnostics)

    def scan_cb(self, msg: LaserScan):
        self.last_scan_stamp = self.get_clock().now()

    def publish_diagnostics(self):
        now = self.get_clock().now()
        da = DiagnosticArray()
        da.header.stamp = now.to_msg()
        st = DiagnosticStatus()
        st.name = "lidar"
        st.hardware_id = "LIDAR"
        st.level = DiagnosticStatus.OK
        st.message = "OK"

        if not os.path.exists(DEVICE_PATH):
            st.level = DiagnosticStatus.ERROR
            st.message = "device not found"
            st.values.append(KeyValue(key="error", value=f"{DEVICE_PATH} missing"))
        elif self.last_scan_stamp is None:
            st.level = DiagnosticStatus.WARN
            st.message = "no scan received yet"
            st.values.append(KeyValue(key="error", value="no data"))
        else:
            age = (now - self.last_scan_stamp).nanoseconds * 1e-9
            period = 1.0 / LIDAR_FREQ_HZ
            if age > period * STALE_FACTOR:
                st.level = DiagnosticStatus.STALE
                st.message = f"stale ({age:.2f}s since last scan)"
                st.values.append(KeyValue(key="stale_duration", value=f"{age:.2f}s"))
            else:
                st.values.append(KeyValue(key="age", value=f"{age:.2f}s"))

        da.status.append(st)
        print(da)
        self.diag_pub.publish(da)
        


def main(args=None):
    rclpy.init(args=args)
    node = LidarDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
