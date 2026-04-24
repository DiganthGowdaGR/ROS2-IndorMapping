#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanMotionGate(Node):
    def __init__(self) -> None:
        super().__init__("scan_motion_gate")

        self.declare_parameter("input_scan_topic", "/scan")
        self.declare_parameter("output_scan_topic", "/scan_for_slam")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("use_cmd_vel_fallback", True)
        self.declare_parameter("require_motion_for_scan", True)
        self.declare_parameter("linear_threshold", 0.02)
        self.declare_parameter("angular_threshold", 0.05)
        self.declare_parameter("hold_open_sec", 0.40)

        self.input_scan_topic = str(self.get_parameter("input_scan_topic").value)
        self.output_scan_topic = str(self.get_parameter("output_scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.use_cmd_vel_fallback = bool(self.get_parameter("use_cmd_vel_fallback").value)
        self.require_motion_for_scan = bool(self.get_parameter("require_motion_for_scan").value)
        self.linear_threshold = float(self.get_parameter("linear_threshold").value)
        self.angular_threshold = float(self.get_parameter("angular_threshold").value)
        self.hold_open_duration = Duration(seconds=float(self.get_parameter("hold_open_sec").value))

        self.last_motion_time: Optional[rclpy.time.Time] = None
        self.gate_open = False

        self.scan_pub = self.create_publisher(LaserScan, self.output_scan_topic, 10)
        self.create_subscription(LaserScan, self.input_scan_topic, self._scan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)

        if self.use_cmd_vel_fallback:
            self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)

        if self.require_motion_for_scan:
            self.get_logger().info(
                "Scan gate enabled: /scan_for_slam will publish only after motion is detected."
            )
        else:
            self.get_logger().info("Scan gate pass-through: forwarding scans immediately.")

    def _is_motion(self, linear_x: float, angular_z: float) -> bool:
        return abs(linear_x) > self.linear_threshold or abs(angular_z) > self.angular_threshold

    def _record_motion(self) -> None:
        self.last_motion_time = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        if self._is_motion(twist.linear.x, twist.angular.z):
            self._record_motion()

    def _cmd_vel_callback(self, msg: Twist) -> None:
        if self._is_motion(msg.linear.x, msg.angular.z):
            self._record_motion()

    def _scan_callback(self, msg: LaserScan) -> None:
        if not self.require_motion_for_scan:
            self.scan_pub.publish(msg)
            return

        now = self.get_clock().now()
        open_now = False

        if self.last_motion_time is not None and (now - self.last_motion_time) <= self.hold_open_duration:
            open_now = True

        if open_now != self.gate_open:
            self.gate_open = open_now
            if self.gate_open:
                self.get_logger().info("Motion detected: opening scan gate for Cartographer.")
            else:
                self.get_logger().info("No motion: closing scan gate for Cartographer.")

        if self.gate_open:
            self.scan_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanMotionGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
