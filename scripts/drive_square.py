#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveSquare(Node):
    def __init__(self) -> None:
        super().__init__("drive_square")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.completed = False
        self.failed = False

        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.45)
        self.declare_parameter("side_length", 2.0)
        self.declare_parameter("loops", 2)
        self.declare_parameter("startup_timeout_sec", 20.0)
        self.declare_parameter("require_clock", False)

        linear_speed = float(self.get_parameter("linear_speed").value)
        angular_speed = float(self.get_parameter("angular_speed").value)
        side_length = float(self.get_parameter("side_length").value)
        loops = int(self.get_parameter("loops").value)
        startup_timeout_sec = float(self.get_parameter("startup_timeout_sec").value)
        self.require_clock = bool(self.get_parameter("require_clock").value)

        turn_angle = math.pi / 2.0
        forward_duration = side_length / linear_speed
        turn_duration = turn_angle / angular_speed

        self.sequence = []
        for _ in range(loops):
            for _ in range(4):
                self.sequence.append(("forward", forward_duration))
                self.sequence.append(("turn", turn_duration))

        self.sequence_index = 0
        self.phase_start = None
        self.ready = False
        self.startup_deadline = time.monotonic() + startup_timeout_sec
        self.last_status_log = 0.0
        self.timer = self.create_timer(0.1, self.step)

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self.get_logger().info(
            "Driving a square with side_length=%.2f m, loops=%d"
            % (side_length, loops)
        )

    def publish_stop(self) -> None:
        self.publisher.publish(Twist())

    def step(self) -> None:
        if not self.ready:
            self._wait_for_simulation()
            return

        if self.sequence_index >= len(self.sequence):
            self.publish_stop()
            self.get_logger().info("Drive sequence complete.")
            self.timer.cancel()
            self.completed = True
            return

        phase_name, duration = self.sequence[self.sequence_index]
        elapsed = time.monotonic() - self.phase_start
        if elapsed >= duration:
            self.sequence_index += 1
            self.phase_start = time.monotonic()
            self.publish_stop()
            return

        cmd = Twist()
        if phase_name == "forward":
            cmd.linear.x = self.linear_speed
        else:
            cmd.angular.z = self.angular_speed
        self.publisher.publish(cmd)

    def _wait_for_simulation(self) -> None:
        now = time.monotonic()
        if self.publisher.get_subscription_count() == 0:
            self._log_wait_status("Waiting for Gazebo to subscribe to /cmd_vel.")
        elif self.require_clock:
            self._log_wait_status("Waiting for /clock. Gazebo may not be running or physics may be paused.")
        else:
            self.ready = True
            self.phase_start = now
            self.get_logger().info("Simulation ready. Starting motion sequence.")
            return

        if now >= self.startup_deadline:
            self.publish_stop()
            self.get_logger().error(
                "Simulation did not become ready before timeout. Start Gazebo first and ensure /cmd_vel has a subscriber."
            )
            self.timer.cancel()
            self.completed = True
            self.failed = True

    def _log_wait_status(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_status_log >= 2.0:
            self.get_logger().info(message)
            self.last_status_log = now


def main() -> None:
    rclpy.init()
    node = DriveSquare()
    try:
        while rclpy.ok() and not node.completed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if node.failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
