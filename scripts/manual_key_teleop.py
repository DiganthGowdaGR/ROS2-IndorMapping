#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Manual teleop (continuous publish)
---------------------------------
i : forward
, : backward
j : turn left
l : turn right
k : stop
q : quit

Use this node only in Manual Mode.
Do not run together with explore_lite/Nav2 to avoid /cmd_vel conflicts.
"""


def read_key(timeout_sec: float = 0.05) -> str:
    rlist, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if rlist:
        return sys.stdin.read(1)
    return ""


class ManualKeyTeleop(Node):
    def __init__(self) -> None:
        super().__init__("manual_key_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.speed = 0.30
        self.turn = 1.20
        self.linear = 0.0
        self.angular = 0.0
        self.quit = False

        self.create_timer(0.05, self._publish_cmd)  # 20 Hz
        self.create_timer(3.0, self._warn_if_other_cmd_vel_sources)

    def _publish_cmd(self) -> None:
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

    def _warn_if_other_cmd_vel_sources(self) -> None:
        # Non-invasive runtime warning to reduce accidental command-source conflicts.
        node_names = set(self.get_node_names())
        conflicts = []

        if "explore_lite" in node_names:
            conflicts.append("explore_lite")

        nav2_nodes = ["controller_server", "planner_server", "bt_navigator", "velocity_smoother"]
        conflicts.extend([name for name in nav2_nodes if name in node_names])

        if conflicts:
            self.get_logger().warn(
                "Potential /cmd_vel conflict: manual teleop is running with "
                + ", ".join(conflicts)
                + ". Stop other motion-control nodes."
            )

    def handle_key(self, key: str) -> None:
        if key == "i":
            self.linear = self.speed
            self.angular = 0.0
        elif key == ",":
            self.linear = -self.speed
            self.angular = 0.0
        elif key == "j":
            self.linear = 0.0
            self.angular = self.turn
        elif key == "l":
            self.linear = 0.0
            self.angular = -self.turn
        elif key == "k":
            self.linear = 0.0
            self.angular = 0.0
        elif key == "q":
            self.quit = True

    def stop(self) -> None:
        self.linear = 0.0
        self.angular = 0.0
        self._publish_cmd()


def main() -> None:
    print(HELP)
    rclpy.init()
    node = ManualKeyTeleop()
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok() and not node.quit:
            key = read_key()
            if key:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
