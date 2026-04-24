#!/usr/bin/env python3

from pathlib import Path
import sys
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class MapSaver(Node):
    def __init__(self, output_prefix: Path, timeout_sec: float) -> None:
        super().__init__("indoor_bot_map_saver")
        self.output_prefix = output_prefix
        self.timeout_sec = timeout_sec
        self.map_msg: Optional[OccupancyGrid] = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, qos)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def wait_and_save(self) -> int:
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.map_msg is not None:
                self._write_files(self.map_msg)
                return 0

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.timeout_sec:
                self.get_logger().error("Timed out waiting for /map.")
                return 1

        return 1

    def _write_files(self, msg: OccupancyGrid) -> None:
        pgm_path = self.output_prefix.with_suffix(".pgm")
        yaml_path = self.output_prefix.with_suffix(".yaml")
        pgm_path.parent.mkdir(parents=True, exist_ok=True)

        width = msg.info.width
        height = msg.info.height

        with pgm_path.open("wb") as pgm_file:
            pgm_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            for row in range(height - 1, -1, -1):
                for col in range(width):
                    occupancy = msg.data[row * width + col]
                    if occupancy < 0:
                        value = 205
                    elif occupancy >= 65:
                        value = 0
                    else:
                        value = 254
                    pgm_file.write(bytes((value,)))

        origin = msg.info.origin
        yaml_text = "\n".join(
            [
                f"image: {pgm_path.name}",
                f"resolution: {msg.info.resolution}",
                f"origin: [{origin.position.x}, {origin.position.y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        )
        yaml_path.write_text(yaml_text, encoding="ascii")
        self.get_logger().info(f"Saved map to {pgm_path} and {yaml_path}")


def main() -> int:
    output_prefix = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path("~/indoor_bot_map").expanduser()
    timeout_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    rclpy.init()
    node = MapSaver(output_prefix=output_prefix, timeout_sec=timeout_sec)
    try:
        return node.wait_and_save()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
