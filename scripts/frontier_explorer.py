#!/usr/bin/env python3

import math
from collections import deque
from typing import List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


GridCell = Tuple[int, int]


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "robot_footprint")
        self.declare_parameter("timer_period", 2.0)
        self.declare_parameter("min_frontier_size", 12)
        self.declare_parameter("goal_timeout_sec", 75.0)
        self.declare_parameter("blacklist_radius", 0.45)
        self.declare_parameter("distance_penalty_scale", 0.45)
        self.declare_parameter("min_known_free_cells", 5000)
        self.declare_parameter("completion_cycles", 10)
        self.declare_parameter("goal_safety_cells", 2)
        self.declare_parameter("auto_save_map", True)
        self.declare_parameter("save_map_url", "/home/sharath/indoor_bot_ws/auto_explore_map")
        self.declare_parameter("image_format", "png")
        self.declare_parameter("map_mode", "trinary")
        self.declare_parameter("free_thresh", 0.25)
        self.declare_parameter("occupied_thresh", 0.65)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.min_frontier_size = int(self.get_parameter("min_frontier_size").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.blacklist_radius = float(self.get_parameter("blacklist_radius").value)
        self.distance_penalty_scale = float(self.get_parameter("distance_penalty_scale").value)
        self.min_known_free_cells = int(self.get_parameter("min_known_free_cells").value)
        self.completion_cycles = int(self.get_parameter("completion_cycles").value)
        self.goal_safety_cells = int(self.get_parameter("goal_safety_cells").value)
        self.auto_save_map = bool(self.get_parameter("auto_save_map").value)
        self.save_map_url = str(self.get_parameter("save_map_url").value)
        self.image_format = str(self.get_parameter("image_format").value)
        self.map_mode = str(self.get_parameter("map_mode").value)
        self.free_thresh = float(self.get_parameter("free_thresh").value)
        self.occupied_thresh = float(self.get_parameter("occupied_thresh").value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.no_frontier_cycles = 0
        self.exploration_complete = False
        self.save_requested = False
        self.blacklist: List[Tuple[float, float]] = []

        self.goal_handle = None
        self.goal_result_future = None
        self.goal_start_time = None
        self.goal_position: Optional[Tuple[float, float]] = None
        self.goal_cancel_requested = False
        self._status_message: Optional[str] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.save_map_client = self.create_client(SaveMap, "/map_saver/save_map")

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, map_qos)
        self.frontier_marker_pub = self.create_publisher(Marker, "/frontier_markers", 10)
        self.goal_marker_pub = self.create_publisher(Marker, "/selected_frontier", 10)

        self.timer = self.create_timer(self.timer_period, self._on_timer)

        self.get_logger().info("Frontier exploration node started.")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_timer(self) -> None:
        if self.exploration_complete:
            return

        if self.map_msg is None:
            self._set_status("Waiting for /map updates.")
            return

        if not self.navigate_client.server_is_ready():
            self._set_status("Waiting for navigate_to_pose action server.")
            self.navigate_client.wait_for_server(timeout_sec=0.1)
            return

        self._check_goal_state()
        if self.goal_handle is not None:
            self._set_status(None)
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._set_status(
                f"Waiting for TF {self.global_frame} -> {self.robot_frame}."
            )
            return

        frontier_groups = self._find_frontiers()
        self._publish_frontier_markers(frontier_groups)

        if not frontier_groups:
            known_free = self._count_free_cells()
            if known_free < self.min_known_free_cells:
                self._set_status("Waiting for enough mapped free space before exploring.")
                self.get_logger().debug("Waiting for more of the map to become known before exploring.")
                return

            self.no_frontier_cycles += 1
            if self.no_frontier_cycles >= self.completion_cycles:
                self.get_logger().info("No more frontiers detected. Exploration appears complete.")
                self.exploration_complete = True
                self._publish_goal_marker(None)
                if self.auto_save_map:
                    self._request_map_save()
            return

        self.no_frontier_cycles = 0
        goal = self._select_frontier_goal(frontier_groups, robot_pose)
        if goal is None:
            self._set_status("Frontiers found, but no safe goal passed filtering.")
            self.get_logger().warn("Frontiers exist, but none passed goal safety / blacklist checks.")
            return

        self._set_status(None)
        self._publish_goal_marker(goal)
        self._send_goal(goal, robot_pose)

    def _set_status(self, message: Optional[str]) -> None:
        if message == self._status_message:
            return

        self._status_message = message
        if message is not None:
            self.get_logger().info(message)

    def _lookup_robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().debug(f"Waiting for TF {self.global_frame} -> {self.robot_frame}: {exc}")
            return None

        translation = transform.transform.translation
        return (translation.x, translation.y)

    def _check_goal_state(self) -> None:
        if self.goal_handle is None or self.goal_result_future is None:
            return

        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if elapsed > self.goal_timeout_sec and not self.goal_cancel_requested:
            self.get_logger().warn("Goal timed out, canceling and blacklisting it.")
            self.goal_cancel_requested = True
            self.goal_handle.cancel_goal_async()

        if not self.goal_result_future.done():
            return

        result = self.goal_result_future.result()
        status = result.status
        if status != GoalStatus.STATUS_SUCCEEDED and self.goal_position is not None:
            self.blacklist.append(self.goal_position)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Frontier goal reached.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Frontier goal aborted.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Frontier goal canceled.")
        else:
            self.get_logger().warn(f"Frontier goal finished with status {status}.")

        self.goal_handle = None
        self.goal_result_future = None
        self.goal_start_time = None
        self.goal_position = None
        self.goal_cancel_requested = False

    def _send_goal(self, goal_xy: Tuple[float, float], robot_xy: Tuple[float, float]) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_xy[0]
        goal_msg.pose.pose.position.y = goal_xy[1]
        goal_msg.pose.pose.position.z = 0.0

        yaw = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        self.goal_start_time = self.get_clock().now()
        self.goal_position = goal_xy
        self.get_logger().info(
            f"Sending frontier goal to x={goal_xy[0]:.2f}, y={goal_xy[1]:.2f}"
        )

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Frontier goal rejected by Nav2.")
            if self.goal_position is not None:
                self.blacklist.append(self.goal_position)
            self.goal_position = None
            self.goal_start_time = None
            return

        self.goal_handle = goal_handle
        self.goal_result_future = goal_handle.get_result_async()

    def _request_map_save(self) -> None:
        if self.save_requested:
            return
        if not self.save_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Map saver service is unavailable; skipping auto-save.")
            return

        request = SaveMap.Request()
        request.map_topic = self.map_topic
        request.map_url = self.save_map_url
        request.image_format = self.image_format
        request.map_mode = self.map_mode
        request.free_thresh = self.free_thresh
        request.occupied_thresh = self.occupied_thresh

        self.save_requested = True
        future = self.save_map_client.call_async(request)
        future.add_done_callback(self._save_map_callback)
        self.get_logger().info(f"Requesting map save to {self.save_map_url}")

    def _save_map_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Auto-save failed: {exc}")
            return

        if response.result:
            self.get_logger().info("Auto-saved exploration map successfully.")
        else:
            self.get_logger().error("Map saver service returned failure.")

    def _find_frontiers(self) -> List[List[GridCell]]:
        assert self.map_msg is not None
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        data = list(self.map_msg.data)

        frontier_cells = {
            (x, y)
            for y in range(height)
            for x in range(width)
            if self._is_frontier_cell(data, x, y, width, height)
        }

        visited = set()
        groups: List[List[GridCell]] = []
        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = []
            queue = deque([cell])
            visited.add(cell)
            while queue:
                current = queue.popleft()
                cluster.append(current)
                for neighbor in self._neighbors8(current[0], current[1], width, height):
                    if neighbor in frontier_cells and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

            if len(cluster) >= self.min_frontier_size:
                groups.append(cluster)

        return groups

    def _select_frontier_goal(
        self, frontier_groups: Sequence[Sequence[GridCell]], robot_xy: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        assert self.map_msg is not None
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        data = list(self.map_msg.data)

        best_goal = None
        best_score = -float("inf")
        for cluster in frontier_groups:
            goal = self._cluster_goal(cluster, data, width, height)
            if goal is None:
                continue

            if self._is_blacklisted(goal):
                continue

            distance = math.hypot(goal[0] - robot_xy[0], goal[1] - robot_xy[1])
            score = len(cluster) - (distance / self.map_msg.info.resolution) * self.distance_penalty_scale
            if score > best_score:
                best_score = score
                best_goal = goal

        return best_goal

    def _cluster_goal(
        self,
        cluster: Sequence[GridCell],
        data: Sequence[int],
        width: int,
        height: int,
    ) -> Optional[Tuple[float, float]]:
        centroid_x = sum(cell[0] for cell in cluster) / len(cluster)
        centroid_y = sum(cell[1] for cell in cluster) / len(cluster)
        candidates = sorted(
            cluster,
            key=lambda cell: (cell[0] - centroid_x) ** 2 + (cell[1] - centroid_y) ** 2,
        )

        for cell in candidates:
            if self._is_safe_goal_cell(data, cell[0], cell[1], width, height):
                return self._cell_to_world(cell[0], cell[1])
        return None

    def _publish_frontier_markers(self, frontier_groups: Sequence[Sequence[GridCell]]) -> None:
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 1.0

        for cluster in frontier_groups:
            for cell_x, cell_y in cluster:
                x, y = self._cell_to_world(cell_x, cell_y)
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                marker.points.append(point)

        self.frontier_marker_pub.publish(marker)

    def _publish_goal_marker(self, goal: Optional[Tuple[float, float]]) -> None:
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "selected_frontier"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETE if goal is None else Marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        if goal is not None:
            marker.pose.position.x = goal[0]
            marker.pose.position.y = goal[1]
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0

        self.goal_marker_pub.publish(marker)

    def _is_blacklisted(self, goal: Tuple[float, float]) -> bool:
        return any(
            math.hypot(goal[0] - blacklisted[0], goal[1] - blacklisted[1]) < self.blacklist_radius
            for blacklisted in self.blacklist
        )

    def _count_free_cells(self) -> int:
        assert self.map_msg is not None
        return sum(1 for value in self.map_msg.data if value == 0)

    def _cell_to_world(self, cell_x: int, cell_y: int) -> Tuple[float, float]:
        assert self.map_msg is not None
        origin = self.map_msg.info.origin.position
        resolution = self.map_msg.info.resolution
        x = origin.x + (cell_x + 0.5) * resolution
        y = origin.y + (cell_y + 0.5) * resolution
        return (x, y)

    def _is_safe_goal_cell(
        self, data: Sequence[int], x: int, y: int, width: int, height: int
    ) -> bool:
        if data[y * width + x] != 0:
            return False

        for offset_y in range(-self.goal_safety_cells, self.goal_safety_cells + 1):
            for offset_x in range(-self.goal_safety_cells, self.goal_safety_cells + 1):
                neighbor_x = x + offset_x
                neighbor_y = y + offset_y
                if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                    continue
                occupancy = data[neighbor_y * width + neighbor_x]
                if occupancy > 50:
                    return False

        return True

    @staticmethod
    def _is_frontier_cell(
        data: Sequence[int], x: int, y: int, width: int, height: int
    ) -> bool:
        if data[y * width + x] != 0:
            return False

        # Cartographer crops the occupancy grid to the known submap bounds, so
        # free cells on the outer edge still represent unexplored space beyond
        # the current map extents.
        if x == 0 or y == 0 or x == width - 1 or y == height - 1:
            return True

        for neighbor_x, neighbor_y in FrontierExplorer._neighbors4(x, y, width, height):
            if data[neighbor_y * width + neighbor_x] == -1:
                return True
        return False

    @staticmethod
    def _neighbors4(x: int, y: int, width: int, height: int) -> List[GridCell]:
        neighbors = []
        for offset_x, offset_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor_x = x + offset_x
            neighbor_y = y + offset_y
            if 0 <= neighbor_x < width and 0 <= neighbor_y < height:
                neighbors.append((neighbor_x, neighbor_y))
        return neighbors

    @staticmethod
    def _neighbors8(x: int, y: int, width: int, height: int) -> List[GridCell]:
        neighbors = []
        for offset_y in (-1, 0, 1):
            for offset_x in (-1, 0, 1):
                if offset_x == 0 and offset_y == 0:
                    continue
                neighbor_x = x + offset_x
                neighbor_y = y + offset_y
                if 0 <= neighbor_x < width and 0 <= neighbor_y < height:
                    neighbors.append((neighbor_x, neighbor_y))
        return neighbors


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
