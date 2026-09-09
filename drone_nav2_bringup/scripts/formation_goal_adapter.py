#!/usr/bin/env python3
"""Normalize formation RViz poses and GeoJSON node IDs into Mission Goals."""

from __future__ import annotations

import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32


class TopologyGoalResolver:
    """Resolve only map-frame GeoJSON Point features by their numeric ID."""

    def __init__(self, graph_file: str, map_frame: str) -> None:
        try:
            document = json.loads(Path(graph_file).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"Cannot read topology graph '{graph_file}': {error}") from error
        if not isinstance(document, dict):
            raise ValueError(f"Topology graph '{graph_file}' is not a GeoJSON object")
        self._map_frame = map_frame
        self._nodes = {}
        for feature in document.get("features", []):
            if not isinstance(feature, dict):
                continue
            properties = feature.get("properties")
            geometry = feature.get("geometry")
            if (
                not isinstance(properties, dict)
                or not isinstance(geometry, dict)
                or geometry.get("type") != "Point"
                or not isinstance(properties.get("id"), int)
            ):
                continue
            self._nodes[properties["id"]] = feature

    def resolve(self, node_id: int) -> tuple[float, float]:
        feature = self._nodes.get(node_id)
        if feature is None:
            raise ValueError(f"Topology node ID {node_id} is not a Point feature")
        properties = feature["properties"]
        if properties.get("frame") != self._map_frame:
            raise ValueError(
                f"Topology node ID {node_id} is not in the {self._map_frame} frame"
            )
        coordinates = feature["geometry"].get("coordinates", [])
        if not isinstance(coordinates, list) or len(coordinates) < 2:
            raise ValueError(f"Topology node ID {node_id} has invalid Point coordinates")
        try:
            return float(coordinates[0]), float(coordinates[1])
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"Topology node ID {node_id} has non-numeric Point coordinates"
            ) from error


class FormationGoalAdapter(Node):
    """Own the external formation-goal normalization boundary for one vehicle."""

    def __init__(self) -> None:
        super().__init__("formation_goal_adapter")
        namespace = self.declare_parameter("vehicle_namespace", "MAV1").value
        self._map_frame = self.declare_parameter("map_frame", "map").value
        graph_file = self.declare_parameter("graph_file", "").value
        if not graph_file:
            raise ValueError("The graph_file parameter is required")
        self._resolver = TopologyGoalResolver(graph_file, self._map_frame)
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._mission_goal_publisher = self.create_publisher(
            PoseStamped, f"/{namespace}/mission_goal", reliable_qos
        )
        self.create_subscription(
            PoseStamped,
            f"/{namespace}/formation_goal_pose",
            self._on_formation_goal_pose,
            reliable_qos,
        )
        self.create_subscription(
            Int32,
            f"/{namespace}/formation_goal_node_id",
            self._on_formation_goal_node_id,
            reliable_qos,
        )

    def _on_formation_goal_pose(self, goal: PoseStamped) -> None:
        if goal.header.frame_id != self._map_frame:
            self.get_logger().warn(
                f"Rejecting Formation Goal outside the {self._map_frame} frame"
            )
            return
        self._mission_goal_publisher.publish(goal)

    def _on_formation_goal_node_id(self, node_id: Int32) -> None:
        try:
            x, y = self._resolver.resolve(node_id.data)
        except ValueError as error:
            self.get_logger().warn(f"Rejecting Formation Goal: {error}")
            return
        goal = PoseStamped()
        goal.header.frame_id = self._map_frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self._mission_goal_publisher.publish(goal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationGoalAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
