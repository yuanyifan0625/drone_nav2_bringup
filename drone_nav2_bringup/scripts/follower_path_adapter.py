#!/usr/bin/env python3
"""Turn a moving Formation Target Pose into bounded-rate FollowPath goals."""

import math

from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def target_moved_enough(
    previous: PoseStamped | None, target: PoseStamped, threshold: float
) -> bool:
    """Return whether a new short path is needed for the moving slot."""

    return previous is None or math.hypot(
        target.pose.position.x - previous.pose.position.x,
        target.pose.position.y - previous.pose.position.y,
    ) >= threshold


def should_replace_path(
    previous: PoseStamped | None, target: PoseStamped, threshold: float,
    last_sent_seconds: float | None, now_seconds: float, minimum_interval: float,
) -> bool:
    """Rate-limit FollowPath replacement while a leader moves its slot."""

    return (
        target_moved_enough(previous, target, threshold)
        and (last_sent_seconds is None or now_seconds - last_sent_seconds >= minimum_interval)
    )


def short_straight_path(start: PoseStamped, target: PoseStamped) -> list[PoseStamped]:
    """Return a map-frame path sampled densely enough for MPPI path critics."""

    distance = math.hypot(
        target.pose.position.x - start.pose.position.x,
        target.pose.position.y - start.pose.position.y,
    )
    steps = max(1, math.ceil(distance / 0.2))
    poses = []
    for index in range(steps + 1):
        ratio = index / steps
        pose = PoseStamped()
        pose.header = target.header
        pose.pose.position.x = start.pose.position.x + ratio * (
            target.pose.position.x - start.pose.position.x
        )
        pose.pose.position.y = start.pose.position.y + ratio * (
            target.pose.position.y - start.pose.position.y
        )
        pose.pose.position.z = start.pose.position.z + ratio * (
            target.pose.position.z - start.pose.position.z
        )
        pose.pose.orientation = target.pose.orientation
        poses.append(pose)
    return poses


class FollowerPathAdapter(Node):
    def __init__(self) -> None:
        super().__init__("follower_path_adapter")
        self._target = None
        self._odom = None
        self._last_sent = None
        self._goal_handle = None
        self._last_sent_seconds = None
        self._target_threshold = float(self.declare_parameter("target_update_threshold", 0.35).value)
        self._minimum_replacement_interval = float(
            self.declare_parameter("minimum_replacement_interval", 1.0).value
        )
        self.create_subscription(PoseStamped, "formation_target_pose", self._on_target, 10)
        self.create_subscription(Odometry, "map_odom", self._on_odom, 10)
        self._client = ActionClient(self, FollowPath, "follow_path")
        self.create_timer(0.5, self._send_if_needed)

    def _on_target(self, message: PoseStamped) -> None:
        self._target = message

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message

    def _send_if_needed(self) -> None:
        if self._target is None or self._odom is None or not self._client.server_is_ready():
            return
        if not should_replace_path(
            self._last_sent, self._target, self._target_threshold,
            self._last_sent_seconds, self.get_clock().now().nanoseconds / 1e9,
            self._minimum_replacement_interval,
        ):
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        start = PoseStamped()
        start.header = self._target.header
        start.pose = self._odom.pose.pose
        path = Path(header=self._target.header, poses=short_straight_path(start, self._target))
        goal = FollowPath.Goal(path=path, controller_id="FollowPath")
        self._client.send_goal_async(goal).add_done_callback(self._on_goal)
        self._last_sent = self._target
        self._last_sent_seconds = self.get_clock().now().nanoseconds / 1e9

    def _on_goal(self, future) -> None:
        self._goal_handle = future.result() if future.result().accepted else None


def main() -> None:
    rclpy.init()
    node = FollowerPathAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
