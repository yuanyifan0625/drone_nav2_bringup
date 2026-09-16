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


class FollowerPathAdapter(Node):
    def __init__(self) -> None:
        super().__init__("follower_path_adapter")
        self._target = None
        self._odom = None
        self._last_sent = None
        self._goal_handle = None
        self._target_threshold = float(self.declare_parameter("target_update_threshold", 0.15).value)
        self.create_subscription(PoseStamped, "formation_target_pose", self._on_target, 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self._client = ActionClient(self, FollowPath, "follow_path")
        self.create_timer(0.5, self._send_if_needed)

    def _on_target(self, message: PoseStamped) -> None:
        self._target = message

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message

    def _send_if_needed(self) -> None:
        if self._target is None or self._odom is None or not self._client.server_is_ready():
            return
        if not target_moved_enough(
            self._last_sent, self._target, self._target_threshold
        ):
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        start = PoseStamped()
        start.header = self._target.header
        start.pose = self._odom.pose.pose
        path = Path(header=self._target.header, poses=[start, self._target])
        goal = FollowPath.Goal(path=path, controller_id="FollowPath")
        self._client.send_goal_async(goal).add_done_callback(self._on_goal)
        self._last_sent = self._target

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
