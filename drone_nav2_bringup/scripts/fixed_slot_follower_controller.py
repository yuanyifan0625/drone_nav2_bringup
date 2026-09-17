#!/usr/bin/env python3
"""Track one yaw-relative Fixed V-Slot and publish its target pose."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def body_offset_to_map_target(
    *,
    leader_x: float,
    leader_y: float,
    leader_yaw: float,
    slot_forward: float,
    slot_left: float,
) -> tuple[float, float]:
    """Rotate a Leader FLU slot into its shared ENU map target."""

    return (
        leader_x + slot_forward * math.cos(leader_yaw) - slot_left * math.sin(leader_yaw),
        leader_y + slot_forward * math.sin(leader_yaw) + slot_left * math.cos(leader_yaw),
    )


def map_frame_formation_target_pose(
    *,
    leader_x: float,
    leader_y: float,
    leader_z: float,
    leader_yaw: float,
    slot_forward: float,
    slot_left: float,
) -> PoseStamped:
    """Create a map-frame Formation Slot target for the migration seam."""

    target_x, target_y = body_offset_to_map_target(
        leader_x=leader_x,
        leader_y=leader_y,
        leader_yaw=leader_yaw,
        slot_forward=slot_forward,
        slot_left=slot_left,
    )
    target = PoseStamped()
    target.header.frame_id = "map"
    target.pose.position.x = target_x
    target.pose.position.y = target_y
    target.pose.position.z = leader_z
    target.pose.orientation.z = math.sin(leader_yaw / 2.0)
    target.pose.orientation.w = math.cos(leader_yaw / 2.0)
    return target


def odometry_yaw(odometry: Odometry) -> float:
    """Extract planar ENU yaw from an odometry quaternion."""

    orientation = odometry.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


class FixedSlotFollowerController(Node):
    """Control one follower only while the Formation Mission authorizes tracking."""

    def __init__(self) -> None:
        super().__init__("fixed_slot_follower_controller")
        vehicle_namespace = self.declare_parameter("vehicle_namespace", "MAV2").value
        leader_namespace = self.declare_parameter("leader_namespace", "MAV1").value
        self._slot_forward = float(self.declare_parameter("slot_forward", -0.8).value)
        self._slot_left = float(self.declare_parameter("slot_left", 0.8).value)
        self._telemetry_timeout = float(
            self.declare_parameter("telemetry_timeout", 0.5).value
        )
        self._phase = "idle"
        self._leader_odom: Odometry | None = None
        self._follower_odom: Odometry | None = None
        self._leader_received_ns: int | None = None
        self._follower_received_ns: int | None = None

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        phase_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._target_publisher = self.create_publisher(
            PoseStamped, f"/{vehicle_namespace}/formation_target_pose", reliable_qos
        )
        self.create_subscription(
            Odometry, f"/{leader_namespace}/odom", self._on_leader_odom, reliable_qos
        )
        self.create_subscription(
            Odometry, f"/{vehicle_namespace}/odom", self._on_follower_odom, reliable_qos
        )
        self.create_subscription(
            String, f"/{vehicle_namespace}/mission_phase", self._on_phase, phase_qos
        )
        self.create_timer(0.05, self._tick)

    def _on_leader_odom(self, odometry: Odometry) -> None:
        self._leader_odom = odometry
        self._leader_received_ns = self.get_clock().now().nanoseconds

    def _on_follower_odom(self, odometry: Odometry) -> None:
        self._follower_odom = odometry
        self._follower_received_ns = self.get_clock().now().nanoseconds

    def _on_phase(self, message: String) -> None:
        self._phase = message.data.strip().lower()

    def _telemetry_is_fresh(self, received_ns: int | None) -> bool:
        if received_ns is None:
            return False
        return (self.get_clock().now().nanoseconds - received_ns) / 1e9 <= self._telemetry_timeout

    def _tick(self) -> None:
        if self._phase not in {"form_up", "navigate_leader"}:
            return
        if not (
            self._leader_odom
            and self._follower_odom
            and self._telemetry_is_fresh(self._leader_received_ns)
            and self._telemetry_is_fresh(self._follower_received_ns)
        ):
            return

        leader_position = self._leader_odom.pose.pose.position
        leader_yaw = odometry_yaw(self._leader_odom)
        target = map_frame_formation_target_pose(
            leader_x=leader_position.x,
            leader_y=leader_position.y,
            leader_z=leader_position.z,
            leader_yaw=leader_yaw,
            slot_forward=self._slot_forward,
            slot_left=self._slot_left,
        )
        target.header.stamp = self.get_clock().now().to_msg()
        self._target_publisher.publish(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedSlotFollowerController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
