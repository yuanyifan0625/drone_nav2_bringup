#!/usr/bin/env python3
"""Track one yaw-relative Fixed V-Slot and publish vehicle-scoped FLU cmd_vel."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, Twist
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


def map_error_to_flu(
    *, error_east: float, error_north: float, follower_yaw: float
) -> tuple[float, float]:
    """Express a map-frame ENU position error in the follower's FLU frame."""

    return (
        math.cos(follower_yaw) * error_east + math.sin(follower_yaw) * error_north,
        -math.sin(follower_yaw) * error_east + math.cos(follower_yaw) * error_north,
    )


def normalized_angle(angle: float) -> float:
    """Return an angle in [-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


def limited_follower_command(
    *,
    error_forward: float,
    error_left: float,
    yaw_error: float,
    position_gain: float,
    yaw_gain: float,
    max_linear_speed: float,
    max_yaw_rate: float,
) -> tuple[float, float, float]:
    """Create bounded FLU velocity intent from body-frame and yaw errors."""

    forward = position_gain * error_forward
    left = position_gain * error_left
    speed = math.hypot(forward, left)
    if speed > max_linear_speed > 0.0:
        scale = max_linear_speed / speed
        forward *= scale
        left *= scale
    yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_gain * yaw_error))
    return forward, left, yaw_rate


def prevent_leader_closing_command(
    *,
    forward: float,
    left: float,
    follower_yaw: float,
    leader_x: float,
    leader_y: float,
    follower_x: float,
    follower_y: float,
    minimum_leader_distance: float,
) -> tuple[float, float]:
    """Stop planar motion that would reduce an already-close Leader separation."""

    east_from_leader = follower_x - leader_x
    north_from_leader = follower_y - leader_y
    separation = math.hypot(east_from_leader, north_from_leader)
    if separation <= 0.0 or separation > minimum_leader_distance:
        return forward, left
    command_east = math.cos(follower_yaw) * forward - math.sin(follower_yaw) * left
    command_north = math.sin(follower_yaw) * forward + math.cos(follower_yaw) * left
    if command_east * east_from_leader + command_north * north_from_leader < 0.0:
        return 0.0, 0.0
    return forward, left


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
        self._position_gain = float(self.declare_parameter("position_gain", 1.0).value)
        self._yaw_gain = float(self.declare_parameter("yaw_gain", 1.5).value)
        self._max_linear_speed = float(
            self.declare_parameter("max_linear_speed", 0.6).value
        )
        self._max_yaw_rate = float(self.declare_parameter("max_yaw_rate", 0.8).value)
        self._minimum_leader_distance = float(
            self.declare_parameter("minimum_leader_distance", 0.9).value
        )
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
        self._publisher = self.create_publisher(Twist, f"/{vehicle_namespace}/cmd_vel", reliable_qos)
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
        command = Twist()
        if not (
            self._leader_odom
            and self._follower_odom
            and self._telemetry_is_fresh(self._leader_received_ns)
            and self._telemetry_is_fresh(self._follower_received_ns)
        ):
            self._publisher.publish(command)
            return

        leader_position = self._leader_odom.pose.pose.position
        follower_position = self._follower_odom.pose.pose.position
        leader_yaw = odometry_yaw(self._leader_odom)
        follower_yaw = odometry_yaw(self._follower_odom)
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
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        forward_error, left_error = map_error_to_flu(
            error_east=target_x - follower_position.x,
            error_north=target_y - follower_position.y,
            follower_yaw=follower_yaw,
        )
        command.linear.x, command.linear.y, command.angular.z = limited_follower_command(
            error_forward=forward_error,
            error_left=left_error,
            yaw_error=normalized_angle(leader_yaw - follower_yaw),
            position_gain=self._position_gain,
            yaw_gain=self._yaw_gain,
            max_linear_speed=self._max_linear_speed,
            max_yaw_rate=self._max_yaw_rate,
        )
        command.linear.x, command.linear.y = prevent_leader_closing_command(
            forward=command.linear.x,
            left=command.linear.y,
            follower_yaw=follower_yaw,
            leader_x=leader_position.x,
            leader_y=leader_position.y,
            follower_x=follower_position.x,
            follower_y=follower_position.y,
            minimum_leader_distance=self._minimum_leader_distance,
        )
        self._publisher.publish(command)


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
