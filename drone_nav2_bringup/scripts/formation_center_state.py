#!/usr/bin/env python3
"""Expose an optional Formation Center state derived from physical MAV1."""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Iterable, Sequence

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


Point2 = tuple[float, float]


def formation_centroid(points: Iterable[Point2]) -> Point2:
    """Return the arithmetic centroid of Leader and Fixed V-Slot centers."""
    centers = tuple(points)
    if not centers:
        raise ValueError("formation requires at least one vehicle center")
    return (
        sum(point[0] for point in centers) / len(centers),
        sum(point[1] for point in centers) / len(centers),
    )


def formation_envelope_radius(
    points: Iterable[Point2],
    *,
    center: Point2,
    vehicle_radius: float,
    safety_margin: float,
    rounding_increment: float,
) -> float:
    """Return a conservatively rounded circle enclosing every vehicle body."""
    centers = tuple(points)
    if not centers:
        raise ValueError("formation requires at least one vehicle center")
    if vehicle_radius < 0.0 or safety_margin < 0.0 or rounding_increment <= 0.0:
        raise ValueError("formation radii must be non-negative and rounding positive")
    required = max(
        math.hypot(point[0] - center[0], point[1] - center[1])
        for point in centers
    ) + vehicle_radius + safety_margin
    return math.ceil((required - 1e-12) / rounding_increment) * rounding_increment


def validated_envelope_radius(required: float, configured: float) -> float:
    """Reject a configured collision radius smaller than the formation requires."""
    if configured + 1e-12 < required:
        raise ValueError("envelope_radius cannot be smaller than formation geometry")
    return configured


def virtual_center_transform(
    *, parent_frame: str, child_frame: str, forward: float, left: float
) -> TransformStamped:
    """Describe the fixed body-frame offset from physical MAV1 to its center."""
    transform = TransformStamped()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = forward
    transform.transform.translation.y = left
    transform.transform.rotation.w = 1.0
    return transform


def _odometry_yaw(odometry: Odometry) -> float:
    orientation = odometry.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


def virtual_center_odometry(
    leader: Odometry,
    *,
    child_frame: str,
    offset_forward: float,
    offset_left: float,
    yaw_rate: float,
) -> Odometry:
    """Derive Formation Center map pose and velocity from physical MAV1 state."""
    center = deepcopy(leader)
    yaw = _odometry_yaw(leader)
    offset_x = math.cos(yaw) * offset_forward - math.sin(yaw) * offset_left
    offset_y = math.sin(yaw) * offset_forward + math.cos(yaw) * offset_left
    center.child_frame_id = child_frame
    center.pose.pose.position.x += offset_x
    center.pose.pose.position.y += offset_y
    map_east = leader.twist.twist.linear.x
    map_north = leader.twist.twist.linear.y
    center.twist.twist.linear.x = (
        math.cos(yaw) * map_east
        + math.sin(yaw) * map_north
        - yaw_rate * offset_left
    )
    center.twist.twist.linear.y = (
        -math.sin(yaw) * map_east
        + math.cos(yaw) * map_north
        + yaw_rate * offset_forward
    )
    center.twist.twist.angular.z = yaw_rate
    return center


def physical_mav1_command(
    virtual_command: Twist, *, center_forward: float, center_left: float
) -> Twist:
    """Convert a Formation Center body command into physical MAV1 body velocity."""
    command = deepcopy(virtual_command)
    yaw_rate = virtual_command.angular.z
    command.linear.x += yaw_rate * center_left
    command.linear.y -= yaw_rate * center_forward
    return command


class FormationCenterState(Node):
    """Publish an optional safety state without redefining physical MAV1."""

    def __init__(self) -> None:
        super().__init__("formation_center_state")
        self._leader_frame = self.declare_parameter(
            "leader_frame", "MAV1/base_link"
        ).value
        self._center_frame = self.declare_parameter(
            "center_frame", "MAV1/formation_center"
        ).value
        points: Sequence[Point2] = (
            (0.0, 0.0),
            (
                float(self.declare_parameter("mav2_slot_forward", -0.8).value),
                float(self.declare_parameter("mav2_slot_left", 0.8).value),
            ),
            (
                float(self.declare_parameter("mav3_slot_forward", -0.8).value),
                float(self.declare_parameter("mav3_slot_left", -0.8).value),
            ),
        )
        self._center = formation_centroid(points)
        required_envelope_radius = formation_envelope_radius(
            points,
            center=self._center,
            vehicle_radius=float(self.declare_parameter("vehicle_radius", 0.4).value),
            safety_margin=float(self.declare_parameter("safety_margin", 0.2).value),
            rounding_increment=float(
                self.declare_parameter("envelope_rounding_increment", 0.05).value
            ),
        )
        self._envelope_radius = validated_envelope_radius(
            required_envelope_radius,
            float(
                self.declare_parameter(
                    "envelope_radius", required_envelope_radius
                ).value
            ),
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._odometry_publisher = self.create_publisher(
            Odometry,
            self.declare_parameter(
                "virtual_odometry_topic", "/MAV1/formation_center/map_odom"
            ).value,
            reliable_qos,
        )
        self._physical_command_publisher = self.create_publisher(
            Twist,
            self.declare_parameter(
                "physical_command_topic", "/MAV1/formation_center/physical_cmd_vel"
            ).value,
            reliable_qos,
        )
        self._radius_publisher = self.create_publisher(
            Float64,
            self.declare_parameter(
                "envelope_radius_topic", "/MAV1/formation_center/envelope_radius"
            ).value,
            state_qos,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter("leader_odometry_topic", "/MAV1/map_odom").value,
            self._on_leader_odometry,
            reliable_qos,
        )
        self.create_subscription(
            Twist,
            self.declare_parameter(
                "virtual_command_topic", "/MAV1/formation_center/cmd_vel"
            ).value,
            self._on_virtual_command,
            reliable_qos,
        )
        self._previous_yaw: float | None = None
        self._previous_stamp: float | None = None
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        transform = virtual_center_transform(
            parent_frame=self._leader_frame,
            child_frame=self._center_frame,
            forward=self._center[0],
            left=self._center[1],
        )
        transform.header.stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(transform)
        self._radius_publisher.publish(Float64(data=self._envelope_radius))

    def _on_leader_odometry(self, odometry: Odometry) -> None:
        yaw = _odometry_yaw(odometry)
        stamp = odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1e9
        yaw_rate = odometry.twist.twist.angular.z
        if self._previous_yaw is not None and self._previous_stamp is not None:
            elapsed = stamp - self._previous_stamp
            if elapsed > 1e-6:
                yaw_rate = math.atan2(
                    math.sin(yaw - self._previous_yaw),
                    math.cos(yaw - self._previous_yaw),
                ) / elapsed
        self._previous_yaw = yaw
        self._previous_stamp = stamp
        self._odometry_publisher.publish(
            virtual_center_odometry(
                odometry,
                child_frame=self._center_frame,
                offset_forward=self._center[0],
                offset_left=self._center[1],
                yaw_rate=yaw_rate,
            )
        )

    def _on_virtual_command(self, command: Twist) -> None:
        self._physical_command_publisher.publish(
            physical_mav1_command(
                command,
                center_forward=self._center[0],
                center_left=self._center[1],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationCenterState()
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
