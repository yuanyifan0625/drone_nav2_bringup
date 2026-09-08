"""Publish MAV1 ENU odometry and TF from PX4 local NED position estimates."""

from dataclasses import dataclass
import math
from typing import Optional, Protocol, Tuple

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster


class LocalPositionEstimate(Protocol):
    """PX4 fields required to construct an ENU odometry estimate."""

    xy_valid: bool
    z_valid: bool
    v_xy_valid: bool
    v_z_valid: bool
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    heading: float


@dataclass(frozen=True)
class EnuLocalState:
    """Validated PX4 local-position data expressed in ENU coordinates."""

    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    yaw: float


def local_position_to_enu(
    local_position: LocalPositionEstimate,
) -> Optional[EnuLocalState]:
    """Convert one valid PX4 local NED estimate into an ENU local state.

    PX4 local position is North-East-Down (NED), while ROS navigation uses
    East-North-Up (ENU). PX4 heading is clockwise from North; ROS yaw is
    counter-clockwise from East.
    """

    validity_flags = (
        local_position.xy_valid,
        local_position.z_valid,
        local_position.v_xy_valid,
        local_position.v_z_valid,
    )
    values = (
        local_position.x,
        local_position.y,
        local_position.z,
        local_position.vx,
        local_position.vy,
        local_position.vz,
        local_position.heading,
    )
    if not all(validity_flags) or not all(math.isfinite(value) for value in values):
        return None

    return EnuLocalState(
        position=(local_position.y, local_position.x, -local_position.z),
        velocity=(local_position.vy, local_position.vx, -local_position.vz),
        yaw=math.pi / 2.0 - local_position.heading,
    )


class Px4OdometryBridge(Node):
    """Bridge a PX4 VehicleLocalPosition topic into MAV1 ROS odometry and TF."""

    def __init__(self) -> None:
        super().__init__("px4_odometry_bridge")

        vehicle_prefix = self.declare_parameter("vehicle_prefix", "MAV1").value
        px4_topic = self.declare_parameter(
            "px4_topic", "/MAV1/fmu/out/vehicle_local_position_v1"
        ).value
        odom_topic = self.declare_parameter("odom_topic", "/MAV1/odom").value
        self._odom_frame = self.declare_parameter(
            "odom_frame", f"{vehicle_prefix}/odom"
        ).value
        self._base_frame = self.declare_parameter(
            "base_frame", f"{vehicle_prefix}/base_link"
        ).value

        px4_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._odom_publisher = self.create_publisher(Odometry, odom_topic, odom_qos)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            VehicleLocalPosition,
            px4_topic,
            self._on_local_position,
            px4_qos,
        )
        self.get_logger().info(
            "Bridging %s to %s and %s -> %s"
            % (px4_topic, odom_topic, self._odom_frame, self._base_frame)
        )

    def _on_local_position(self, local_position: VehicleLocalPosition) -> None:
        enu_state = local_position_to_enu(local_position)
        if enu_state is None:
            self.get_logger().warn(
                "Ignoring invalid PX4 VehicleLocalPosition estimate",
                throttle_duration_sec=5.0,
            )
            return

        timestamp = self.get_clock().now().to_msg()
        orientation_z = math.sin(enu_state.yaw / 2.0)
        orientation_w = math.cos(enu_state.yaw / 2.0)

        odometry = Odometry()
        odometry.header.stamp = timestamp
        odometry.header.frame_id = self._odom_frame
        odometry.child_frame_id = self._base_frame
        odometry.pose.pose.position.x = enu_state.position[0]
        odometry.pose.pose.position.y = enu_state.position[1]
        odometry.pose.pose.position.z = enu_state.position[2]
        odometry.pose.pose.orientation.z = orientation_z
        odometry.pose.pose.orientation.w = orientation_w
        odometry.twist.twist.linear.x = enu_state.velocity[0]
        odometry.twist.twist.linear.y = enu_state.velocity[1]
        odometry.twist.twist.linear.z = enu_state.velocity[2]
        self._odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = enu_state.position[0]
        transform.transform.translation.y = enu_state.position[1]
        transform.transform.translation.z = enu_state.position[2]
        transform.transform.rotation.z = orientation_z
        transform.transform.rotation.w = orientation_w
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4OdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
