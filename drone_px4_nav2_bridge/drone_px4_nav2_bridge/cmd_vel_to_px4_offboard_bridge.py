"""Convert a vehicle-scoped FLU cmd_vel into PX4 NED Offboard input."""

from __future__ import annotations

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


def enu_position_to_ned(position: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Convert a shared-map ENU position into a PX4 local NED position."""

    east, north, up = position
    return (north, east, -up)


def flu_velocity_to_ned(
    *,
    forward: float,
    left: float,
    up: float,
    yaw_enu: float,
    yaw_rate_enu: float,
) -> Tuple[float, float, float, float]:
    """Rotate MAV1 base-link FLU velocity into PX4 local NED velocity.

    ROS Nav2 publishes x-forward, y-left and positive counter-clockwise yaw.
    PX4 TrajectorySetpoint uses north, east, down and positive NED yaw rate.
    """

    east = math.cos(yaw_enu) * forward - math.sin(yaw_enu) * left
    north = math.sin(yaw_enu) * forward + math.cos(yaw_enu) * left
    return (north, east, -up, -yaw_rate_enu)


class CmdVelToPx4OffboardBridge(Node):
    """Own MAV1 PX4 Offboard input while a mission phase authorizes it."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_to_px4_offboard_bridge")
        vehicle_namespace = self.declare_parameter("vehicle_namespace", "MAV1").value
        self._flight_level = float(self.declare_parameter("flight_level", 3.0).value)
        self._warmup_seconds = float(self.declare_parameter("warmup_seconds", 1.0).value)
        self._target_system = int(self.declare_parameter("target_system", 1).value)
        self._phase = "idle"
        self._phase_started_ns = self.get_clock().now().nanoseconds
        self._offboard_requested = False
        self._land_requested = False
        self._latest_position: Optional[VehicleLocalPosition] = None
        self._latest_command = Twist()
        self._takeoff_xy: Optional[Tuple[float, float]] = None
        self._hold_xy: Optional[Tuple[float, float]] = None

        px4_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        phase_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._offboard_publisher = self.create_publisher(
            OffboardControlMode,
            f"/{vehicle_namespace}/fmu/in/offboard_control_mode",
            px4_qos,
        )
        self._setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            f"/{vehicle_namespace}/fmu/in/trajectory_setpoint",
            px4_qos,
        )
        self._vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            f"/{vehicle_namespace}/fmu/in/vehicle_command",
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.declare_parameter(
                "px4_local_position_topic",
                f"/{vehicle_namespace}/fmu/out/vehicle_local_position_v1",
            ).value,
            self._on_local_position,
            px4_qos,
        )
        self.create_subscription(
            Twist,
            self.declare_parameter("cmd_vel_topic", f"/{vehicle_namespace}/cmd_vel").value,
            self._on_cmd_vel,
            command_qos,
        )
        self.create_subscription(
            String,
            self.declare_parameter("mission_phase_topic", f"/{vehicle_namespace}/mission_phase").value,
            self._on_phase,
            phase_qos,
        )
        self.create_timer(0.05, self._tick)

    def _on_local_position(self, message: VehicleLocalPosition) -> None:
        if not all((message.xy_valid, message.z_valid)):
            return
        if not all(math.isfinite(value) for value in (message.x, message.y, message.z, message.heading)):
            return
        self._latest_position = message

    def _on_cmd_vel(self, message: Twist) -> None:
        self._latest_command = message

    def _on_phase(self, message: String) -> None:
        phase = message.data.strip().lower()
        if phase not in {
            "idle", "preflight", "warmup", "takeoff", "navigate", "abort_hold", "land"
        }:
            self.get_logger().warn(f"Ignoring unknown mission phase: {message.data}")
            return
        if phase == self._phase:
            return
        self._phase = phase
        self._phase_started_ns = self.get_clock().now().nanoseconds
        if phase == "warmup":
            self._offboard_requested = False
            self._land_requested = False
        if phase == "takeoff" and self._latest_position is not None:
            self._takeoff_xy = (self._latest_position.x, self._latest_position.y)
        if phase == "abort_hold" and self._latest_position is not None:
            self._hold_xy = (self._latest_position.x, self._latest_position.y)
        if phase == "land":
            self._land_requested = False

    def _tick(self) -> None:
        if self._phase in {"idle", "preflight"} or self._latest_position is None:
            return
        if self._phase == "land":
            if not self._land_requested:
                self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._land_requested = True
            return

        self._publish_heartbeat(velocity=self._phase == "navigate")
        if self._phase == "warmup":
            self._publish_position_hold(self._latest_position.x, self._latest_position.y, self._latest_position.z)
            return

        if self._phase == "takeoff":
            elapsed_s = (self.get_clock().now().nanoseconds - self._phase_started_ns) / 1e9
            if not self._offboard_requested and elapsed_s >= self._warmup_seconds:
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )
                self._offboard_requested = True
            target_x, target_y = self._takeoff_xy or (
                self._latest_position.x,
                self._latest_position.y,
            )
            self._publish_position_hold(target_x, target_y, -self._flight_level)
            return

        if self._phase == "abort_hold":
            target_x, target_y = self._hold_xy or (
                self._latest_position.x,
                self._latest_position.y,
            )
            self._publish_position_hold(target_x, target_y, -self._flight_level)
            return

        self._publish_navigation_setpoint()

    def _publish_heartbeat(self, velocity: bool) -> None:
        message = OffboardControlMode()
        message.timestamp = self._timestamp_us()
        message.position = True
        message.velocity = velocity
        self._offboard_publisher.publish(message)

    def _publish_position_hold(self, north: float, east: float, down: float) -> None:
        message = TrajectorySetpoint()
        message.timestamp = self._timestamp_us()
        message.position = [north, east, down]
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = math.nan
        message.yawspeed = math.nan
        self._setpoint_publisher.publish(message)

    def _publish_navigation_setpoint(self) -> None:
        assert self._latest_position is not None
        yaw_enu = math.pi / 2.0 - self._latest_position.heading
        north, east, _, yaw_rate = flu_velocity_to_ned(
            forward=self._latest_command.linear.x,
            left=self._latest_command.linear.y,
            up=0.0,
            yaw_enu=yaw_enu,
            yaw_rate_enu=self._latest_command.angular.z,
        )
        message = TrajectorySetpoint()
        message.timestamp = self._timestamp_us()
        message.position = [math.nan, math.nan, -self._flight_level]
        message.velocity = [north, east, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = math.nan
        message.yawspeed = yaw_rate
        self._setpoint_publisher.publish(message)

    def _publish_vehicle_command(self, command: int, **parameters: float) -> None:
        message = VehicleCommand()
        message.timestamp = self._timestamp_us()
        message.command = command
        message.param1 = parameters.get("param1", math.nan)
        message.param2 = parameters.get("param2", math.nan)
        message.param3 = parameters.get("param3", math.nan)
        message.param4 = parameters.get("param4", math.nan)
        message.param5 = parameters.get("param5", math.nan)
        message.param6 = parameters.get("param6", math.nan)
        message.param7 = parameters.get("param7", math.nan)
        message.target_system = self._target_system
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self._vehicle_command_publisher.publish(message)

    def _timestamp_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToPx4OffboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
