"""Public FLU, ENU and NED contracts for the MAV1 PX4 interface."""

import math
import os
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
from drone_px4_nav2_bridge.cmd_vel_to_px4_offboard_bridge import (
    enu_position_to_ned,
    flu_velocity_to_ned,
)
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


def test_enu_flight_level_becomes_negative_px4_ned_down() -> None:
    """A positive ENU Flight Level maps to PX4's negative down coordinate."""

    assert enu_position_to_ned((4.0, -2.0, 3.0)) == (-2.0, 4.0, -3.0)


def test_body_flu_velocity_rotates_by_enu_yaw_before_ned_conversion() -> None:
    """Forward/left MAV1 cmd_vel becomes a world NED velocity."""

    north, east, down, yaw_rate = flu_velocity_to_ned(
        forward=0.4,
        left=0.2,
        up=0.0,
        yaw_enu=math.pi / 2.0,
        yaw_rate_enu=0.3,
    )

    assert math.isclose(north, 0.4, abs_tol=1e-9)
    assert math.isclose(east, -0.2, abs_tol=1e-9)
    assert down == 0.0
    assert math.isclose(yaw_rate, -0.3, abs_tol=1e-9)


def test_navigate_phase_is_the_only_cmd_vel_to_px4_input_authority() -> None:
    """The public mission phase authorizes one bridge publisher and NED command."""

    os.environ["ROS_DOMAIN_ID"] = "108"
    process = subprocess.Popen(
        ["ros2", "run", "drone_px4_nav2_bridge", "cmd_vel_to_px4_offboard_bridge"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        env=os.environ.copy(),
    )
    rclpy.init()
    node = Node("cmd_vel_to_px4_interface_test")
    px4_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
    phase_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    position_publisher = node.create_publisher(
        VehicleLocalPosition, "/MAV1/fmu/out/vehicle_local_position_v1", px4_qos
    )
    command_publisher = node.create_publisher(Twist, "/MAV1/cmd_vel", 10)
    phase_publisher = node.create_publisher(String, "/MAV1/mission_phase", phase_qos)
    setpoints = []
    node.create_subscription(
        TrajectorySetpoint,
        "/MAV1/fmu/in/trajectory_setpoint",
        setpoints.append,
        px4_qos,
    )

    try:
        position = VehicleLocalPosition()
        position.xy_valid = True
        position.z_valid = True
        position.heading = math.pi / 2.0
        command = Twist()
        command.linear.x = 0.4
        command.linear.y = 0.2
        command.angular.z = 0.3
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not any(
            abs(setpoint.velocity[0]) > 1e-6 for setpoint in setpoints
        ):
            position_publisher.publish(position)
            command_publisher.publish(command)
            phase_publisher.publish(String(data="navigate"))
            rclpy.spin_once(node, timeout_sec=0.05)
        setpoint = next(
            setpoint
            for setpoint in reversed(setpoints)
            if abs(setpoint.velocity[0]) > 1e-6
        )
        assert math.isnan(setpoint.position[0])
        assert math.isnan(setpoint.position[1])
        assert setpoint.position[2] == -3.0
        assert math.isclose(setpoint.velocity[0], 0.2, abs_tol=1e-6)
        assert math.isclose(setpoint.velocity[1], 0.4, abs_tol=1e-6)
        assert math.isclose(setpoint.yawspeed, -0.3, abs_tol=1e-6)
        assert len(node.get_publishers_info_by_topic(
            "/MAV1/fmu/in/trajectory_setpoint"
        )) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5.0)
