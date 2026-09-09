"""Public MAV1 fixed-height mission lifecycle without PX4 SITL."""

import os
import signal
import subprocess
import time
import unittest

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, String


class Mav1OffboardMissionLaunchTest(unittest.TestCase):
    """Mission lifecycle authorizes fixed-height PX4 input only in its phases."""

    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "109"
        cls.process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "drone_nav2_bringup",
                "mav1_offboard_mission.launch.py",
                "use_sim_time:=false",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env=os.environ.copy(),
        )
        rclpy.init()
        cls.node = Node("mav1_offboard_mission_launch_test")
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
        cls.position_publisher = cls.node.create_publisher(
            VehicleLocalPosition, "/MAV1/fmu/out/vehicle_local_position_v1", px4_qos
        )
        cls.goal_publisher = cls.node.create_publisher(
            PoseStamped, "/MAV1/mission_goal", 10
        )
        cls.formation_goal_publisher = cls.node.create_publisher(
            PoseStamped, "/MAV1/formation_goal_pose", 10
        )
        cls.cancel_publisher = cls.node.create_publisher(
            Empty, "/MAV1/mission_cancel", 10
        )
        cls.phases = []
        cls.commands = []
        cls.setpoints = []
        cls.node.create_subscription(String, "/MAV1/mission_phase", cls.phases.append, phase_qos)
        cls.node.create_subscription(
            VehicleCommand, "/MAV1/fmu/in/vehicle_command", cls.commands.append, px4_qos
        )
        cls.node.create_subscription(
            TrajectorySetpoint,
            "/MAV1/fmu/in/trajectory_setpoint",
            cls.setpoints.append,
            px4_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.killpg(cls.process.pid, signal.SIGINT)
        cls.process.wait(timeout=5.0)

    @classmethod
    def spin_with_position_until(cls, predicate, timeout=20.0):
        position = VehicleLocalPosition()
        position.xy_valid = True
        position.z_valid = True
        position.v_xy_valid = True
        position.v_z_valid = True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.position_publisher.publish(position)
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("MAV1 mission launch did not satisfy its public contract")

    @classmethod
    def spin_with_position_for(cls, duration):
        position = VehicleLocalPosition()
        position.xy_valid = True
        position.z_valid = True
        position.v_xy_valid = True
        position.v_z_valid = True
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            cls.position_publisher.publish(position)
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    def test_cancel_enters_abort_hold_before_px4_land(self):
        self.spin_with_position_until(
            lambda: any(message.data == "idle" for message in self.phases)
        )
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 1.0
        goal.pose.position.y = -6.5
        goal.pose.orientation.w = 1.0
        self.formation_goal_publisher.publish(goal)
        self.spin_with_position_until(
            lambda: any(message.data == "takeoff" for message in self.phases)
        )
        self.spin_with_position_until(
            lambda: any(
                command.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                for command in self.commands
            )
        )
        self.assertTrue(
            any(setpoint.position[2] == -3.0 for setpoint in self.setpoints)
        )
        phase_count_before_replacement = len(self.phases)
        replacement_goal = PoseStamped()
        replacement_goal.header.frame_id = "map"
        replacement_goal.pose.position.x = 6.5
        replacement_goal.pose.position.y = -6.5
        replacement_goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(replacement_goal)
        self.spin_with_position_for(0.5)
        self.assertEqual(phase_count_before_replacement, len(self.phases))
        self.assertEqual("takeoff", self.phases[-1].data)
        self.cancel_publisher.publish(Empty())
        self.spin_with_position_until(
            lambda: any(message.data == "abort_hold" for message in self.phases)
        )
        self.spin_with_position_until(
            lambda: any(message.data == "land" for message in self.phases)
        )
        self.spin_with_position_until(
            lambda: any(
                command.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
                for command in self.commands
            )
        )


if __name__ == "__main__":
    unittest.main()
