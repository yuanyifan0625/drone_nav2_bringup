"""Launch-level contracts for one vehicle-scoped Fixed V-Slot Follower."""

import math
import os
import signal
import subprocess
import time
import unittest

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class FixedSlotFollowerControllerLaunchTest(unittest.TestCase):
    """MAV2 only commands MAV2 and fails closed on stale Leader odometry."""

    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "112"
        cls.process = subprocess.Popen(
            [
                "ros2", "run", "drone_nav2_bringup", "fixed_slot_follower_controller.py",
                "--ros-args", "-p", "vehicle_namespace:=MAV2",
                "-p", "leader_namespace:=MAV1", "-p", "telemetry_timeout:=0.2",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env=os.environ.copy(),
        )
        rclpy.init()
        cls.node = Node("fixed_slot_follower_controller_launch_test")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cls.leader_publisher = cls.node.create_publisher(Odometry, "/MAV1/odom", qos)
        cls.follower_publisher = cls.node.create_publisher(Odometry, "/MAV2/odom", qos)
        cls.phase_publisher = cls.node.create_publisher(
            String,
            "/MAV2/mission_phase",
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.commands = []
        cls.node.create_subscription(Twist, "/MAV2/cmd_vel", cls.commands.append, qos)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.killpg(cls.process.pid, signal.SIGINT)
        cls.process.wait(timeout=5.0)

    @staticmethod
    def odometry(x: float, y: float, yaw: float = 0.0) -> Odometry:
        message = Odometry()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return message

    @classmethod
    def spin_until(cls, predicate, publish_leader=True, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish_leader:
                cls.leader_publisher.publish(cls.odometry(0.0, 0.0))
            cls.follower_publisher.publish(cls.odometry(0.0, 0.0))
            cls.phase_publisher.publish(String(data="form_up"))
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("Follower controller did not satisfy its public contract")

    def test_vehicle_scoped_tracking_and_stale_leader_fails_closed(self):
        self.spin_until(
            lambda: any(abs(command.linear.x) > 0.01 for command in self.commands)
        )
        self.assertEqual(
            1, len(self.node.get_publishers_info_by_topic("/MAV2/cmd_vel"))
        )
        command_count = len(self.commands)
        self.spin_until(
            lambda: any(
                abs(command.linear.x) < 1e-9
                and abs(command.linear.y) < 1e-9
                and abs(command.angular.z) < 1e-9
                for command in self.commands[command_count:]
            ),
            publish_leader=False,
            timeout=2.0,
        )


if __name__ == "__main__":
    unittest.main()
