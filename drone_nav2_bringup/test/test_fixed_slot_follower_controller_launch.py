"""Launch-level contracts for one vehicle-scoped Fixed V-Slot Follower."""

import math
import os
import signal
import subprocess
import time
import unittest

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class FixedSlotFollowerControllerLaunchTest(unittest.TestCase):
    """MAV2 publishes only its yaw-relative Formation Target Pose."""

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
        cls.leader_publisher = cls.node.create_publisher(Odometry, "/MAV1/map_odom", qos)
        cls.follower_publisher = cls.node.create_publisher(Odometry, "/MAV2/map_odom", qos)
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
        cls.targets = []
        cls.node.create_subscription(
            PoseStamped, "/MAV2/formation_target_pose", cls.targets.append, qos
        )

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

    def test_map_frame_target_pose_is_vehicle_scoped_and_periodic(self):
        self.targets.clear()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and len(self.targets) < 3:
            self.leader_publisher.publish(self.odometry(2.0, 1.0, math.pi / 2.0))
            self.follower_publisher.publish(self.odometry(0.0, 0.0))
            self.phase_publisher.publish(String(data="form_up"))
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertGreaterEqual(len(self.targets), 3)
        self.assertEqual(
            1, len(self.node.get_publishers_info_by_topic("/MAV2/formation_target_pose"))
        )
        target = self.targets[-1]
        self.assertEqual("map", target.header.frame_id)
        self.assertTrue(math.isclose(target.pose.position.x, 1.2, abs_tol=1e-6))
        self.assertTrue(math.isclose(target.pose.position.y, 0.2, abs_tol=1e-6))


    def test_controller_has_no_direct_cmd_vel_publisher(self):
        self.assertEqual(
            0, len(self.node.get_publishers_info_by_topic("/MAV2/cmd_vel"))
        )


if __name__ == "__main__":
    unittest.main()
