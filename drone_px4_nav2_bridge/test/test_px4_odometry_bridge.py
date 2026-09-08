"""Behavioural contract for MAV1 PX4-local-position conversion."""

import math
from types import SimpleNamespace
import time
import unittest

from drone_px4_nav2_bridge.px4_odometry_bridge import local_position_to_enu
from drone_px4_nav2_bridge.px4_odometry_bridge import Px4OdometryBridge
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


class LocalPositionToEnuTest(unittest.TestCase):
    def valid_local_position(self, **overrides):
        values = {
            "xy_valid": True,
            "z_valid": True,
            "v_xy_valid": True,
            "v_z_valid": True,
            "x": 3.0,
            "y": 7.0,
            "z": -2.0,
            "vx": 1.5,
            "vy": -4.0,
            "vz": 0.25,
            "heading": 0.0,
        }
        values.update(overrides)
        return SimpleNamespace(**values)

    def test_converts_known_ned_position_velocity_and_heading_to_enu(self):
        converted = local_position_to_enu(self.valid_local_position())

        self.assertIsNotNone(converted)
        self.assertEqual(converted.position, (7.0, 3.0, 2.0))
        self.assertEqual(converted.velocity, (-4.0, 1.5, -0.25))
        self.assertAlmostEqual(converted.yaw, math.pi / 2.0)

    def test_rejects_an_invalid_px4_position_estimate(self):
        converted = local_position_to_enu(self.valid_local_position(xy_valid=False))

        self.assertIsNone(converted)

    def test_rejects_non_finite_position_or_heading(self):
        self.assertIsNone(
            local_position_to_enu(self.valid_local_position(z=math.nan))
        )
        self.assertIsNone(
            local_position_to_enu(self.valid_local_position(heading=math.inf))
        )


class Px4OdometryBridgeRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.bridge = Px4OdometryBridge()
        self.observer = Node("px4_odometry_bridge_test_observer")
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.observer)
        self.odometry_messages = []
        self.tf_messages = []

        px4_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        tf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.local_position_publisher = self.observer.create_publisher(
            VehicleLocalPosition,
            "/MAV1/fmu/out/vehicle_local_position_v1",
            px4_qos,
        )
        self.observer.create_subscription(
            Odometry,
            "/MAV1/odom",
            self.odometry_messages.append,
            10,
        )
        self.observer.create_subscription(
            TFMessage,
            "/tf",
            self.tf_messages.append,
            tf_qos,
        )

    def tearDown(self):
        self.executor.remove_node(self.observer)
        self.executor.remove_node(self.bridge)
        self.observer.destroy_node()
        self.bridge.destroy_node()
        self.executor.shutdown()

    @staticmethod
    def valid_message():
        message = VehicleLocalPosition()
        message.xy_valid = True
        message.z_valid = True
        message.v_xy_valid = True
        message.v_z_valid = True
        message.x = 3.0
        message.y = 7.0
        message.z = -2.0
        message.vx = 1.5
        message.vy = -4.0
        message.vz = 0.25
        message.heading = 0.0
        return message

    def publish_until(self, predicate):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.local_position_publisher.publish(self.valid_message())
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        self.fail("Bridge did not publish the expected ROS output")

    def test_valid_px4_message_publishes_prefixed_odom_and_tf(self):
        self.publish_until(lambda: self.odometry_messages and self.tf_messages)

        odometry = self.odometry_messages[-1]
        self.assertEqual(odometry.header.frame_id, "MAV1/odom")
        self.assertEqual(odometry.child_frame_id, "MAV1/base_link")
        self.assertEqual(
            (
                odometry.pose.pose.position.x,
                odometry.pose.pose.position.y,
                odometry.pose.pose.position.z,
            ),
            (7.0, 3.0, 2.0),
        )
        self.assertAlmostEqual(odometry.pose.pose.orientation.z, math.sqrt(0.5))
        self.assertAlmostEqual(odometry.pose.pose.orientation.w, math.sqrt(0.5))

        transforms = [
            transform
            for tf_message in self.tf_messages
            for transform in tf_message.transforms
            if transform.header.frame_id == "MAV1/odom"
            and transform.child_frame_id == "MAV1/base_link"
        ]
        self.assertTrue(transforms)
        self.assertEqual(transforms[-1].transform.translation.x, 7.0)
        self.assertEqual(transforms[-1].transform.translation.y, 3.0)
        self.assertEqual(transforms[-1].transform.translation.z, 2.0)

    def test_invalid_px4_position_does_not_publish_odometry(self):
        invalid_message = self.valid_message()
        invalid_message.xy_valid = False

        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            self.local_position_publisher.publish(invalid_message)
            self.executor.spin_once(timeout_sec=0.05)

        self.assertEqual(self.odometry_messages, [])

    def test_bridge_does_not_create_a_flight_control_publisher(self):
        publisher_topics = {
            topic_name
            for topic_name, _ in self.bridge.get_publisher_names_and_types_by_node(
                self.bridge.get_name(), self.bridge.get_namespace()
            )
        }

        self.assertIn("/MAV1/odom", publisher_topics)
        self.assertIn("/tf", publisher_topics)
        forbidden_topics = {
            "/cmd_vel",
            "/MAV1/fmu/in/offboard_control_mode",
            "/MAV1/fmu/in/trajectory_setpoint",
            "/MAV1/fmu/in/vehicle_command",
        }
        self.assertTrue(forbidden_topics.isdisjoint(publisher_topics))


if __name__ == "__main__":
    unittest.main()
