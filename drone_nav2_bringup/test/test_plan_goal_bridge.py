"""End-to-end contract for the MAV1 Plan Goal Bridge."""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import math
from px4_msgs.msg import VehicleLocalPosition
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import signal
import subprocess
import time
from tf2_msgs.msg import TFMessage
import unittest


class PlanGoalBridgeTest(unittest.TestCase):
    @staticmethod
    def persistent_plan_qos():
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

    @classmethod
    def setUpClass(cls):
        # Keep this self-contained launch separate from any active MAV1/SITL graph.
        os.environ["ROS_DOMAIN_ID"] = "103"
        cls.launch_process = subprocess.Popen(
            ["ros2", "launch", "drone_nav2_bringup", "plan_only.launch.py", "use_sim_time:=false", "rviz:=false"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=os.environ.copy(),
        )
        rclpy.init()
        cls.node = Node("plan_goal_bridge_integration_test")
        px4_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
        cls.position_publisher = cls.node.create_publisher(VehicleLocalPosition, "/MAV1/fmu/out/vehicle_local_position_v1", px4_qos)
        cls.goal_publisher = cls.node.create_publisher(PoseStamped, "/MAV1/goal_pose", 10)
        cls.plans = []
        cls.maps = []
        cls.tf_messages = []
        cls.static_tf_messages = []
        cls.node.create_subscription(
            Path, "/MAV1/plan", cls.plans.append, cls.persistent_plan_qos()
        )
        cls.node.create_subscription(
            OccupancyGrid, "/map", cls.maps.append, cls.persistent_plan_qos()
        )
        tf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        cls.node.create_subscription(TFMessage, "/tf", cls.tf_messages.append, tf_qos)
        cls.node.create_subscription(
            TFMessage,
            "/tf_static",
            cls.static_tf_messages.append,
            cls.persistent_plan_qos(),
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.killpg(cls.launch_process.pid, signal.SIGINT)
        try:
            cls.launch_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(cls.launch_process.pid, signal.SIGTERM)
            cls.launch_process.wait(timeout=5.0)

    @classmethod
    def spin_with_pose_until(cls, predicate, timeout=15.0):
        pose = VehicleLocalPosition()
        pose.xy_valid = pose.z_valid = pose.v_xy_valid = pose.v_z_valid = True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.position_publisher.publish(pose)
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("Plan Goal Bridge did not produce the expected output")

    @classmethod
    def wait_for_graph_ready(cls):
        """Allow the launch graph, action client, and ROS discovery to settle."""
        ready_at = time.monotonic() + 3.0
        cls.spin_with_pose_until(lambda: time.monotonic() >= ready_at)

    @staticmethod
    def goal(frame_id, x, y):
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        return goal

    def test_map_goal_publishes_path_and_invalid_goal_clears_it(self):
        self.wait_for_graph_ready()
        self.spin_with_pose_until(
            lambda: any(
                transform.header.frame_id == "MAV1/odom"
                and transform.child_frame_id == "MAV1/base_link"
                for message in self.tf_messages
                for transform in message.transforms
            )
        )
        self.assertFalse(
            any(
                transform.header.frame_id == "odom"
                or transform.child_frame_id == "base_link"
                for message in self.tf_messages
                for transform in message.transforms
            )
        )
        self.assertTrue(
            any(
                transform.header.frame_id == "map"
                and transform.child_frame_id == "MAV1/odom"
                for message in self.static_tf_messages
                for transform in message.transforms
            )
        )
        publisher_topics = {
            topic_name
            for topic_name, _ in self.node.get_publisher_names_and_types_by_node(
                "plan_goal_bridge", "/MAV1"
            )
        }
        self.assertIn("/MAV1/plan", publisher_topics)
        forbidden_topics = {
            "/cmd_vel",
            "/MAV1/cmd_vel",
            "/MAV1/fmu/in/offboard_control_mode",
            "/MAV1/fmu/in/trajectory_setpoint",
            "/MAV1/fmu/in/vehicle_command",
        }
        self.assertTrue(forbidden_topics.isdisjoint(publisher_topics))
        for topic in forbidden_topics:
            self.assertEqual(self.node.get_publishers_info_by_topic(topic), [])

        self.goal_publisher.publish(self.goal("map", 1.0, -6.5))
        self.spin_with_pose_until(lambda: self.plans and len(self.plans[-1].poses) > 1)
        self.assertEqual(self.plans[-1].header.frame_id, "map")
        self.assertTrue(self.maps)
        occupancy_map = self.maps[-1]
        self.assertEqual(occupancy_map.header.frame_id, "map")
        for pose in self.plans[-1].poses:
            column = math.floor(
                (pose.pose.position.x - occupancy_map.info.origin.position.x)
                / occupancy_map.info.resolution
            )
            row = math.floor(
                (pose.pose.position.y - occupancy_map.info.origin.position.y)
                / occupancy_map.info.resolution
            )
            self.assertGreaterEqual(column, 0)
            self.assertLess(column, occupancy_map.info.width)
            self.assertGreaterEqual(row, 0)
            self.assertLess(row, occupancy_map.info.height)
            self.assertEqual(
                occupancy_map.data[row * occupancy_map.info.width + column], 0
            )

        late_observer = Node("late_plan_observer")
        late_plans = []
        late_observer.create_subscription(
            Path, "/MAV1/plan", late_plans.append, self.persistent_plan_qos()
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not late_plans:
            rclpy.spin_once(late_observer, timeout_sec=0.05)
        late_observer.destroy_node()
        self.assertTrue(late_plans and len(late_plans[-1].poses) > 1)

        self.goal_publisher.publish(self.goal("MAV1/odom", 1.0, -6.5))
        self.spin_with_pose_until(lambda: self.plans and not self.plans[-1].poses)


if __name__ == "__main__":
    unittest.main()
