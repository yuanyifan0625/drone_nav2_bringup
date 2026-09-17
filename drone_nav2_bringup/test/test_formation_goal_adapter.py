"""Public Formation Goal Adapter contracts without PX4 or Nav2."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class FormationGoalAdapterTest(unittest.TestCase):
    """Formation inputs normalize to map-frame Mission Goals only."""

    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "110"
        cls.graph_file = tempfile.NamedTemporaryFile(
            mode="w", suffix=".geojson", delete=False, encoding="utf-8"
        )
        json.dump(
            {
                "type": "FeatureCollection",
                "features": [
                    {
                        "type": "Feature",
                        "properties": {"id": 4, "frame": "map"},
                        "geometry": {"type": "Point", "coordinates": [1.0, -6.5]},
                    },
                    {
                        "type": "Feature",
                        "properties": {"id": 10, "frame": "map", "operations": {"scan": {}}},
                        "geometry": {"type": "Point", "coordinates": [27.0, 16.0]},
                    },
                    {
                        "type": "Feature",
                        "properties": {"id": 77, "frame": "odom"},
                        "geometry": {"type": "Point", "coordinates": [2.0, 3.0]},
                    },
                    {
                        "type": "Feature",
                        "properties": {"id": 88, "frame": "map"},
                        "geometry": None,
                    },
                ],
            },
            cls.graph_file,
        )
        cls.graph_file.close()
        cls.process = subprocess.Popen(
            [
                "ros2",
                "run",
                "drone_nav2_bringup",
                "formation_goal_adapter.py",
                "--ros-args",
                "-p",
                "vehicle_namespace:=MAV1",
                "-p",
                f"graph_file:={cls.graph_file.name}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env=os.environ.copy(),
        )
        rclpy.init()
        cls.node = Node("formation_goal_adapter_test")
        cls.pose_publisher = cls.node.create_publisher(
            PoseStamped, "/MAV1/formation_goal_pose", 10
        )
        cls.node_id_publisher = cls.node.create_publisher(
            Int32, "/MAV1/formation_goal_node_id", 10
        )
        cls.mission_goals = []
        cls.vehicle_commands = []
        cls.node.create_subscription(
            PoseStamped, "/MAV1/mission_goal", cls.mission_goals.append, 10
        )
        cls.node.create_subscription(
            VehicleCommand,
            "/MAV1/fmu/in/vehicle_command",
            cls.vehicle_commands.append,
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.killpg(cls.process.pid, signal.SIGINT)
        cls.process.wait(timeout=5.0)
        os.unlink(cls.graph_file.name)

    @classmethod
    def publish_until(cls, publisher, message, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("Formation Goal Adapter did not satisfy its public contract")

    def test_map_pose_is_normalized_but_non_map_pose_is_rejected(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 3.0
        goal.pose.position.y = -2.0
        goal.pose.orientation.w = 1.0
        self.publish_until(
            self.pose_publisher,
            goal,
            lambda: any(message.pose.position.x == 3.0 for message in self.mission_goals),
        )
        goal.header.frame_id = "odom"
        self.pose_publisher.publish(goal)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertFalse(
            any(message.header.frame_id == "odom" for message in self.mission_goals)
        )

    def test_node_id_resolves_map_points_and_rejects_invalid_or_non_map_nodes(self):
        self.publish_until(
            self.node_id_publisher,
            Int32(data=4),
            lambda: any(
                message.header.frame_id == "map"
                and message.pose.position.x == 1.0
                and message.pose.position.y == -6.5
                for message in self.mission_goals
            ),
        )
        self.publish_until(
            self.node_id_publisher,
            Int32(data=10),
            lambda: any(
                message.pose.position.x == 27.0 and message.pose.position.y == 16.0
                for message in self.mission_goals
            ),
        )
        self.assertEqual([], self.vehicle_commands)
        count_before_rejection = len(self.mission_goals)
        for node_id in (77, 88, 999):
            self.node_id_publisher.publish(Int32(data=node_id))
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(count_before_rejection, len(self.mission_goals))

    def test_mission_rviz_config_keeps_plan_and_formation_goal_interfaces_separate(self):
        config_path = (
            Path(get_package_share_directory("drone_nav2_bringup"))
            / "rviz"
            / "mav1_offboard_mission.rviz"
        )
        config = config_path.read_text(encoding="utf-8")
        self.assertIn("/MAV1/formation_goal_pose", config)
        self.assertNotIn("/MAV1/goal_pose", config)
        self.assertIn("Fixed Frame: map", config)


if __name__ == "__main__":
    unittest.main()
