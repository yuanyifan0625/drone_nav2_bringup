"""Public launch graph contract for a Leader-owned Formation Mission."""

import os
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.node import Node


class FormationMissionLaunchTest(unittest.TestCase):
    """Only MAV1 has Nav2; each Follower has exactly its own controller seam."""

    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "113"
        cls.process = subprocess.Popen(
            [
                "ros2", "launch", "drone_nav2_bringup", "formation_mission.launch.py",
                "use_sim_time:=false", "rviz:=false",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env=os.environ.copy(),
        )
        rclpy.init()
        cls.node = Node("formation_mission_launch_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.killpg(cls.process.pid, signal.SIGINT)
        cls.process.wait(timeout=8.0)

    def test_leader_owns_nav2_and_followers_own_only_their_cmd_vel(self):
        deadline = time.monotonic() + 15.0
        expected = {
            ("planner_server", "/MAV1"),
            ("controller_server", "/MAV1"),
            ("bt_navigator", "/MAV1"),
            ("fixed_slot_follower_controller", "/MAV2"),
            ("fixed_slot_follower_controller", "/MAV3"),
            ("formation_mission_manager", "/"),
        }
        nodes = set()
        while time.monotonic() < deadline:
            nodes = set(self.node.get_node_names_and_namespaces())
            if expected.issubset(nodes):
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(expected.issubset(nodes))
        for follower in ("MAV2", "MAV3"):
            self.assertNotIn(("planner_server", f"/{follower}"), nodes)
            self.assertNotIn(("controller_server", f"/{follower}"), nodes)
            self.assertNotIn(("bt_navigator", f"/{follower}"), nodes)
            publisher_topics = {
                topic
                for topic, _ in self.node.get_publisher_names_and_types_by_node(
                    "fixed_slot_follower_controller", f"/{follower}"
                )
                if topic not in {"/parameter_events", "/rosout"}
            }
            self.assertEqual({f"/{follower}/cmd_vel"}, publisher_topics)


if __name__ == "__main__":
    unittest.main()
