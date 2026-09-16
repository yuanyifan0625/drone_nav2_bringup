"""Public launch graph contract for a Leader-owned Formation Mission."""

import os
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class FormationMissionLaunchTest(unittest.TestCase):
    """Only MAV1 has Nav2; each Follower owns its migration controller seams."""

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
        cls.tf_broadcaster = StaticTransformBroadcaster(cls.node)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "MAV2/base_link"
        transform.transform.rotation.w = 1.0
        cls.tf_broadcaster.sendTransform(transform)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.killpg(cls.process.pid, signal.SIGINT)
        cls.process.wait(timeout=8.0)

    def test_leader_owns_nav2_and_followers_own_controller_seams(self):
        deadline = time.monotonic() + 15.0
        expected = {
            ("planner_server", "/MAV1"),
            ("controller_server", "/MAV1"),
            ("bt_navigator", "/MAV1"),
            ("fixed_slot_follower_controller", "/MAV2"),
            ("fixed_slot_follower_controller", "/MAV3"),
            ("follower_mppi_controller_server", "/MAV2"),
            ("follower_local_lifecycle_manager", "/MAV2"),
            ("follower_path_adapter", "/MAV2"),
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
            expected_topics = {f"/{follower}/formation_target_pose"}
            if follower == "MAV3":
                expected_topics.add(f"/{follower}/cmd_vel")
            topic_deadline = time.monotonic() + 5.0
            while publisher_topics != expected_topics and time.monotonic() < topic_deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                publisher_topics = {
                    topic
                    for topic, _ in self.node.get_publisher_names_and_types_by_node(
                        "fixed_slot_follower_controller", f"/{follower}"
                    )
                    if topic not in {"/parameter_events", "/rosout"}
                }
            self.assertEqual(expected_topics, publisher_topics)
        mav2_cmd_vel_publishers = self.node.get_publishers_info_by_topic("/MAV2/cmd_vel")
        self.assertEqual(1, len(mav2_cmd_vel_publishers))
        publisher = mav2_cmd_vel_publishers[0]
        self.assertEqual("follower_mppi_controller_server", publisher.node_name)
        self.assertEqual("/MAV2", publisher.node_namespace)


    def test_mav2_mppi_lifecycle_is_active(self):
        """The only MAV2 local-control lifecycle node reaches active."""

        client = self.node.create_client(
            GetState, "/MAV2/follower_mppi_controller_server/get_state"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            if (
                future.result() is not None
                and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            ):
                return
        raise AssertionError("MAV2 MPPI controller did not become active")


if __name__ == "__main__":
    unittest.main()
