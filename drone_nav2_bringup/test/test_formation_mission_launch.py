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
        mav3_transform = TransformStamped()
        mav3_transform.header.frame_id = "map"
        mav3_transform.child_frame_id = "MAV3/base_link"
        mav3_transform.transform.rotation.w = 1.0
        cls.tf_broadcaster.sendTransform([transform, mav3_transform])

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
            ("cooperative_obstacle_publisher", "/MAV2"),
            ("follower_mppi_controller_server", "/MAV3"),
            ("follower_local_lifecycle_manager", "/MAV3"),
            ("follower_path_adapter", "/MAV3"),
            ("cooperative_obstacle_publisher", "/MAV3"),
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
        for follower in ("MAV2", "MAV3"):
            publishers = self.node.get_publishers_info_by_topic(f"/{follower}/cmd_vel")
            self.assertEqual(1, len(publishers))
            publisher = publishers[0]
            self.assertEqual("follower_mppi_controller_server", publisher.node_name)
            self.assertEqual(f"/{follower}", publisher.node_namespace)
            consumer_deadline = time.monotonic() + 5.0
            consumers = []
            while time.monotonic() < consumer_deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                consumers = self.node.get_subscriptions_info_by_topic(
                    f"/{follower}/cmd_vel"
                )
                if len(consumers) == 1 and consumers[0].node_name != "_NODE_NAME_UNKNOWN_":
                    break
            self.assertEqual(1, len(consumers))
            self.assertEqual("cmd_vel_to_px4_offboard_bridge", consumers[0].node_name)
            self.assertEqual(f"/{follower}", consumers[0].node_namespace)


    def test_follower_mppi_lifecycles_are_active(self):
        """Each follower local-control owner activates its only controller."""

        for follower in ("MAV2", "MAV3"):
            client = self.node.create_client(
                GetState, f"/{follower}/follower_mppi_controller_server/get_state"
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
                    break
            else:
                raise AssertionError(f"{follower} MPPI controller did not become active")


if __name__ == "__main__":
    unittest.main()
