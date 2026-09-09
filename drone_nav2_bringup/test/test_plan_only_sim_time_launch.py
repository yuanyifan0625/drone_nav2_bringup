"""Launch contract for the Plan-Only simulated-time profile."""

import os
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray


class PlanOnlySimTimeLaunchTest(unittest.TestCase):
    """Simulated time requires a dedicated Gazebo-to-ROS clock bridge."""

    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "104"
        cls.launch_process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "drone_nav2_bringup",
                "plan_only.launch.py",
                "use_sim_time:=true",
                "rviz:=false",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=os.environ.copy(),
        )
        rclpy.init()
        cls.node = Node("plan_only_sim_time_launch_test")
        cls.clock_messages = []
        cls.marker_messages = []
        cls.node.create_subscription(Clock, "/clock", cls.clock_messages.append, 10)
        cls.node.create_subscription(
            MarkerArray,
            "/arena_walls",
            cls.marker_messages.append,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
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

    def wait_for_clock_bridge(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            node_names = self.node.get_node_names_and_namespaces()
            if ("gazebo_clock_bridge", "/") in node_names:
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("use_sim_time profile did not start gazebo_clock_bridge")

    def test_sim_time_profile_starts_the_gazebo_clock_bridge(self):
        self.wait_for_clock_bridge()

    def test_gazebo_world_clock_is_forwarded_to_ros_clock(self):
        self.wait_for_clock_bridge()
        for seconds in range(42, 47):
            subprocess.run(
                [
                    "gz",
                    "topic",
                    "-t",
                    "/world/nav2_arena/clock",
                    "-m",
                    "gz.msgs.Clock",
                    "-p",
                    f"sim {{ sec: {seconds} nsec: 0 }}",
                ],
                check=True,
            )
            time.sleep(0.2)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if any(message.clock.sec >= 42 for message in self.clock_messages):
                return
        self.fail("Gazebo world clock was not forwarded to ROS /clock")

    def test_graph_markers_republish_after_clock_advances(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if any(
                markers.markers and markers.markers[0].header.stamp.sec >= 42
                for markers in self.marker_messages
            ):
                return
        self.fail("graph markers did not republish using the Gazebo-derived clock")

if __name__ == "__main__":
    unittest.main()
