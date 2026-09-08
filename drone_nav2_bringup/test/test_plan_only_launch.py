"""End-to-end Plan-Only contract from PX4 pose to a safe planner path."""

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from px4_msgs.msg import VehicleLocalPosition
import math
import os
from pathlib import Path
import re
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rcl_interfaces.srv import GetParameters
import signal
import subprocess
import time
import unittest


class PlanOnlyLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch_process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "drone_nav2_bringup",
                "plan_only.launch.py",
                "use_sim_time:=false",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        cls.node = Node("plan_only_integration_test")
        px4_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        cls.local_position_publisher = cls.node.create_publisher(
            VehicleLocalPosition,
            "/MAV1/fmu/out/vehicle_local_position_v1",
            px4_qos,
        )
        cls.planner_client = ActionClient(
            cls.node,
            ComputePathToPose,
            "/MAV1/compute_path_to_pose",
        )
        cls.map_state_client = cls.node.create_client(
            GetState, "/map_server/get_state"
        )
        cls.planner_state_client = cls.node.create_client(
            GetState, "/MAV1/planner_server/get_state"
        )
        cls.bridge_parameters_client = cls.node.create_client(
            GetParameters, "/MAV1/px4_odometry_bridge/get_parameters"
        )
        cls.static_tf_parameters_client = cls.node.create_client(
            GetParameters, "/static_map_to_odom/get_parameters"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(cls.launch_process.pid, signal.SIGINT)
            cls.launch_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(cls.launch_process.pid, signal.SIGTERM)
            cls.launch_process.wait(timeout=5.0)

    @staticmethod
    def local_position_at_mav1_spawn():
        message = VehicleLocalPosition()
        message.xy_valid = True
        message.z_valid = True
        message.v_xy_valid = True
        message.v_z_valid = True
        message.x = 0.0
        message.y = 0.0
        message.z = 0.0
        message.vx = 0.0
        message.vy = 0.0
        message.vz = 0.0
        message.heading = 0.0
        return message

    @classmethod
    def spin_and_publish_until(cls, predicate, timeout=15.0):
        message = cls.local_position_at_mav1_spawn()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.local_position_publisher.publish(message)
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("Plan-Only launch did not become ready")

    @classmethod
    def wait_for_active(cls, state_client):
        cls.spin_and_publish_until(
            lambda: state_client.service_is_ready()
        )
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            request_future = state_client.call_async(GetState.Request())
            cls.spin_and_publish_until(request_future.done, timeout=2.0)
            if request_future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                return
        raise AssertionError("Lifecycle node did not become active")

    @classmethod
    def uses_requested_simulation_time(cls, parameters_client):
        cls.spin_and_publish_until(
            lambda: parameters_client.service_is_ready()
        )
        request = GetParameters.Request(names=["use_sim_time"])
        response_future = parameters_client.call_async(request)
        cls.spin_and_publish_until(response_future.done)
        return response_future.result().values[0].bool_value

    @staticmethod
    def map_metadata_and_pixels():
        map_share = Path(get_package_share_directory("drone_nav2_apriltag")) / "maps"
        yaml_text = (map_share / "nav2_arena.yaml").read_text(encoding="utf-8")
        resolution = float(re.search(r"resolution: ([0-9.]+)", yaml_text).group(1))
        origin_match = re.search(r"origin: \[([^]]+)\]", yaml_text)
        origin = tuple(float(value.strip()) for value in origin_match.group(1).split(","))

        with (map_share / "nav2_arena.pgm").open("rb") as pgm_file:
            magic = pgm_file.readline().strip()
            header_lines = []
            while len(header_lines) < 2:
                line = pgm_file.readline()
                if not line.startswith(b"#"):
                    header_lines.append(line)
            dimensions = header_lines[0].split()
            maximum = header_lines[1].strip()
            pixels = pgm_file.read()

        if magic != b"P5" or maximum != b"255":
            raise AssertionError("Static Occupancy Map is not an 8-bit binary PGM")
        width, height = (int(value) for value in dimensions)
        return resolution, origin, width, height, pixels

    def test_map_goal_returns_a_non_empty_path_through_known_free_space(self):
        self.assertFalse(
            self.uses_requested_simulation_time(self.bridge_parameters_client)
        )
        self.assertFalse(
            self.uses_requested_simulation_time(self.static_tf_parameters_client)
        )
        self.wait_for_active(self.map_state_client)
        self.wait_for_active(self.planner_state_client)
        self.spin_and_publish_until(
            lambda: self.planner_client.wait_for_server(timeout_sec=0.0)
        )

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = 1.0
        goal.goal.pose.position.y = -6.5
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False

        goal_future = self.planner_client.send_goal_async(goal)
        self.spin_and_publish_until(goal_future.done)
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        result_future = goal_handle.get_result_async()
        self.spin_and_publish_until(result_future.done)
        result = result_future.result()
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        path = result.result.path
        self.assertEqual(path.header.frame_id, "map")
        self.assertGreater(len(path.poses), 1)

        resolution, origin, width, height, pixels = self.map_metadata_and_pixels()
        for pose_stamped in path.poses:
            position = pose_stamped.pose.position
            column = math.floor((position.x - origin[0]) / resolution)
            row_from_bottom = math.floor((position.y - origin[1]) / resolution)
            row = height - 1 - row_from_bottom
            self.assertGreaterEqual(column, 0)
            self.assertLess(column, width)
            self.assertGreaterEqual(row, 0)
            self.assertLess(row, height)
            self.assertEqual(pixels[row * width + column], 254)


if __name__ == "__main__":
    unittest.main()
