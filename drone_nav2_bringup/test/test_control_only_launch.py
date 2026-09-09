"""Public command-only contract for MAV1 native Nav2 control."""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import os
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import signal
import subprocess
import time
import unittest


class ControlOnlyLaunchTest(unittest.TestCase):
    """NavigateToPose drives MAV1 cmd_vel without any PX4 input authority."""

    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "106"
        cls.launch_process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "drone_nav2_bringup",
                "control_only.launch.py",
                "use_sim_time:=false",
                "rviz:=false",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env=os.environ.copy(),
        )
        rclpy.init()
        cls.node = Node("control_only_launch_test")
        px4_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        cls.position_publisher = cls.node.create_publisher(
            VehicleLocalPosition,
            "/MAV1/fmu/out/vehicle_local_position_v1",
            px4_qos,
        )
        cls.navigate_client = ActionClient(
            cls.node, NavigateToPose, "/MAV1/navigate_to_pose"
        )
        cls.navigator_state_client = cls.node.create_client(
            GetState, "/MAV1/bt_navigator/get_state"
        )
        cls.planner_state_client = cls.node.create_client(
            GetState, "/MAV1/planner_server/get_state"
        )
        cls.commands = []
        cls.node.create_subscription(Twist, "/MAV1/cmd_vel", cls.commands.append, 10)

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

    @staticmethod
    def local_position_at_spawn():
        position = VehicleLocalPosition()
        position.xy_valid = True
        position.z_valid = True
        position.v_xy_valid = True
        position.v_z_valid = True
        position.heading = 0.0
        return position

    @classmethod
    def spin_with_position_until(cls, predicate, timeout=30.0):
        position = cls.local_position_at_spawn()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.position_publisher.publish(position)
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("Control-only launch did not satisfy its public contract")

    @classmethod
    def wait_for_active(cls, state_client, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state_future = state_client.call_async(GetState.Request())
            try:
                cls.spin_with_position_until(state_future.done, timeout=1.0)
            except AssertionError:
                continue
            if state_future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                return
        raise AssertionError("A required Nav2 lifecycle node did not become active")

    @staticmethod
    def is_zero(command):
        return (
            abs(command.linear.x) < 1e-6
            and abs(command.linear.y) < 1e-6
            and abs(command.angular.z) < 1e-6
        )

    def test_navigation_commands_mav1_without_px4_input(self):
        self.spin_with_position_until(
            lambda: self.navigate_client.wait_for_server(timeout_sec=0.1)
        )
        self.assertTrue(self.navigator_state_client.wait_for_service(timeout_sec=5.0))
        self.assertTrue(self.planner_state_client.wait_for_service(timeout_sec=5.0))
        self.wait_for_active(self.navigator_state_client)
        self.wait_for_active(self.planner_state_client)
        forbidden_topics = {
            "/MAV1/fmu/in/offboard_control_mode",
            "/MAV1/fmu/in/trajectory_setpoint",
            "/MAV1/fmu/in/vehicle_command",
        }
        for topic in forbidden_topics:
            self.assertEqual(self.node.get_publishers_info_by_topic(topic), [])

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.position.y = -6.5
        goal.pose.pose.orientation.w = 1.0
        goal_future = self.navigate_client.send_goal_async(goal)
        self.spin_with_position_until(goal_future.done)
        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        self.spin_with_position_until(
            lambda: any(not self.is_zero(command) for command in self.commands)
        )

        command_count_before_cancel = len(self.commands)
        cancel_future = goal_handle.cancel_goal_async()
        self.spin_with_position_until(cancel_future.done)
        self.assertTrue(cancel_future.result().goals_canceling)

        result_future = goal_handle.get_result_async()
        self.spin_with_position_until(result_future.done)
        self.assertEqual(result_future.result().status, GoalStatus.STATUS_CANCELED)
        self.spin_with_position_until(
            lambda: any(
                self.is_zero(command)
                for command in self.commands[command_count_before_cancel:]
            )
        )


if __name__ == "__main__":
    unittest.main()
