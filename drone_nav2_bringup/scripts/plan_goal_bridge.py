#!/usr/bin/env python3
"""Bridge MAV1 RViz goals to Nav2 planning and a persistent display path."""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PlanGoalBridge(Node):
    """Accept map-frame goals and publish only the newest successful MAV1 plan."""

    def __init__(self):
        super().__init__("plan_goal_bridge")
        vehicle_prefix = self.declare_parameter("vehicle_prefix", "MAV1").value
        self._map_frame = self.declare_parameter("map_frame", "map").value
        self._base_frame = self.declare_parameter(
            "base_frame", f"{vehicle_prefix}/base_link"
        ).value
        goal_topic = self.declare_parameter(
            "goal_topic", f"/{vehicle_prefix}/goal_pose"
        ).value
        plan_topic = self.declare_parameter("plan_topic", f"/{vehicle_prefix}/plan").value
        planner_action = self.declare_parameter(
            "planner_action", f"/{vehicle_prefix}/compute_path_to_pose"
        ).value

        plan_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._plan_publisher = self.create_publisher(Path, plan_topic, plan_qos)
        self._goal_subscription = self.create_subscription(
            PoseStamped, goal_topic, self._on_goal, 10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._planner_client = ActionClient(self, ComputePathToPose, planner_action)
        self._request_number = 0
        self._active_goal_handle = None
        self.get_logger().info(
            f"Bridging {goal_topic} to {planner_action} and publishing {plan_topic}"
        )

    def _clear_plan(self):
        empty_path = Path()
        empty_path.header.frame_id = self._map_frame
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self._plan_publisher.publish(empty_path)

    def _on_goal(self, goal):
        self._request_number += 1
        request_number = self._request_number
        self._clear_plan()

        if goal.header.frame_id != self._map_frame:
            self.get_logger().error(
                f"Rejected goal frame '{goal.header.frame_id}'; expected '{self._map_frame}'"
            )
            return
        if not self._planner_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().error("Planner action server is unavailable")
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time(), timeout=Duration(seconds=0.2)
            )
        except TransformException as error:
            self.get_logger().error(f"Cannot get current MAV1 TF pose: {error}")
            return

        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()

        request = ComputePathToPose.Goal()
        request.goal = goal
        request.use_start = True
        request.start.header = transform.header
        request.start.header.frame_id = self._map_frame
        request.start.pose.position.x = transform.transform.translation.x
        request.start.pose.position.y = transform.transform.translation.y
        request.start.pose.position.z = transform.transform.translation.z
        request.start.pose.orientation = transform.transform.rotation
        future = self._planner_client.send_goal_async(request)
        future.add_done_callback(
            lambda response: self._on_goal_response(request_number, response)
        )

    def _on_goal_response(self, request_number, response):
        goal_handle = response.result()
        if request_number != self._request_number:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().error("Planner rejected goal")
            self._clear_plan()
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_plan_result(request_number, result)
        )

    def _on_plan_result(self, request_number, result_future):
        if request_number != self._request_number:
            return
        self._active_goal_handle = None
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
            self.get_logger().error(f"Planner failed with action status {result.status}")
            self._clear_plan()
            return
        self._plan_publisher.publish(result.result.path)


def main(args=None):
    rclpy.init(args=args)
    node = PlanGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
