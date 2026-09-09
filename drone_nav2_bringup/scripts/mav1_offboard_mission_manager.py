#!/usr/bin/env python3
"""Authorize one MAV1 fixed-height Nav2 mission in Gazebo SITL."""

from __future__ import annotations

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLandDetected, VehicleStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


class Mav1OffboardMissionManager(Node):
    """Run Preflight, Takeoff, Navigate, Abort Hold and Land for MAV1."""

    def __init__(self) -> None:
        super().__init__("mav1_offboard_mission_manager")
        namespace = self.declare_parameter("vehicle_namespace", "MAV1").value
        self._flight_level = float(self.declare_parameter("flight_level", 3.0).value)
        self._height_tolerance = float(
            self.declare_parameter("height_tolerance", 0.15).value
        )
        self._warmup_seconds = float(
            self.declare_parameter("warmup_seconds", 1.0).value
        )
        self._abort_hold_seconds = float(
            self.declare_parameter("abort_hold_seconds", 2.0).value
        )
        self._telemetry_timeout = float(
            self.declare_parameter("telemetry_timeout", 0.5).value
        )
        self._phase = "idle"
        self._phase_started_ns = self.get_clock().now().nanoseconds
        self._goal: PoseStamped | None = None
        self._odom: Odometry | None = None
        self._last_odom_ns: int | None = None
        self._navigation_goal = None
        self._landed = False

        phase_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._phase_publisher = self.create_publisher(
            String, f"/{namespace}/mission_phase", phase_qos
        )
        self._status_publisher = self.create_publisher(
            String, f"/{namespace}/mission_status", phase_qos
        )
        self.create_subscription(
            PoseStamped,
            f"/{namespace}/mission_goal",
            self._on_mission_goal,
            reliable_qos,
        )
        self.create_subscription(
            Empty,
            f"/{namespace}/mission_cancel",
            self._on_cancel,
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            f"/{namespace}/odom",
            self._on_odom,
            reliable_qos,
        )
        self.create_subscription(
            VehicleLandDetected,
            f"/{namespace}/fmu/out/vehicle_land_detected",
            self._on_land_detected,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            VehicleStatus,
            f"/{namespace}/fmu/out/vehicle_status_v1",
            self._on_vehicle_status,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._navigator = ActionClient(
            self, NavigateToPose, f"/{namespace}/navigate_to_pose"
        )
        self.create_timer(0.1, self._tick)
        self._publish_phase("idle", "Ready for a MAV1 Mission Goal")

    def _on_mission_goal(self, goal: PoseStamped) -> None:
        if self._phase != "idle":
            self.get_logger().warn("Rejecting Mission Goal while a mission is active")
            return
        if goal.header.frame_id != "map":
            self.get_logger().warn("Rejecting Mission Goal outside the map frame")
            return
        self._goal = goal
        self._publish_phase("preflight", "Waiting for fresh MAV1 odometry")

    def _on_cancel(self, _message: Empty) -> None:
        if self._phase != "idle":
            self._enter_abort_hold("Operator cancelled the MAV1 mission")

    def _on_odom(self, odometry: Odometry) -> None:
        self._odom = odometry
        self._last_odom_ns = self.get_clock().now().nanoseconds

    def _on_land_detected(self, message: VehicleLandDetected) -> None:
        self._landed = bool(message.landed)

    def _on_vehicle_status(self, message: VehicleStatus) -> None:
        if self._phase in {"takeoff", "navigate", "abort_hold"} and message.failsafe:
            self._cancel_navigation()
            self._publish_phase("idle", "PX4 failsafe active; bridge relinquished authority")

    def _tick(self) -> None:
        if self._phase == "idle":
            return
        if self._phase == "preflight":
            if not self._telemetry_is_stale():
                self._publish_phase("warmup", "Preflight complete; warming Offboard heartbeat")
            return
        if self._phase != "land" and self._telemetry_is_stale():
            self._enter_abort_hold("PX4 odometry timed out")
            return
        elapsed = (self.get_clock().now().nanoseconds - self._phase_started_ns) / 1e9
        if self._phase == "warmup" and elapsed >= self._warmup_seconds:
            self._publish_phase("takeoff", "Offboard warm-up complete; taking off")
        elif self._phase == "takeoff" and self._at_flight_level():
            self._start_navigation()
        elif self._phase == "abort_hold" and elapsed >= self._abort_hold_seconds:
            self._publish_phase("land", "Abort Hold complete; requesting PX4 land")
        elif self._phase == "land" and self._landed:
            self._publish_phase("idle", "MAV1 landed; mission complete")

    def _telemetry_is_stale(self) -> bool:
        if self._last_odom_ns is None:
            return True
        age = (self.get_clock().now().nanoseconds - self._last_odom_ns) / 1e9
        return age > self._telemetry_timeout

    def _at_flight_level(self) -> bool:
        return self._odom is not None and abs(
            self._odom.pose.pose.position.z - self._flight_level
        ) <= self._height_tolerance

    def _start_navigation(self) -> None:
        assert self._goal is not None
        if not self._navigator.wait_for_server(timeout_sec=0.0):
            self._enter_abort_hold("MAV1 NavigateToPose action is unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._goal
        self._publish_phase("navigate", "Flight Level reached; starting MAV1 Nav2 navigation")
        future = self._navigator.send_goal_async(goal)
        future.add_done_callback(self._on_navigation_goal)

    def _on_navigation_goal(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._enter_abort_hold("MAV1 NavigateToPose rejected the Mission Goal")
            return
        self._navigation_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
        result = future.result()
        self._navigation_goal = None
        if self._phase != "navigate":
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_phase("land", "MAV1 Navigation succeeded; requesting PX4 land")
        else:
            self._enter_abort_hold("MAV1 navigation failed")

    def _enter_abort_hold(self, reason: str) -> None:
        if self._phase in {"abort_hold", "land", "idle"}:
            return
        self._cancel_navigation()
        self._publish_phase("abort_hold", reason)

    def _cancel_navigation(self) -> None:
        if self._navigation_goal is not None:
            self._navigation_goal.cancel_goal_async()
            self._navigation_goal = None

    def _publish_phase(self, phase: str, detail: str) -> None:
        self._phase = phase
        self._phase_started_ns = self.get_clock().now().nanoseconds
        self._phase_publisher.publish(String(data=phase))
        self._status_publisher.publish(String(data=f"{phase}: {detail}"))
        self.get_logger().info(f"MAV1 mission {phase}: {detail}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Mav1OffboardMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
