#!/usr/bin/env python3
"""Coordinate one Leader Nav2 mission and two Fixed V-Slot Followers."""

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLandDetected, VehicleStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from fixed_slot_follower_controller import body_offset_to_map_target, odometry_yaw


class FormationMissionManager(Node):
    """Own the observable lifecycle of a three-vehicle Formation Mission."""

    def __init__(self) -> None:
        super().__init__("formation_mission_manager")
        self._leader = self.declare_parameter("leader_namespace", "MAV1").value
        self._followers = (
            self.declare_parameter("mav2_namespace", "MAV2").value,
            self.declare_parameter("mav3_namespace", "MAV3").value,
        )
        self._vehicles = (self._leader, *self._followers)
        self._flight_level = float(self.declare_parameter("flight_level", 3.0).value)
        self._height_tolerance = float(
            self.declare_parameter("height_tolerance", 0.15).value
        )
        self._slot_tolerance = float(
            self.declare_parameter("slot_tolerance", 0.25).value
        )
        self._slot_convergence_seconds = float(
            self.declare_parameter("slot_convergence_seconds", 3.0).value
        )
        self._abort_slot_error = float(
            self.declare_parameter("abort_slot_error", 2.0).value
        )
        self._minimum_separation = float(
            self.declare_parameter("minimum_separation", 0.7).value
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
        self._slots = {
            self._followers[0]: (
                float(self.declare_parameter("mav2_slot_forward", -0.8).value),
                float(self.declare_parameter("mav2_slot_left", 0.8).value),
            ),
            self._followers[1]: (
                float(self.declare_parameter("mav3_slot_forward", -0.8).value),
                float(self.declare_parameter("mav3_slot_left", -0.8).value),
            ),
        }
        self._phase = "idle"
        self._phase_started_ns = self.get_clock().now().nanoseconds
        self._slot_converged_since_ns: int | None = None
        self._goal: PoseStamped | None = None
        self._navigation_goal = None
        self._leader_goal_succeeded = False
        self._odometry: dict[str, Odometry] = {}
        self._odom_received_ns: dict[str, int] = {}
        self._landed: dict[str, bool] = {vehicle: False for vehicle in self._vehicles}

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
        self._phase_publishers = {
            vehicle: self.create_publisher(String, f"/{vehicle}/mission_phase", phase_qos)
            for vehicle in self._vehicles
        }
        self._status_publishers = {
            vehicle: self.create_publisher(String, f"/{vehicle}/mission_status", phase_qos)
            for vehicle in self._vehicles
        }
        self.create_subscription(
            PoseStamped, f"/{self._leader}/mission_goal", self._on_mission_goal, reliable_qos
        )
        self.create_subscription(
            Empty, f"/{self._leader}/mission_cancel", self._on_cancel, reliable_qos
        )
        for vehicle in self._vehicles:
            self.create_subscription(
                Odometry,
                f"/{vehicle}/odom",
                lambda message, name=vehicle: self._on_odom(name, message),
                reliable_qos,
            )
            self.create_subscription(
                VehicleLandDetected,
                f"/{vehicle}/fmu/out/vehicle_land_detected",
                lambda message, name=vehicle: self._on_land_detected(name, message),
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.create_subscription(
                VehicleStatus,
                f"/{vehicle}/fmu/out/vehicle_status_v1",
                lambda message, name=vehicle: self._on_vehicle_status(name, message),
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        self._navigator = ActionClient(
            self, NavigateToPose, f"/{self._leader}/navigate_to_pose"
        )
        self.create_timer(0.1, self._tick)
        self._publish_phase("idle", "Ready for a Formation Mission Goal")

    def _on_mission_goal(self, goal: PoseStamped) -> None:
        if self._phase != "idle":
            self.get_logger().warn("Rejecting Formation Mission Goal while active")
            return
        if goal.header.frame_id != "map":
            self.get_logger().warn("Rejecting Formation Mission Goal outside map")
            return
        self._goal = goal
        self._leader_goal_succeeded = False
        self._landed = {vehicle: False for vehicle in self._vehicles}
        self._publish_phase("preflight", "Waiting for fresh MAV1/MAV2/MAV3 odometry")

    def _on_cancel(self, _message: Empty) -> None:
        if self._phase != "idle":
            self._enter_abort_hold("Operator cancelled the Formation Mission")

    def _on_odom(self, vehicle: str, odometry: Odometry) -> None:
        self._odometry[vehicle] = odometry
        self._odom_received_ns[vehicle] = self.get_clock().now().nanoseconds

    def _on_land_detected(self, vehicle: str, message: VehicleLandDetected) -> None:
        self._landed[vehicle] = bool(message.landed)

    def _on_vehicle_status(self, vehicle: str, message: VehicleStatus) -> None:
        if self._phase not in {"idle", "preflight", "warmup", "land_all"} and message.failsafe:
            self._enter_abort_hold(f"{vehicle} PX4 failsafe active")

    def _tick(self) -> None:
        if self._phase == "idle":
            return
        if self._phase == "preflight":
            if self._all_telemetry_fresh():
                self._publish_phase("warmup", "All vehicles have fresh odometry")
            return
        if self._phase not in {"land_all", "abort_hold"} and not self._all_telemetry_fresh():
            self._enter_abort_hold("Vehicle odometry timed out")
            return
        elapsed = (self.get_clock().now().nanoseconds - self._phase_started_ns) / 1e9
        if self._phase == "warmup" and elapsed >= self._warmup_seconds:
            self._publish_phase("takeoff_all", "Offboard warm-up complete; taking off all vehicles")
        elif self._phase == "takeoff_all" and self._all_at_flight_level():
            self._publish_phase("form_up", "Flight Level reached; forming Fixed V-Slots")
        elif self._phase == "form_up":
            if self._followers_converged():
                if self._slot_converged_for_required_duration():
                    self._start_leader_navigation()
            else:
                self._slot_converged_since_ns = None
        elif self._phase == "navigate_leader":
            unsafe_reason = self._unsafe_formation_reason()
            if unsafe_reason is not None:
                self._enter_abort_hold(unsafe_reason)
            elif self._leader_goal_succeeded and self._followers_converged():
                if self._slot_converged_for_required_duration():
                    self._publish_phase("land_all", "Leader goal and follower slots converged")
            else:
                self._slot_converged_since_ns = None
        elif self._phase == "abort_hold" and elapsed >= self._abort_hold_seconds:
            self._publish_phase("land_all", "Abort Hold complete; requesting LandAll")
        elif self._phase == "land_all" and all(self._landed.values()):
            self._goal = None
            self._publish_phase("idle", "MAV1/MAV2/MAV3 landed; mission complete")

    def _all_telemetry_fresh(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        return all(
            vehicle in self._odometry
            and (now_ns - self._odom_received_ns.get(vehicle, 0)) / 1e9
            <= self._telemetry_timeout
            for vehicle in self._vehicles
        )

    def _all_at_flight_level(self) -> bool:
        return all(
            abs(self._odometry[vehicle].pose.pose.position.z - self._flight_level)
            <= self._height_tolerance
            for vehicle in self._vehicles
        )

    def _follower_slot_error(self, follower: str) -> float:
        leader = self._odometry[self._leader]
        follower_odom = self._odometry[follower]
        forward, left = self._slots[follower]
        target_x, target_y = body_offset_to_map_target(
            leader_x=leader.pose.pose.position.x,
            leader_y=leader.pose.pose.position.y,
            leader_yaw=odometry_yaw(leader),
            slot_forward=forward,
            slot_left=left,
        )
        return math.hypot(
            target_x - follower_odom.pose.pose.position.x,
            target_y - follower_odom.pose.pose.position.y,
        )

    def _followers_converged(self) -> bool:
        return all(
            self._follower_slot_error(follower) <= self._slot_tolerance
            for follower in self._followers
        )

    def _slot_converged_for_required_duration(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if self._slot_converged_since_ns is None:
            self._slot_converged_since_ns = now_ns
            return self._slot_convergence_seconds <= 0.0
        return (now_ns - self._slot_converged_since_ns) / 1e9 >= self._slot_convergence_seconds

    def _unsafe_formation_reason(self) -> str | None:
        """Return one externally useful safety-fault reason, if any."""

        for follower in self._followers:
            error = self._follower_slot_error(follower)
            if error > self._abort_slot_error:
                return (
                    f"{follower} slot error {error:.2f} m exceeds "
                    f"{self._abort_slot_error:.2f} m"
                )
        for index, first_name in enumerate(self._vehicles):
            first = self._odometry[first_name].pose.pose.position
            for second_name in self._vehicles[index + 1 :]:
                second = self._odometry[second_name].pose.pose.position
                separation = math.hypot(first.x - second.x, first.y - second.y)
                if separation < self._minimum_separation:
                    return (
                        f"{first_name}/{second_name} separation {separation:.2f} m is below "
                        f"{self._minimum_separation:.2f} m"
                    )
        return None

    def _start_leader_navigation(self) -> None:
        assert self._goal is not None
        if not self._navigator.wait_for_server(timeout_sec=0.0):
            self._enter_abort_hold("MAV1 NavigateToPose action is unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._goal
        self._publish_phase("navigate_leader", "Form-up complete; starting MAV1 Nav2 navigation")
        future = self._navigator.send_goal_async(goal)
        future.add_done_callback(self._on_navigation_goal)

    def _on_navigation_goal(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._enter_abort_hold("MAV1 NavigateToPose rejected the Formation Goal")
            return
        self._navigation_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
        result = future.result()
        self._navigation_goal = None
        if self._phase != "navigate_leader":
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._leader_goal_succeeded = True
            self._slot_converged_since_ns = None
            self._publish_status("navigate_leader", "MAV1 goal succeeded; waiting for follower convergence")
        else:
            self._enter_abort_hold("MAV1 navigation failed")

    def _enter_abort_hold(self, detail: str) -> None:
        if self._phase in {"idle", "abort_hold", "land_all"}:
            return
        self._cancel_navigation()
        self._publish_phase("abort_hold", detail)

    def _cancel_navigation(self) -> None:
        if self._navigation_goal is not None:
            self._navigation_goal.cancel_goal_async()
            self._navigation_goal = None

    def _publish_status(self, phase: str, detail: str) -> None:
        for publisher in self._status_publishers.values():
            publisher.publish(String(data=f"{phase}: {detail}"))

    def _publish_phase(self, phase: str, detail: str) -> None:
        self._phase = phase
        self._phase_started_ns = self.get_clock().now().nanoseconds
        self._slot_converged_since_ns = None
        for publisher in self._phase_publishers.values():
            publisher.publish(String(data=phase))
        self._publish_status(phase, detail)
        self.get_logger().info(f"Formation mission {phase}: {detail}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationMissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
