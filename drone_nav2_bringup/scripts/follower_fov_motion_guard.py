#!/usr/bin/env python3
"""Gate follower MPPI motion until its front depth camera covers travel."""

import math
import struct
from typing import NamedTuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


class MotionCommand(NamedTuple):
    """One planar FLU command produced by MPPI."""

    forward: float
    left: float
    yaw_rate: float
    up: float = 0.0


class MotionDecision(NamedTuple):
    """One safe final command and whether the guard intervened."""

    forward: float
    left: float
    yaw_rate: float
    up: float
    active: bool


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def camera_horizontal_half_fov(camera_info: CameraInfo) -> float | None:
    """Return horizontal half FOV from the standard CameraInfo intrinsics."""

    focal_length = camera_info.k[0]
    if camera_info.width <= 0 or focal_length <= 0.0:
        return None
    return math.atan(camera_info.width / (2.0 * focal_length))


def odometry_yaw(odometry: Odometry) -> float:
    """Return the follower's map-frame yaw."""

    orientation = odometry.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


def image_has_valid_depth(
    image: Image, heading: float, half_fov: float | None
) -> bool:
    """Check the depth-image sector corresponding to the requested motion."""

    encodings = {"32FC1": ("f", 4), "16UC1": ("H", 2)}
    encoding = encodings.get(image.encoding)
    if (
        encoding is None
        or image.width == 0
        or image.height == 0
        or half_fov is None
        or abs(heading) > half_fov
    ):
        return False
    value_format, value_size = encoding
    byte_order = ">" if image.is_bigendian else "<"
    rows = (image.height // 4, image.height // 2, (3 * image.height) // 4)
    column = round((heading / (2.0 * half_fov) + 0.5) * (image.width - 1))
    column = max(0, min(image.width - 1, column))
    for row in rows:
        offset = row * image.step + column * value_size
        if offset + value_size > len(image.data):
            continue
        value = struct.unpack_from(byte_order + value_format, image.data, offset)[0]
        if math.isfinite(value) and value > 0.0:
            return True
    return False


def depth_is_fresh(received: float | None, now: float, timeout: float) -> bool:
    """Return whether a depth callback arrived within its allowed age."""

    return received is not None and now - received <= timeout


class FovMotionPolicy:
    """Keep translation inside observed camera coverage without changing MPPI."""

    def __init__(
        self,
        fov_margin: float = math.radians(10.0),
        yaw_gain: float = 1.5,
        max_yaw_rate: float = 0.6,
        release_seconds: float = 0.4,
        translation_epsilon: float = 0.02,
    ) -> None:
        self._fov_margin = fov_margin
        self._yaw_gain = yaw_gain
        self._max_yaw_rate = max_yaw_rate
        self._release_seconds = release_seconds
        self._translation_epsilon = translation_epsilon
        self._travel_heading: float | None = None
        self._release_started: float | None = None

    def evaluate(
        self,
        command: MotionCommand,
        yaw: float,
        half_fov: float | None,
        depth_ready: bool,
        now: float,
    ) -> MotionDecision:
        """Return a final command after FOV coverage and depth freshness gating."""

        if math.hypot(command.forward, command.left) < self._translation_epsilon:
            self._travel_heading = None
            self._release_started = None
            return MotionDecision(*command, active=False)

        allowed_half_fov = None if half_fov is None else max(0.0, half_fov - self._fov_margin)
        relative_heading = math.atan2(command.left, command.forward)
        if self._travel_heading is None:
            if depth_ready and allowed_half_fov is not None and abs(relative_heading) <= allowed_half_fov:
                return MotionDecision(*command, active=False)
            self._travel_heading = normalize_angle(yaw + relative_heading)

        yaw_error = normalize_angle(self._travel_heading - yaw)
        command_matches_heading = abs(
            normalize_angle(yaw + relative_heading - self._travel_heading)
        ) <= self._fov_margin
        covered = (
            depth_ready
            and allowed_half_fov is not None
            and abs(yaw_error) <= allowed_half_fov
            and command_matches_heading
        )
        if not covered:
            self._release_started = None
            return MotionDecision(
                0.0,
                0.0,
                max(-self._max_yaw_rate, min(self._max_yaw_rate, self._yaw_gain * yaw_error)),
                0.0,
                True,
            )

        if self._release_started is None:
            self._release_started = now
        if self._release_seconds <= 0.0:
            scale = 1.0
        else:
            scale = min(1.0, (now - self._release_started) / self._release_seconds)
        if scale + 1e-9 >= 1.0:
            self._travel_heading = None
            self._release_started = None
            return MotionDecision(*command, active=False)
        return MotionDecision(
            command.forward * scale,
            command.left * scale,
            max(-self._max_yaw_rate, min(self._max_yaw_rate, self._yaw_gain * yaw_error)),
            command.up,
            True,
        )


class FovMotionGuard(Node):
    """Publish the only final follower cmd_vel after front-camera gating."""

    def __init__(self) -> None:
        super().__init__("fov_motion_guard")
        self._command_timeout = float(self.declare_parameter("command_timeout", 0.3).value)
        self._depth_timeout = float(self.declare_parameter("depth_timeout", 0.3).value)
        self._policy = FovMotionPolicy(
            fov_margin=math.radians(
                float(self.declare_parameter("fov_margin_degrees", 10.0).value)
            ),
            yaw_gain=float(self.declare_parameter("yaw_gain", 1.5).value),
            max_yaw_rate=float(self.declare_parameter("max_yaw_rate", 0.6).value),
            release_seconds=float(self.declare_parameter("release_seconds", 0.4).value),
        )
        self._command: MotionCommand | None = None
        self._command_received: float | None = None
        self._odom: Odometry | None = None
        self._depth_received: float | None = None
        self._depth: Image | None = None
        self._half_fov: float | None = None

        self._command_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self._active_publisher = self.create_publisher(Bool, "fov_motion_guard/active", 10)
        self.create_subscription(Twist, "mppi_cmd_vel", self._on_command, 10)
        self.create_subscription(Odometry, "map_odom", self._on_odometry, 10)
        self.create_subscription(Image, "depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, "depth/camera_info", self._on_camera_info, qos_profile_sensor_data
        )
        self.create_timer(0.05, self._publish_safe_command)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_command(self, message: Twist) -> None:
        self._command = MotionCommand(
            message.linear.x, message.linear.y, message.angular.z, message.linear.z
        )
        self._command_received = self._now_seconds()

    def _on_odometry(self, message: Odometry) -> None:
        self._odom = message

    def _on_depth(self, message: Image) -> None:
        self._depth = message
        self._depth_received = self._now_seconds()

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._half_fov = camera_horizontal_half_fov(message)

    def _publish_safe_command(self) -> None:
        now = self._now_seconds()
        if (
            self._command is None
            or self._command_received is None
            or now - self._command_received > self._command_timeout
            or self._odom is None
        ):
            decision = MotionDecision(0.0, 0.0, 0.0, 0.0, False)
        else:
            relative_heading = math.atan2(self._command.left, self._command.forward)
            depth_ready = (
                self._depth is not None
                and depth_is_fresh(self._depth_received, now, self._depth_timeout)
                and image_has_valid_depth(self._depth, relative_heading, self._half_fov)
            )
            decision = self._policy.evaluate(
                self._command, odometry_yaw(self._odom), self._half_fov, depth_ready, now
            )
        command = Twist()
        command.linear.x = decision.forward
        command.linear.y = decision.left
        command.linear.z = decision.up
        command.angular.z = decision.yaw_rate
        self._command_publisher.publish(command)
        self._active_publisher.publish(Bool(data=decision.active))


def main() -> None:
    rclpy.init()
    node = FovMotionGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
