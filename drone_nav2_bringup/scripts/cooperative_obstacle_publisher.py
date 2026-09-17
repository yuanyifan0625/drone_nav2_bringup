#!/usr/bin/env python3
"""Publish shared-odometry peers as local 2D PointCloud2 obstacles."""

from copy import deepcopy
import math
import struct

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


VEHICLES = ("MAV1", "MAV2", "MAV3")


def odometry_yaw(message: Odometry) -> float:
    """Return the ENU yaw represented by an odometry quaternion."""

    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def peer_footprint_points(
    *,
    self_odom: Odometry,
    peer_odoms: list[Odometry],
    radius: float,
    resolution: float,
    max_range: float,
) -> list[tuple[float, float, float]]:
    """Return filled circular peer footprints in the follower base frame."""

    self_position = self_odom.pose.pose.position
    yaw = odometry_yaw(self_odom)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    cells = math.ceil(radius / resolution)
    points: list[tuple[float, float, float]] = []
    for peer_odom in peer_odoms:
        peer_position = peer_odom.pose.pose.position
        east = peer_position.x - self_position.x
        north = peer_position.y - self_position.y
        local_x = cos_yaw * east + sin_yaw * north
        local_y = -sin_yaw * east + cos_yaw * north
        if math.hypot(local_x, local_y) > max_range:
            continue
        for x_cell in range(-cells, cells + 1):
            for y_cell in range(-cells, cells + 1):
                offset_x = x_cell * resolution
                offset_y = y_cell * resolution
                if math.hypot(offset_x, offset_y) <= radius:
                    points.append((local_x + offset_x, local_y + offset_y, 0.0))
    return points


def fresh_peer_odometry(
    *,
    vehicle: str,
    odometry: dict[str, Odometry],
    received_ns: dict[str, int],
    now_ns: int,
    timeout: float,
) -> list[Odometry]:
    """Return only fresh peer odometry without treating self as an obstacle."""

    return [
        odometry[peer]
        for peer in VEHICLES
        if (
            peer != vehicle
            and peer in odometry
            and (now_ns - received_ns.get(peer, 0)) / 1e9 <= timeout
        )
    ]


def pointcloud(header, points: list[tuple[float, float, float]]) -> PointCloud2:
    """Create an XYZ32 PointCloud2 without adding a sensor helper dependency."""

    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.width * message.point_step
    message.is_dense = True
    message.data = b"".join(struct.pack("<fff", *point) for point in points)
    return message


class CooperativeObstaclePublisher(Node):
    """Convert fresh MAV odometry into obstacles for one follower local costmap."""

    def __init__(self) -> None:
        super().__init__("cooperative_obstacle_publisher")
        self._vehicle = str(
            self.declare_parameter("vehicle_namespace", "MAV2").value
        )
        self._radius = float(
            self.declare_parameter("cooperative_radius", 0.25).value
        )
        self._resolution = float(
            self.declare_parameter("footprint_resolution", 0.05).value
        )
        self._max_range = float(
            self.declare_parameter("max_peer_range", 2.4).value
        )
        self._timeout = float(
            self.declare_parameter("observation_timeout", 0.3).value
        )
        self._odometry: dict[str, Odometry] = {}
        self._received_ns: dict[str, int] = {}
        self._publisher = self.create_publisher(
            PointCloud2, "cooperative_obstacles", qos_profile_sensor_data
        )
        for vehicle in VEHICLES:
            self.create_subscription(
                Odometry,
                f"/{vehicle}/odom",
                lambda message, name=vehicle: self._on_odom(name, message),
                10,
            )
        self.create_timer(0.05, self._publish_obstacles)

    def _on_odom(self, vehicle: str, message: Odometry) -> None:
        self._odometry[vehicle] = message
        self._received_ns[vehicle] = self.get_clock().now().nanoseconds

    def _publish_obstacles(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._vehicle not in self._odometry
            or (now_ns - self._received_ns.get(self._vehicle, 0)) / 1e9
            > self._timeout
        ):
            return
        self_odom = self._odometry[self._vehicle]
        peers = fresh_peer_odometry(
            vehicle=self._vehicle,
            odometry=self._odometry,
            received_ns=self._received_ns,
            now_ns=now_ns,
            timeout=self._timeout,
        )
        header = deepcopy(self_odom.header)
        header.frame_id = f"{self._vehicle}/base_link"
        self._publisher.publish(
            pointcloud(
                header,
                peer_footprint_points(
                    self_odom=self_odom,
                    peer_odoms=peers,
                    radius=self._radius,
                    resolution=self._resolution,
                    max_range=self._max_range,
                ),
            )
        )


def main() -> None:
    rclpy.init()
    node = CooperativeObstaclePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
