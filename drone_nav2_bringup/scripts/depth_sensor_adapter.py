#!/usr/bin/env python3
"""Normalize one Gazebo depth camera into a vehicle-scoped ROS interface."""

from copy import deepcopy

from nav_msgs.msg import Odometry
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def fresh_odometry_for_cloud(
    vehicle_names: list[str],
    odometry: dict[str, Odometry],
    received_ns: dict[str, int],
    *,
    cloud_stamp_ns: int,
    now_ns: int,
    tolerance: float,
) -> list[Odometry]:
    """Return odometry aligned to the cloud stamp, with receipt-time fallback."""
    tolerance_ns = int(tolerance * 1e9)
    fresh = []
    for vehicle in vehicle_names:
        message = odometry.get(vehicle)
        if message is None:
            continue
        odom_stamp_ns = _stamp_ns(message.header.stamp)
        if cloud_stamp_ns and odom_stamp_ns:
            age_ns = abs(cloud_stamp_ns - odom_stamp_ns)
        else:
            age_ns = now_ns - received_ns.get(vehicle, 0)
        if 0 <= age_ns <= tolerance_ns:
            fresh.append(message)
    return fresh


def _rotate(vector, orientation):
    x, y, z = vector
    qx, qy, qz, qw = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def _rotation_matrix(orientation):
    return np.asarray(
        [
            _rotate(axis, orientation)
            for axis in (
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0),
            )
        ]
    ).T


def _known_vehicle_mask_and_clearing_points(
    points,
    *,
    self_odom,
    peer_odoms,
    camera_offset,
    xy_half_extent,
    z_bounds,
    tolerance,
):
    if not len(points):
        return np.zeros(0, dtype=bool), points
    self_pose = self_odom.pose.pose
    self_rotation = _rotation_matrix(self_pose.orientation).astype(
        points.dtype, copy=False
    )
    attributable = np.zeros(len(points), dtype=bool)
    clearing_scale = np.zeros(len(points), dtype=points.dtype)
    lower_bounds = np.asarray(
        [-xy_half_extent, -xy_half_extent, z_bounds[0]], dtype=points.dtype
    ) - tolerance
    upper_bounds = np.asarray(
        [xy_half_extent, xy_half_extent, z_bounds[1]], dtype=points.dtype
    ) + tolerance
    for peer_odom in peer_odoms:
        peer_pose = peer_odom.pose.pose
        peer_rotation = _rotation_matrix(peer_pose.orientation).astype(
            points.dtype, copy=False
        )
        transform = self_rotation.T @ peer_rotation
        translation = (
            np.asarray(camera_offset, dtype=points.dtype) @ transform
            + np.asarray(
                [
                    self_pose.position.x - peer_pose.position.x,
                    self_pose.position.y - peer_pose.position.y,
                    self_pose.position.z - peer_pose.position.z,
                ],
                dtype=points.dtype,
            )
            @ peer_rotation
        )
        directions = points @ transform
        transformed = directions + translation
        inside = np.all(
            (transformed >= lower_bounds) & (transformed <= upper_bounds), axis=1
        )
        with np.errstate(divide="ignore", invalid="ignore"):
            far_scale = np.where(
                directions > 0.0,
                (upper_bounds - translation) / directions,
                np.where(
                    directions < 0.0,
                    (lower_bounds - translation) / directions,
                    np.inf,
                ),
            ).min(axis=1)
        valid_exit = inside & np.isfinite(far_scale) & (far_scale >= 1.0)
        clearing_scale = np.maximum(
            clearing_scale, np.where(valid_exit, far_scale, 0.0)
        )
        attributable |= inside
    has_clearing_ray = clearing_scale > 0.0
    clearing_points = points[has_clearing_ray] * clearing_scale[has_clearing_ray, None]
    return attributable, clearing_points


def _known_vehicle_mask(points, **kwargs):
    return _known_vehicle_mask_and_clearing_points(points, **kwargs)[0]


def filter_known_vehicle_points(
    points: list[tuple[float, float, float]],
    *,
    self_odom: Odometry,
    peer_odoms: list[Odometry],
    camera_offset: tuple[float, float, float],
    xy_half_extent: float,
    z_bounds: tuple[float, float],
    tolerance: float,
) -> list[tuple[float, float, float]]:
    """Remove only depth returns inside a time-aligned known vehicle body."""
    point_array = np.asarray(points, dtype=float).reshape((-1, 3))
    attributable = _known_vehicle_mask(
        point_array,
        self_odom=self_odom,
        peer_odoms=peer_odoms,
        camera_offset=camera_offset,
        xy_half_extent=xy_half_extent,
        z_bounds=z_bounds,
        tolerance=tolerance,
    )
    return [tuple(point) for point in point_array[~attributable]]


def filter_and_clear_pointcloud_known_vehicles(
    message: PointCloud2,
    *,
    self_odom: Odometry,
    peer_odoms: list[Odometry],
    camera_offset: tuple[float, float, float],
    xy_half_extent: float,
    z_bounds: tuple[float, float],
    tolerance: float,
) -> tuple[PointCloud2, PointCloud2]:
    """Return filtered marking points and bounded clearing-only rays."""
    points = point_cloud2.read_points_numpy(
        message, field_names=["x", "y", "z"], skip_nans=True
    )
    finite_points = points[np.isfinite(points).all(axis=1)]
    attributable, clearing_points = _known_vehicle_mask_and_clearing_points(
        finite_points,
        self_odom=self_odom,
        peer_odoms=peer_odoms,
        camera_offset=camera_offset,
        xy_half_extent=xy_half_extent,
        z_bounds=z_bounds,
        tolerance=tolerance,
    )
    kept = finite_points[~attributable].astype(np.float32, copy=False)
    clearing_points = clearing_points.astype(np.float32, copy=False)
    return (
        point_cloud2.create_cloud_xyz32(deepcopy(message.header), kept),
        point_cloud2.create_cloud_xyz32(deepcopy(message.header), clearing_points),
    )


def filter_pointcloud_known_vehicles(message: PointCloud2, **kwargs) -> PointCloud2:
    """Return an XYZ cloud without time-aligned known-vehicle returns."""
    return filter_and_clear_pointcloud_known_vehicles(message, **kwargs)[0]


class DepthSensorAdapter(Node):
    """Relay sensor data while assigning the vehicle-scoped camera frame."""

    def __init__(self) -> None:
        super().__init__("depth_sensor_adapter")
        frame_id = self.declare_parameter("frame_id", "MAV1/depth_camera_link").value
        image_input = self.declare_parameter("image_input", "depth/raw/image").value
        camera_info_input = self.declare_parameter(
            "camera_info_input", "depth/raw/camera_info"
        ).value
        points_input = self.declare_parameter("points_input", "depth/raw/points").value

        self._frame_id = str(frame_id)
        self._vehicle = str(
            self.declare_parameter("vehicle_namespace", "MAV1").value
        )
        self._filtered_vehicles = [
            vehicle
            for vehicle in self.declare_parameter(
                "filtered_vehicles", [""]
            ).value
            if vehicle
        ]
        self._camera_offset = tuple(
            float(value)
            for value in self.declare_parameter(
                "camera_offset", [0.13233, 0.0, 0.26078]
            ).value
        )
        self._vehicle_xy_half_extent = float(
            self.declare_parameter("vehicle_xy_half_extent", 0.315).value
        )
        self._vehicle_z_bounds = tuple(
            float(value)
            for value in self.declare_parameter(
                "vehicle_z_bounds", [-0.25, 0.12]
            ).value
        )
        self._geometry_tolerance = float(
            self.declare_parameter("vehicle_geometry_tolerance", 0.05).value
        )
        self._odometry_tolerance = float(
            self.declare_parameter("odometry_tolerance", 0.1).value
        )
        self._odometry: dict[str, Odometry] = {}
        self._received_ns: dict[str, int] = {}
        self._image_publisher = self.create_publisher(
            Image, "depth/image_raw", qos_profile_sensor_data
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, "depth/camera_info", qos_profile_sensor_data
        )
        self._points_publisher = self.create_publisher(
            PointCloud2, "depth/points", qos_profile_sensor_data
        )
        self._clearing_publisher = self.create_publisher(
            PointCloud2, "depth/teammate_clearing_points", qos_profile_sensor_data
        )
        self.create_subscription(
            Image, str(image_input), self._relay_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            str(camera_info_input),
            self._relay_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, str(points_input), self._relay_points, qos_profile_sensor_data
        )
        if self._filtered_vehicles:
            for vehicle in sorted({self._vehicle, *self._filtered_vehicles}):
                self.create_subscription(
                    Odometry,
                    f"/{vehicle}/map_odom",
                    lambda message, name=vehicle: self._on_odom(name, message),
                    10,
                )

    def _relay_image(self, message: Image) -> None:
        message.header.frame_id = self._frame_id
        self._image_publisher.publish(message)

    def _relay_camera_info(self, message: CameraInfo) -> None:
        message.header.frame_id = self._frame_id
        self._camera_info_publisher.publish(message)

    def _on_odom(self, vehicle: str, message: Odometry) -> None:
        self._odometry[vehicle] = message
        self._received_ns[vehicle] = self.get_clock().now().nanoseconds

    def _relay_points(self, message: PointCloud2) -> None:
        output = message
        clearing = point_cloud2.create_cloud_xyz32(
            deepcopy(message.header), np.empty((0, 3), dtype=np.float32)
        )
        if self._filtered_vehicles:
            now_ns = self.get_clock().now().nanoseconds
            cloud_stamp_ns = _stamp_ns(message.header.stamp)
            self_odometry = fresh_odometry_for_cloud(
                [self._vehicle],
                self._odometry,
                self._received_ns,
                cloud_stamp_ns=cloud_stamp_ns,
                now_ns=now_ns,
                tolerance=self._odometry_tolerance,
            )
            peer_odometry = fresh_odometry_for_cloud(
                self._filtered_vehicles,
                self._odometry,
                self._received_ns,
                cloud_stamp_ns=cloud_stamp_ns,
                now_ns=now_ns,
                tolerance=self._odometry_tolerance,
            )
            if self_odometry and peer_odometry:
                output, clearing = filter_and_clear_pointcloud_known_vehicles(
                    message,
                    self_odom=self_odometry[0],
                    peer_odoms=peer_odometry,
                    camera_offset=self._camera_offset,
                    xy_half_extent=self._vehicle_xy_half_extent,
                    z_bounds=self._vehicle_z_bounds,
                    tolerance=self._geometry_tolerance,
                )
        output.header.frame_id = self._frame_id
        clearing.header.frame_id = self._frame_id
        self._points_publisher.publish(output)
        self._clearing_publisher.publish(clearing)


def main() -> None:
    rclpy.init()
    node = DepthSensorAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
