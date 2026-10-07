"""Depth points keep unknown obstacles while removing attributable teammate returns."""

import importlib.util
import math
from pathlib import Path
import struct

from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import PointCloud2, PointField


_SCRIPT = Path(__file__).parents[1] / "scripts" / "depth_sensor_adapter.py"
_SPEC = importlib.util.spec_from_file_location("depth_sensor_adapter", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
filter_and_clear_pointcloud_known_vehicles = _MODULE.filter_and_clear_pointcloud_known_vehicles
filter_known_vehicle_points = _MODULE.filter_known_vehicle_points
filter_pointcloud_known_vehicles = _MODULE.filter_pointcloud_known_vehicles
fresh_odometry_for_cloud = _MODULE.fresh_odometry_for_cloud


def _odom(
    x: float,
    y: float,
    z: float = 0.0,
    yaw: float = 0.0,
    stamp_ns: int = 1_000_000_000,
) -> Odometry:
    message = Odometry()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.position.z = z
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def _cloud(points: list[tuple[float, float, float]]) -> PointCloud2:
    message = PointCloud2()
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = b"".join(struct.pack("<fff", *point) for point in points)
    return message


def test_only_points_attributable_to_a_known_vehicle_are_removed() -> None:
    points = [
        (0.86767, 0.0, -0.26078),
        (0.86767, 0.50, -0.26078),
        (1.50, 0.0, -0.26078),
    ]

    filtered = filter_known_vehicle_points(
        points,
        self_odom=_odom(0.0, 0.0),
        peer_odoms=[_odom(1.0, 0.0)],
        camera_offset=(0.13233, 0.0, 0.26078),
        xy_half_extent=0.315,
        z_bounds=(-0.25, 0.12),
    )

    assert filtered == [points[1], points[2]]


def test_moving_vehicle_updates_mask_without_residual_points() -> None:
    old_return = (0.86767, 0.0, -0.26078)
    new_return = (1.86767, 0.0, -0.26078)
    arguments = {
        "self_odom": _odom(0.0, 0.0),
        "camera_offset": (0.13233, 0.0, 0.26078),
        "xy_half_extent": 0.315,
        "z_bounds": (-0.25, 0.12),
    }

    assert filter_known_vehicle_points(
        [old_return, new_return],
        peer_odoms=[_odom(1.0, 0.0)],
        **arguments,
    ) == [new_return]
    assert filter_known_vehicle_points(
        [old_return, new_return],
        peer_odoms=[_odom(2.0, 0.0)],
        **arguments,
    ) == [old_return]


def test_vehicle_filter_respects_self_and_peer_orientation() -> None:
    inside = (0.86767, 0.0, -0.26078)
    outside = (0.86767, 0.50, -0.26078)

    filtered = filter_known_vehicle_points(
        [inside, outside],
        self_odom=_odom(0.0, 0.0, yaw=math.pi / 2.0),
        peer_odoms=[_odom(0.0, 1.0, yaw=math.pi / 2.0)],
        camera_offset=(0.13233, 0.0, 0.26078),
        xy_half_extent=0.315,
        z_bounds=(-0.25, 0.12),
    )

    assert filtered == [outside]


def test_missing_or_stale_odometry_preserves_depth_points() -> None:
    point = (0.86767, 0.0, -0.26078)
    fresh = _odom(1.0, 0.0, stamp_ns=1_000_000_000)
    stale = _odom(1.0, 0.0, stamp_ns=600_000_000)

    assert fresh_odometry_for_cloud(
        ["MAV1"],
        {"MAV1": fresh},
        {"MAV1": 1_000_000_000},
        cloud_stamp_ns=1_000_000_000,
        now_ns=1_000_000_000,
        tolerance=0.1,
    ) == [fresh]
    assert fresh_odometry_for_cloud(
        ["MAV1"],
        {"MAV1": stale},
        {"MAV1": 1_000_000_000},
        cloud_stamp_ns=1_000_000_000,
        now_ns=1_000_000_000,
        tolerance=0.1,
    ) == []
    assert filter_known_vehicle_points(
        [point],
        self_odom=_odom(0.0, 0.0),
        peer_odoms=[],
        camera_offset=(0.13233, 0.0, 0.26078),
        xy_half_extent=0.315,
        z_bounds=(-0.25, 0.12),
    ) == [point]


def test_zero_header_stamp_uses_bounded_receipt_time_fallback() -> None:
    odom = _odom(1.0, 0.0, stamp_ns=0)

    assert fresh_odometry_for_cloud(
        ["MAV1"],
        {"MAV1": odom},
        {"MAV1": 950_000_000},
        cloud_stamp_ns=0,
        now_ns=1_000_000_000,
        tolerance=0.1,
    ) == [odom]
    assert fresh_odometry_for_cloud(
        ["MAV1"],
        {"MAV1": odom},
        {"MAV1": 800_000_000},
        cloud_stamp_ns=0,
        now_ns=1_000_000_000,
        tolerance=0.1,
    ) == []


def test_pointcloud_filter_drops_no_occluded_or_ambiguous_ray() -> None:
    inside = (0.86767, 0.0, -0.26078)
    wall_edge = (0.86767, 0.50, -0.26078)
    occluded = (1.50, 0.0, -0.26078)

    filtered = filter_pointcloud_known_vehicles(
        _cloud([inside, wall_edge, occluded]),
        self_odom=_odom(0.0, 0.0),
        peer_odoms=[_odom(1.0, 0.0)],
        camera_offset=(0.13233, 0.0, 0.26078),
        xy_half_extent=0.315,
        z_bounds=(-0.25, 0.12),
    )

    assert filtered.height == 1
    assert filtered.width == 2
    assert filtered.row_step == 24
    actual = [
        value for point in struct.iter_unpack("<fff", filtered.data) for value in point
    ]
    assert actual == pytest.approx([*wall_edge, *occluded])


def test_teammate_clearing_ray_stops_at_known_body_exit() -> None:
    teammate_surface = (0.86767, 0.0, -0.26078)
    tolerance_shell_obstacle = (1.20767, 0.0, -0.26078)
    wall_edge = (0.86767, 0.50, -0.26078)
    occluded_obstacle = (1.50, 0.0, -0.26078)

    filtered, clearing = filter_and_clear_pointcloud_known_vehicles(
        _cloud(
            [teammate_surface, tolerance_shell_obstacle, wall_edge, occluded_obstacle]
        ),
        self_odom=_odom(0.0, 0.0),
        peer_odoms=[_odom(1.0, 0.0)],
        camera_offset=(0.13233, 0.0, 0.26078),
        xy_half_extent=0.315,
        z_bounds=(-0.25, 0.12),
    )

    assert filtered.width == 3
    assert clearing.width == 1
    clear_x, clear_y, clear_z = struct.unpack("<fff", clearing.data)
    assert clear_x > teammate_surface[0]
    assert clear_x + 0.13233 == pytest.approx(1.315, abs=1e-5)
    assert clear_x < tolerance_shell_obstacle[0]
    assert clear_y == pytest.approx(0.0, abs=1e-6)
    assert clear_z / clear_x == pytest.approx(
        teammate_surface[2] / teammate_surface[0], abs=1e-6
    )
