"""Geometry contract for follower Cooperative Obstacles."""

import importlib.util
import math
from pathlib import Path

from nav_msgs.msg import Odometry


_SCRIPT = Path(__file__).parents[1] / "scripts" / "cooperative_obstacle_publisher.py"
_SPEC = importlib.util.spec_from_file_location("cooperative_obstacle_publisher", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
peer_footprint_points = _MODULE.peer_footprint_points
fresh_peer_odometry = _MODULE.fresh_peer_odometry


def _odom(x: float, y: float, yaw: float = 0.0) -> Odometry:
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def test_stale_peer_does_not_hide_a_fresh_peer() -> None:
    """One stale stream cannot erase the other peer obstacle."""

    mav1 = _odom(1.0, 0.0)
    mav2 = _odom(0.0, 0.0)
    mav3 = _odom(-1.0, 0.0)
    peers = fresh_peer_odometry(
        vehicle="MAV2",
        odometry={"MAV1": mav1, "MAV2": mav2, "MAV3": mav3},
        received_ns={"MAV1": 900_000_000, "MAV2": 1_000_000_000, "MAV3": 600_000_000},
        now_ns=1_000_000_000,
        timeout=0.3,
    )

    assert peers == [mav1]


def test_peer_footprints_are_follower_local_and_bounded() -> None:
    """Only nearby peers become conservative obstacle disks in the base frame."""

    points = peer_footprint_points(
        self_odom=_odom(1.0, 2.0, math.pi / 2.0),
        peer_odoms=[_odom(1.0, 3.0), _odom(8.0, 2.0)],
        radius=0.25,
        resolution=0.05,
        max_range=2.4,
    )

    assert points
    assert all(math.hypot(x - 1.0, y) <= 0.251 for x, y, _ in points)
    assert all(z == 0.0 for _, _, z in points)
