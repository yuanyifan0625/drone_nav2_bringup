"""Rate-boundary contract for the follower's moving short path."""

import importlib.util
from math import cos, pi, sin
from pathlib import Path

from geometry_msgs.msg import PoseStamped


_SCRIPT = Path(__file__).parents[1] / "scripts" / "follower_path_adapter.py"
_SPEC = importlib.util.spec_from_file_location("follower_path_adapter", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
target_moved_enough = _MODULE.target_moved_enough
short_straight_path = _MODULE.short_straight_path
should_replace_path = _MODULE.should_replace_path
target_yaw_changed_enough = _MODULE.target_yaw_changed_enough


def test_short_path_only_updates_after_material_slot_motion() -> None:
    """A 20 Hz target stream must not cause 20 Hz FollowPath replacements."""

    previous = PoseStamped()
    target = PoseStamped()
    target.pose.position.x = 0.14
    assert not target_moved_enough(previous, target, 0.15)
    target.pose.position.x = 0.15
    assert target_moved_enough(previous, target, 0.15)


def test_short_path_has_intermediate_poses_for_mppi() -> None:
    start = PoseStamped()
    target = PoseStamped()
    target.header.frame_id = "map"
    target.pose.position.x = 0.8

    path = short_straight_path(start, target)

    assert len(path) == 5
    assert path[1].header.frame_id == "map"
    assert path[-1].pose.position.x == 0.8
    assert path[-1].pose.orientation == target.pose.orientation


def test_path_replacement_waits_for_material_motion_and_minimum_interval() -> None:
    previous = PoseStamped()
    target = PoseStamped()
    target.pose.position.x = 0.35

    assert not should_replace_path(
        previous, target, 0.35, 5.0 * pi / 180.0, 10.0, 10.9, 1.0, 0.2
    )
    assert should_replace_path(
        previous, target, 0.35, 5.0 * pi / 180.0, 10.0, 11.0, 1.0, 0.2
    )


def _set_yaw(pose: PoseStamped, yaw: float) -> None:
    pose.pose.orientation.z = sin(yaw / 2.0)
    pose.pose.orientation.w = cos(yaw / 2.0)


def test_yaw_only_change_replaces_path_at_five_hz() -> None:
    previous = PoseStamped()
    target = PoseStamped()
    _set_yaw(previous, 0.0)
    _set_yaw(target, 6.0 * pi / 180.0)

    assert target_yaw_changed_enough(previous, target, 5.0 * pi / 180.0)
    assert not should_replace_path(
        previous, target, 0.35, 5.0 * pi / 180.0, 10.0, 10.19, 1.0, 0.2
    )
    assert should_replace_path(
        previous, target, 0.35, 5.0 * pi / 180.0, 10.0, 10.2, 1.0, 0.2
    )


def test_small_or_unchanged_yaw_never_replaces_the_short_path() -> None:
    previous = PoseStamped()
    target = PoseStamped()
    _set_yaw(previous, 0.0)
    _set_yaw(target, 4.9 * pi / 180.0)

    assert not target_yaw_changed_enough(previous, target, 5.0 * pi / 180.0)
    assert not should_replace_path(
        previous, target, 0.35, 5.0 * pi / 180.0, 10.0, 20.0, 1.0, 0.2
    )
    _set_yaw(target, 0.0)
    assert not should_replace_path(
        previous, target, 0.35, 5.0 * pi / 180.0, 10.0, 30.0, 1.0, 0.2
    )


def test_yaw_threshold_normalizes_across_pi_boundary() -> None:
    previous = PoseStamped()
    target = PoseStamped()
    _set_yaw(previous, 179.0 * pi / 180.0)
    _set_yaw(target, -179.0 * pi / 180.0)

    assert not target_yaw_changed_enough(previous, target, 5.0 * pi / 180.0)
    _set_yaw(target, -174.0 * pi / 180.0)
    assert target_yaw_changed_enough(previous, target, 5.0 * pi / 180.0)
