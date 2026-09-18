"""Rate-boundary contract for the follower's moving short path."""

import importlib.util
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


def test_path_replacement_waits_for_material_motion_and_minimum_interval() -> None:
    previous = PoseStamped()
    target = PoseStamped()
    target.pose.position.x = 0.35

    assert not should_replace_path(previous, target, 0.35, 10.0, 10.9, 1.0)
    assert should_replace_path(previous, target, 0.35, 10.0, 11.0, 1.0)
