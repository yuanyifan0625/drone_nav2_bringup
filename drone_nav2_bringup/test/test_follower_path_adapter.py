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


def test_short_path_only_updates_after_material_slot_motion() -> None:
    """A 20 Hz target stream must not cause 20 Hz FollowPath replacements."""

    previous = PoseStamped()
    target = PoseStamped()
    target.pose.position.x = 0.14
    assert not target_moved_enough(previous, target, 0.15)
    target.pose.position.x = 0.15
    assert target_moved_enough(previous, target, 0.15)
