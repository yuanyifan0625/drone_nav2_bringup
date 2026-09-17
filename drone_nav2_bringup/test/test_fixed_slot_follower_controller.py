"""Public geometry and safety contracts for Fixed V-Slot followers."""

import math
import importlib.util
from pathlib import Path


_SCRIPT = Path(__file__).parents[1] / "scripts" / "fixed_slot_follower_controller.py"
_SPEC = importlib.util.spec_from_file_location("fixed_slot_follower_controller", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
body_offset_to_map_target = _MODULE.body_offset_to_map_target
map_frame_formation_target_pose = _MODULE.map_frame_formation_target_pose


def test_left_rear_slot_rotates_with_leader_enu_yaw() -> None:
    """MAV2 stays left-rear in Leader FLU when the Leader faces north."""

    target_x, target_y = body_offset_to_map_target(
        leader_x=2.0,
        leader_y=1.0,
        leader_yaw=math.pi / 2.0,
        slot_forward=-0.8,
        slot_left=0.8,
    )

    assert math.isclose(target_x, 1.2, abs_tol=1e-9)
    assert math.isclose(target_y, 0.2, abs_tol=1e-9)


def test_formation_target_pose_is_map_frame_and_keeps_leader_flight_level() -> None:
    """The migration seam exposes MAV2's yaw-relative slot as a map-frame pose."""

    target = map_frame_formation_target_pose(
        leader_x=2.0,
        leader_y=1.0,
        leader_z=3.0,
        leader_yaw=math.pi / 2.0,
        slot_forward=-0.8,
        slot_left=0.8,
    )

    assert target.header.frame_id == "map"
    assert math.isclose(target.pose.position.x, 1.2, abs_tol=1e-9)
    assert math.isclose(target.pose.position.y, 0.2, abs_tol=1e-9)
    assert math.isclose(target.pose.position.z, 3.0, abs_tol=1e-9)
    assert math.isclose(
        target.pose.orientation.z, math.sin(math.pi / 4.0), abs_tol=1e-9
    )
    assert math.isclose(
        target.pose.orientation.w, math.cos(math.pi / 4.0), abs_tol=1e-9
    )
