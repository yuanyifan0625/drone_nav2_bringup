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
map_error_to_flu = _MODULE.map_error_to_flu
limited_follower_command = _MODULE.limited_follower_command
prevent_leader_closing_command = _MODULE.prevent_leader_closing_command


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


def test_map_error_becomes_follower_forward_and_left_command() -> None:
    """A map-frame north error is forward for a north-facing follower."""

    forward, left = map_error_to_flu(
        error_east=0.0,
        error_north=0.2,
        follower_yaw=math.pi / 2.0,
    )

    assert math.isclose(forward, 0.2, abs_tol=1e-9)
    assert math.isclose(left, 0.0, abs_tol=1e-9)


def test_follower_command_is_limited_and_tracks_leader_yaw() -> None:
    """The public cmd_vel contract limits FLU speed and angular-z rate."""

    forward, left, yaw_rate = limited_follower_command(
        error_forward=4.0,
        error_left=-3.0,
        yaw_error=math.pi,
        position_gain=1.0,
        yaw_gain=1.0,
        max_linear_speed=0.6,
        max_yaw_rate=0.8,
    )

    assert math.isclose(math.hypot(forward, left), 0.6, abs_tol=1e-9)
    assert math.isclose(yaw_rate, 0.8, abs_tol=1e-9)


def test_close_follower_cannot_continue_toward_leader() -> None:
    """The Fixed V-Slot safety gate stops a follower before it breaches spacing."""

    forward, left = prevent_leader_closing_command(
        forward=-0.4,
        left=0.0,
        follower_yaw=0.0,
        leader_x=0.0,
        leader_y=0.0,
        follower_x=0.8,
        follower_y=0.0,
        minimum_leader_distance=0.9,
    )

    assert forward == 0.0
    assert left == 0.0
