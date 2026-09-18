"""Safety contract for follower front-camera FOV motion gating."""

import importlib.util
from math import pi
from pathlib import Path

from sensor_msgs.msg import CameraInfo, Image


_SCRIPT = Path(__file__).parents[1] / "scripts" / "follower_fov_motion_guard.py"
_SPEC = importlib.util.spec_from_file_location("follower_fov_motion_guard", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
FovMotionPolicy = _MODULE.FovMotionPolicy
MotionCommand = _MODULE.MotionCommand
camera_horizontal_half_fov = _MODULE.camera_horizontal_half_fov
depth_is_fresh = _MODULE.depth_is_fresh
image_has_valid_depth = _MODULE.image_has_valid_depth


def test_forward_motion_with_fresh_depth_passes_through() -> None:
    policy = FovMotionPolicy()
    command = MotionCommand(forward=0.3, left=0.0, yaw_rate=0.2)

    decision = policy.evaluate(command, yaw=0.0, half_fov=0.6, depth_ready=True, now=1.0)

    assert not decision.active
    assert decision.forward == 0.3
    assert decision.left == 0.0
    assert decision.yaw_rate == 0.2


def test_sideways_motion_rotates_before_translation() -> None:
    policy = FovMotionPolicy(yaw_gain=2.0, max_yaw_rate=0.6)
    command = MotionCommand(forward=0.0, left=0.4, yaw_rate=0.0)

    decision = policy.evaluate(command, yaw=0.0, half_fov=0.6, depth_ready=True, now=1.0)

    assert decision.active
    assert decision.forward == 0.0
    assert decision.left == 0.0
    assert decision.yaw_rate == 0.6


def test_stale_or_invalid_depth_blocks_even_forward_motion() -> None:
    command = MotionCommand(forward=0.3, left=0.0, yaw_rate=0.1)

    stale = FovMotionPolicy().evaluate(command, 0.0, 0.6, False, 1.0)

    assert stale.active
    assert stale.forward == 0.0
    assert stale.left == 0.0
    assert stale.yaw_rate == 0.0


def test_invalid_depth_image_and_stale_callback_are_not_ready() -> None:
    invalid = Image(width=4, height=4, encoding="32FC1", step=16, data=bytes(64))

    assert not image_has_valid_depth(invalid, heading=0.0, half_fov=0.6)
    assert not depth_is_fresh(received=1.0, now=1.31, timeout=0.3)


def test_guard_does_not_release_a_command_that_changed_direction() -> None:
    policy = FovMotionPolicy(release_seconds=0.4)

    policy.evaluate(MotionCommand(0.0, 0.4, 0.0), 0.0, 0.6, True, 1.0)
    changed = policy.evaluate(MotionCommand(0.0, 0.4, 0.0), pi / 2.0, 0.6, True, 1.1)

    assert changed.active
    assert changed.forward == 0.0 and changed.left == 0.0


def test_covered_direction_releases_translation_gradually() -> None:
    policy = FovMotionPolicy(release_seconds=0.4)
    command = MotionCommand(forward=0.0, left=0.4, yaw_rate=0.1)
    aligned_command = MotionCommand(forward=0.4, left=0.0, yaw_rate=0.1)

    policy.evaluate(command, yaw=0.0, half_fov=0.6, depth_ready=True, now=1.0)
    first = policy.evaluate(aligned_command, yaw=pi / 2.0, half_fov=0.6, depth_ready=True, now=1.1)
    halfway = policy.evaluate(aligned_command, yaw=pi / 2.0, half_fov=0.6, depth_ready=True, now=1.3)
    released = policy.evaluate(aligned_command, yaw=pi / 2.0, half_fov=0.6, depth_ready=True, now=1.5)

    assert first.active and first.left == 0.0
    assert halfway.active and 0.0 < halfway.forward < aligned_command.forward
    assert not released.active and released.forward == aligned_command.forward


def test_camera_info_defines_horizontal_fov() -> None:
    camera_info = CameraInfo(width=640)
    camera_info.k[0] = 320.0

    assert camera_horizontal_half_fov(camera_info) == pi / 4.0
