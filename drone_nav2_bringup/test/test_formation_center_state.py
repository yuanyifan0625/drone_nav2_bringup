"""Public geometry and ROS seams for the derived Formation Center state."""

import importlib.util
import math
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


_SCRIPT = Path(__file__).parents[1] / "scripts" / "formation_center_state.py"
_SPEC = importlib.util.spec_from_file_location("formation_center_state", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)

formation_centroid = _MODULE.formation_centroid
formation_envelope_radius = _MODULE.formation_envelope_radius
physical_mav1_command = _MODULE.physical_mav1_command
virtual_center_odometry = _MODULE.virtual_center_odometry
virtual_center_transform = _MODULE.virtual_center_transform


def test_fixed_v_slots_define_expected_centroid_and_conservative_envelope() -> None:
    """The default V formation exposes its agreed center and 1.45 m envelope."""
    points = ((0.0, 0.0), (-0.8, 0.8), (-0.8, -0.8))

    center = formation_centroid(points)
    radius = formation_envelope_radius(
        points,
        center=center,
        vehicle_radius=0.4,
        safety_margin=0.2,
        rounding_increment=0.05,
    )

    assert math.isclose(center[0], -0.5333333333333333, abs_tol=1e-9)
    assert math.isclose(center[1], 0.0, abs_tol=1e-9)
    assert math.isclose(radius, 1.45, abs_tol=1e-9)


def test_virtual_transform_keeps_physical_mav1_as_parent() -> None:
    """Formation Center is a fixed derived child, never MAV1's physical frame."""
    transform = virtual_center_transform(
        parent_frame="MAV1/base_link",
        child_frame="MAV1/formation_center",
        forward=-0.5333333333333333,
        left=0.0,
    )

    assert transform.header.frame_id == "MAV1/base_link"
    assert transform.child_frame_id == "MAV1/formation_center"
    assert math.isclose(transform.transform.translation.x, -0.5333333333333333)
    assert transform.transform.translation.y == 0.0
    assert transform.transform.rotation.w == 1.0


def test_virtual_odometry_includes_turning_velocity_at_the_offset_center() -> None:
    """A rotating physical MAV1 gives its offset virtual center extra velocity."""
    leader = Odometry()
    leader.header.frame_id = "map"
    leader.child_frame_id = "MAV1/base_link"
    leader.pose.pose.position.x = 2.0
    leader.pose.pose.position.y = 1.0
    leader.pose.pose.position.z = 3.0
    leader.pose.pose.orientation.z = math.sin(math.pi / 4.0)
    leader.pose.pose.orientation.w = math.cos(math.pi / 4.0)
    leader.twist.twist.linear.x = 1.0
    leader.twist.twist.linear.y = 2.0

    center = virtual_center_odometry(
        leader,
        child_frame="MAV1/formation_center",
        offset_forward=-0.5333333333333333,
        offset_left=0.0,
        yaw_rate=0.5,
    )

    assert center.header.frame_id == "map"
    assert center.child_frame_id == "MAV1/formation_center"
    assert math.isclose(center.pose.pose.position.x, 2.0, abs_tol=1e-9)
    assert math.isclose(center.pose.pose.position.y, 0.4666666666666667, abs_tol=1e-9)
    assert math.isclose(center.pose.pose.position.z, 3.0, abs_tol=1e-9)
    assert math.isclose(center.twist.twist.linear.x, 2.0, abs_tol=1e-9)
    assert math.isclose(center.twist.twist.linear.y, -1.2666666666666666, abs_tol=1e-9)
    assert math.isclose(center.twist.twist.angular.z, 0.5, abs_tol=1e-9)


def test_virtual_command_converts_to_physical_mav1_body_velocity() -> None:
    """Yawing about an offset center adds the required MAV1 lateral velocity."""
    command = Twist()
    command.linear.x = 0.4
    command.linear.y = 0.2
    command.linear.z = 0.1
    command.angular.z = 0.3

    physical = physical_mav1_command(
        command,
        center_forward=-0.5333333333333333,
        center_left=0.0,
    )

    assert math.isclose(physical.linear.x, 0.4, abs_tol=1e-9)
    assert math.isclose(physical.linear.y, 0.36, abs_tol=1e-9)
    assert math.isclose(physical.linear.z, 0.1, abs_tol=1e-9)
    assert math.isclose(physical.angular.z, 0.3, abs_tol=1e-9)
