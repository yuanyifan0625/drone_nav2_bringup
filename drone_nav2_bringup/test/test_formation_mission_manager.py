"""Minimal behavior checks for Formation Mission Manager form-up readiness."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest

from nav_msgs.msg import Odometry
import rclpy


_SCRIPT = Path(__file__).parents[1] / "scripts" / "formation_mission_manager.py"
sys.path.insert(0, str(_SCRIPT.parent))
_SPEC = importlib.util.spec_from_file_location("formation_mission_manager", _SCRIPT)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
FormationMissionManager = _MODULE.FormationMissionManager
wrapped_yaw_error = _MODULE.wrapped_yaw_error


class FormationMissionManagerTest(unittest.TestCase):
    """Form-up enters leader navigation only after the public readiness predicate."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.manager = FormationMissionManager()
        self.manager._publish_phase("form_up", "test")
        self.navigation_starts = 0
        self.manager._start_leader_navigation = self._record_navigation_start
        self._set_formation(yaw=0.0)

    def tearDown(self):
        self.manager.destroy_node()

    @staticmethod
    def _odometry(x: float, y: float, yaw: float) -> Odometry:
        message = Odometry()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = 3.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return message

    def _set_formation(self, yaw: float) -> None:
        self.manager._odometry = {
            "MAV1": self._odometry(0.0, 0.0, 0.0),
            "MAV2": self._odometry(-0.8, 0.8, yaw),
            "MAV3": self._odometry(-0.8, -0.8, yaw),
        }
        now_ns = self.manager.get_clock().now().nanoseconds
        self.manager._odom_received_ns = {vehicle: now_ns for vehicle in self.manager._vehicles}

    def _record_navigation_start(self) -> None:
        self.navigation_starts += 1

    def test_wrapped_yaw_error_handles_pi_boundary(self):
        self.assertAlmostEqual(
            0.02,
            abs(wrapped_yaw_error(math.pi - 0.01, -math.pi + 0.01)),
            places=6,
        )

    def test_form_up_resets_convergence_until_follower_yaw_aligns(self):
        self.manager._slot_converged_since_ns = self.manager.get_clock().now().nanoseconds
        self._set_formation(yaw=0.3)

        self.manager._tick()

        self.assertEqual("form_up", self.manager._phase)
        self.assertIsNone(self.manager._slot_converged_since_ns)
        self.assertEqual(0, self.navigation_starts)

        self.manager._slot_convergence_seconds = 0.0
        self._set_formation(yaw=0.0)
        self.manager._tick()

        self.assertEqual(1, self.navigation_starts)

    def test_form_up_timeout_enters_existing_abort_hold(self):
        self.manager._form_up_timeout_seconds = 0.0
        self.manager._phase_started_ns = self.manager.get_clock().now().nanoseconds - int(1e9)
        self._set_formation(yaw=0.3)

        self.manager._tick()

        self.assertEqual("abort_hold", self.manager._phase)


if __name__ == "__main__":
    unittest.main()
