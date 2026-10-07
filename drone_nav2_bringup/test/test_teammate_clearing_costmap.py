"""Stateful proof that bounded teammate rays clear ghosts, not real obstacles."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import yaml


class TeammateClearingCostmapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "122"
        parameters = {
            "/**": {
                "ros__parameters": {
                    "use_sim_time": False,
                    "global_frame": "map",
                    "robot_base_frame": "map",
                    "update_frequency": 20.0,
                    "publish_frequency": 20.0,
                    "rolling_window": False,
                    "width": 4,
                    "height": 4,
                    "resolution": 0.05,
                    "origin_x": -0.5,
                    "origin_y": -2.0,
                    "robot_radius": 0.1,
                    "track_unknown_space": False,
                    "always_send_full_costmap": True,
                    "plugins": ["obstacle_layer"],
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "observation_sources": (
                            "depth_points teammate_clearing protected"
                        ),
                        "depth_points": {
                            "topic": "/test/depth_points",
                            "data_type": "PointCloud2",
                            "marking": True,
                            "clearing": True,
                            "min_obstacle_height": -1.0,
                            "max_obstacle_height": 1.0,
                            "obstacle_max_range": 2.0,
                            "raytrace_max_range": 2.0,
                        },
                        "teammate_clearing": {
                            "topic": "/test/teammate_clearing",
                            "data_type": "PointCloud2",
                            "marking": False,
                            "clearing": True,
                            "min_obstacle_height": -1.0,
                            "max_obstacle_height": 1.0,
                            "raytrace_max_range": 2.0,
                        },
                        "protected": {
                            "topic": "/test/protected",
                            "data_type": "PointCloud2",
                            "marking": True,
                            "clearing": False,
                            "min_obstacle_height": -1.0,
                            "max_obstacle_height": 1.0,
                            "obstacle_max_range": 2.0,
                            "observation_persistence": 0.0,
                        },
                    },
                }
            }
        }
        cls.parameter_file = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        )
        yaml.safe_dump(parameters, cls.parameter_file)
        cls.parameter_file.close()
        cls.process = subprocess.Popen(
            [
                "ros2",
                "run",
                "nav2_costmap_2d",
                "nav2_costmap_2d",
                "--ros-args",
                "-r",
                "__node:=stateful_costmap",
                "--params-file",
                cls.parameter_file.name,
            ],
            env=os.environ.copy(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        rclpy.init()
        cls.node = Node("teammate_clearing_costmap_test")
        cls.depth = cls.node.create_publisher(
            PointCloud2, "/test/depth_points", qos_profile_sensor_data
        )
        cls.clearing = cls.node.create_publisher(
            PointCloud2,
            "/test/teammate_clearing",
            qos_profile_sensor_data,
        )
        cls.protected = cls.node.create_publisher(
            PointCloud2, "/test/protected", qos_profile_sensor_data
        )
        cls.costmap = None
        cls.node.create_subscription(
            OccupancyGrid,
            "/costmap/costmap",
            lambda message: setattr(cls, "costmap", message),
            10,
        )
        cls.lifecycle = cls.node.create_client(
            ChangeState, "/costmap/costmap/change_state"
        )
        cls._wait_for(lambda: cls.lifecycle.service_is_ready())
        cls._transition(Transition.TRANSITION_CONFIGURE)
        cls._transition(Transition.TRANSITION_ACTIVATE)
        cls._wait_for(lambda: cls.costmap is not None)
        cls._wait_for(
            lambda: all(
                publisher.get_subscription_count()
                for publisher in (cls.depth, cls.clearing, cls.protected)
            )
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.killpg(cls.process.pid, signal.SIGINT)
        try:
            cls.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(cls.process.pid, signal.SIGTERM)
            cls.process.wait(timeout=5.0)
        Path(cls.parameter_file.name).unlink(missing_ok=True)

    @classmethod
    def _wait_for(cls, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("Timed out waiting for stateful costmap")

    @classmethod
    def _transition(cls, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = cls.lifecycle.call_async(request)
        cls._wait_for(future.done)
        if not future.result().success:
            raise AssertionError(f"Lifecycle transition {transition_id} failed")

    @classmethod
    def _cloud(cls, points):
        header = Header()
        header.frame_id = "map"
        header.stamp = cls.node.get_clock().now().to_msg()
        return point_cloud2.create_cloud_xyz32(header, points)

    @classmethod
    def _publish_until(
        cls, predicate, depth, clearing=(), protected=None, timeout=5.0
    ):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls.depth.publish(cls._cloud(depth))
            cls.clearing.publish(cls._cloud(clearing))
            if protected is not None:
                cls.protected.publish(cls._cloud(protected))
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if cls.costmap is not None and predicate():
                return
        sample = {
            point: cls._cost(*point)
            for point in ((1.0, 0.0), (1.0, 0.5), (1.35, 0.0))
        }
        raise AssertionError(
            f"Costmap cells did not reach expected state: {sample}; "
            f"max={max(cls.costmap.data)}, info={cls.costmap.info}"
        )

    @classmethod
    def _cost(cls, x, y):
        info = cls.costmap.info
        column = round((x - info.origin.position.x) / info.resolution)
        row = round((y - info.origin.position.y) / info.resolution)
        return cls.costmap.data[row * info.width + column]

    def test_stale_then_fresh_and_moved_teammate_marks_are_bounded(self):
        self._publish_until(
            lambda: (
                self._cost(1.0, 0.0) == 100
                and self._cost(1.0, 0.5) == 100
                and self._cost(1.35, 0.0) == 100
            ),
            depth=[(1.0, 0.0, 0.1)],
            protected=[(1.0, 0.5, 0.1), (1.35, 0.0, 0.1)],
        )
        self._publish_until(
            lambda: (
                self._cost(1.0, 0.0) == 0
                and self._cost(1.0, 0.5) == 100
                and self._cost(1.35, 0.0) == 100
            ),
            depth=[],
            clearing=[(1.315, 0.0, 0.1)],
        )

        moved = (1.725, -0.775, 0.1)
        self._publish_until(lambda: self._cost(*moved[:2]) == 100, depth=[moved])
        self._publish_until(
            lambda: self._cost(*moved[:2]) == 0,
            depth=[],
            clearing=[(1.98, -0.89, 0.1)],
        )

        out_of_range = (2.5, 1.0, 0.1)
        self._publish_until(
            lambda: self._cost(*out_of_range[:2]) == 0,
            depth=[out_of_range],
        )
        self.assertEqual(self._cost(1.0, 0.5), 100)
        self.assertEqual(self._cost(1.35, 0.0), 100)
