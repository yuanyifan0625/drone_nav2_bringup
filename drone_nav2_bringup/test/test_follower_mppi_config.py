"""Follower MPPI YAML must preserve the shared formation-heading contract."""

from pathlib import Path

import yaml


CONFIG_DIRECTORY = Path(__file__).parents[1] / "config"


def _parameters(vehicle: str) -> dict:
    with (CONFIG_DIRECTORY / f"{vehicle.lower()}_follower_mppi.yaml").open() as stream:
        return yaml.safe_load(stream)


def _follow_path_config(vehicle: str) -> dict:
    return _parameters(vehicle)["follower_mppi_controller_server"]["ros__parameters"][
        "FollowPath"
    ]


def _controller_parameters(vehicle: str) -> dict:
    return _parameters(vehicle)["follower_mppi_controller_server"]["ros__parameters"]


def _local_costmap_parameters(vehicle: str) -> dict:
    return _parameters(vehicle)["local_costmap"]["local_costmap"]["ros__parameters"]


def test_followers_do_not_complete_small_moving_slots() -> None:
    for vehicle in ("MAV2", "MAV3"):
        assert _controller_parameters(vehicle)["goal_checker"]["xy_goal_tolerance"] == 0.05


def test_followers_configure_the_same_formation_heading_costs() -> None:
    mav2 = _follow_path_config("MAV2")
    mav3 = _follow_path_config("MAV3")
    expected_critics = [
        "ConstraintCritic", "ObstaclesCritic", "GoalCritic", "GoalAngleCritic",
    ]
    expected_settings = {
        "ConstraintCritic": {"cost_weight": 4.0},
        "ObstaclesCritic": {"repulsion_weight": 0.4, "critical_weight": 20.0},
        "GoalCritic": {"cost_weight": 5.0, "threshold_to_consider": 2.1},
        "GoalAngleCritic": {"cost_weight": 3.0, "threshold_to_consider": 2.5},
    }

    for config in (mav2, mav3):
        assert config["critics"] == expected_critics
        for critic, settings in expected_settings.items():
            assert config[critic] == settings
        assert "PathAlignCritic" not in config
        assert "PathFollowCritic" not in config


def test_followers_accept_obstacles_at_flight_level() -> None:
    expected_height_range = {
        "min_obstacle_height": 0.0,
        "max_obstacle_height": 10.0,
    }

    for vehicle in ("MAV2", "MAV3"):
        obstacle_layer = _local_costmap_parameters(vehicle)["obstacle_layer"]
        for parameter, value in expected_height_range.items():
            assert obstacle_layer[parameter] == value
            assert obstacle_layer["depth_points"][parameter] == value
            assert obstacle_layer["cooperative_obstacles"][parameter] == value


def test_followers_overlay_the_shared_static_map() -> None:
    for vehicle in ("MAV2", "MAV3"):
        costmap = _local_costmap_parameters(vehicle)
        assert costmap["plugins"] == [
            "static_layer", "obstacle_layer", "inflation_layer",
        ]
        assert costmap["static_layer"] == {
            "plugin": "nav2_costmap_2d::StaticLayer",
            "map_topic": "/map",
            "map_subscribe_transient_local": True,
        }
