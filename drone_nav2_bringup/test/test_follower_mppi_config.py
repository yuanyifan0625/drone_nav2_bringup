"""Follower MPPI YAML must preserve the shared formation-heading contract."""

from pathlib import Path

import yaml


CONFIG_DIRECTORY = Path(__file__).parents[1] / "config"


def _follow_path_config(vehicle: str) -> dict:
    with (CONFIG_DIRECTORY / f"{vehicle.lower()}_follower_mppi.yaml").open() as stream:
        parameters = yaml.safe_load(stream)
    return parameters["follower_mppi_controller_server"]["ros__parameters"]["FollowPath"]


def _controller_parameters(vehicle: str) -> dict:
    with (CONFIG_DIRECTORY / f"{vehicle.lower()}_follower_mppi.yaml").open() as stream:
        parameters = yaml.safe_load(stream)
    return parameters["follower_mppi_controller_server"]["ros__parameters"]


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
