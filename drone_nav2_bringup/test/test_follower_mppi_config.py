"""Follower MPPI YAML must preserve the shared formation-heading contract."""

from pathlib import Path

import yaml


CONFIG_DIRECTORY = Path(__file__).parents[1] / "config"


def _follow_path_config(vehicle: str) -> dict:
    with (CONFIG_DIRECTORY / f"{vehicle.lower()}_follower_mppi.yaml").open() as stream:
        parameters = yaml.safe_load(stream)
    return parameters["follower_mppi_controller_server"]["ros__parameters"]["FollowPath"]


def test_followers_configure_the_same_formation_heading_costs() -> None:
    mav2 = _follow_path_config("MAV2")
    mav3 = _follow_path_config("MAV3")

    for config in (mav2, mav3):
        assert "GoalAngleCritic" in config["critics"]
        assert config["GoalAngleCritic"] == {
            "cost_weight": 3.0,
            "threshold_to_consider": 2.5,
        }
        assert config["PathAlignCritic"]["use_path_orientations"] is True
        assert config["PathAlignCritic"]["offset_from_furthest"] == 1

    assert mav2["critics"] == mav3["critics"]
