#!/usr/bin/env python3
"""Repository-bootstrap contract for the independent Nav2 integration source."""

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class RepositoryBootstrapTest(unittest.TestCase):
    def test_repository_exposes_the_two_declared_ros_packages(self):
        package_names = {
            package_xml.read_text(encoding="utf-8").split("<name>", 1)[1].split(
                "</name>", 1
            )[0]
            for package_xml in REPOSITORY_ROOT.glob("*/package.xml")
        }

        self.assertEqual(
            package_names,
            {"drone_nav2_bringup", "drone_px4_nav2_bridge"},
        )

    def test_repository_keeps_source_assets_and_ignores_local_artifacts(self):
        ignore_rules = (REPOSITORY_ROOT / ".gitignore").read_text(encoding="utf-8")

        for ignored_path in (
            "build/",
            "install/",
            "log/",
            "__pycache__/",
            ".pytest_cache/",
            ".venv/",
            ".vscode/",
            ".ros/",
        ):
            self.assertIn(ignored_path, ignore_rules)

        for source_directory in (
            "drone_nav2_bringup/launch",
            "drone_nav2_bringup/config",
            "drone_nav2_bringup/rviz",
            "drone_px4_nav2_bridge/drone_px4_nav2_bridge",
        ):
            self.assertTrue((REPOSITORY_ROOT / source_directory).is_dir())

    def test_readme_distinguishes_host_git_from_container_ros_work(self):
        readme = (REPOSITORY_ROOT / "README.md").read_text(encoding="utf-8")

        self.assertIn("Host", readme)
        self.assertIn("container", readme)
        self.assertIn("Git", readme)
        self.assertIn("colcon build", readme)


if __name__ == "__main__":
    unittest.main()
