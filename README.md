# drone_nav2_bringup

This repository contains the ROS 2 packages that integrate the PX4 navigation
simulation with Nav2. It is independent from the arena repository, but both
repositories are cloned into the same ROS Workspace so their ROS package
dependencies can be built and run together.

## Host and container responsibilities

### Host

Use Git on the host against this bind-mounted source: clone, branch, status,
commit, and push all happen on the host. The host owns repository history and
does not build ROS packages for this project.

### Development container

Use the development container for ROS work. Source the ROS environment and run
`colcon build` from the ROS Workspace in the container. ROS launch, topic, TF,
action, and RViz validation also run in the container.

The generated workspace `build`, `install`, and `log` directories are local
artifacts. They are not source assets and must not be committed.

## Packages

- `drone_nav2_bringup`: Nav2 launch composition, parameters, RViz settings,
  and the future Plan Goal Bridge.
- `drone_px4_nav2_bridge`: PX4-to-ROS coordinate and message conversion.

## Integrated RViz and arena markers

`plan_only.launch.py` keeps RViz disabled by default. After launching
Plan-Only, start one Integrated RViz manually with the package's RViz config.
It overlays the Static Occupancy Map, arena walls, route graph, MAV1 TF, the
MAV1 global plan, and the PX4-derived vehicle marker in the shared `map` frame.

Plan-Only starts the arena package's Arena Marker Publisher
(`graph_markers.py`). It reads the arena SDF and GeoJSON to publish
`/arena_walls` and `/route_graph`; it also derives `/vehicle_marker` from PX4
local position. These markers are visualization and Coordinate Cross-Check
assets: compare the PX4-derived marker with `MAV1/base_link` TF and
`/MAV1/plan` to expose NED-to-ENU or origin-alignment disagreement. They are
not a Nav2 map, localization source, planner input, or vehicle-control path.

```bash
rviz2 -d "$(ros2 pkg prefix drone_nav2_bringup)/share/drone_nav2_bringup/rviz/mav1_plan_only.rviz"
```

Plan-Only still has no PX4 control path or flight-command publisher.
