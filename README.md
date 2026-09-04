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

The repository bootstrap deliberately contains no PX4 control path, Nav2
planner launch, or flight-command publisher. Those capabilities are added by
later tickets.
