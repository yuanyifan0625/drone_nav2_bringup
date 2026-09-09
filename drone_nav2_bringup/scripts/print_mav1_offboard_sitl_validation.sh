#!/usr/bin/env bash
# Print the ordered, manual MAV1 Gazebo-SITL validation procedure.

set -eu

cat <<'EOF'
[1] Build the two Issue #5 packages.
Command:
docker compose exec ros2_humble bash -lc 'cd /home/ncrl/docker_ubuntu22/px4_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select drone_px4_nav2_bridge drone_nav2_bringup'
Observe:
Both packages finish successfully.

[2] Terminal A: start the DDS agent.
Command:
docker compose exec ros2_humble bash -lc 'MicroXRCEAgent udp4 -p 8888'
Observe:
The agent prints "running..." on UDP port 8888.

[3] Terminal B: start one headless MAV1 in the arena.
Command:
docker compose exec ros2_humble bash -lc 'cd /home/ncrl/docker_ubuntu22/px4_ws/src/drone_nav2_apriltag && GZ_PARTITION=mav1_nav2_issue5 DRONES=1 HEADLESS=1 ./scripts/start_arena_sitl.sh'
Observe:
"MAV1 已連上 XRCE Agent" and the preflight-parameter success line appear.

[4] Terminal C: start the MAV1 mission profile in the same Gazebo partition.
Command:
docker compose exec ros2_humble bash -lc 'cd /home/ncrl/docker_ubuntu22/px4_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && GZ_PARTITION=mav1_nav2_issue5 ros2 launch drone_nav2_bringup mav1_offboard_mission.launch.py use_sim_time:=true'
Observe:
"MAV1 mission idle: Ready for a MAV1 Mission Goal" appears. No PX4 input is sent in idle.

[5] Terminal D: observe the public mission lifecycle.
Command:
docker compose exec ros2_humble bash -lc 'cd /home/ncrl/docker_ubuntu22/px4_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic echo /MAV1/mission_status'
Observe:
After the goal, the order is preflight, warmup, takeoff, navigate, land, then idle.

[6] Terminal E: send the separate map-frame Mission Goal.
Command:
docker compose exec ros2_humble bash -lc 'cd /home/ncrl/docker_ubuntu22/px4_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic pub --once /MAV1/mission_goal geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 1.0, y: -6.5, z: 0.0}, orientation: {w: 1.0}}}"'
Observe:
MAV1 arms only after warmup, reaches the ENU Flight Level 3.0 m, follows the Nav2 path, lands, and reports "idle: MAV1 landed; mission complete".

[7] Optional cancel check: send this while phase is takeoff or navigate.
Command:
docker compose exec ros2_humble bash -lc 'cd /home/ncrl/docker_ubuntu22/px4_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic pub --once /MAV1/mission_cancel std_msgs/msg/Empty "{}"'
Observe:
The lifecycle reports abort_hold before land. PX4 Offboard-loss remains PX4 failsafe authority.
EOF
