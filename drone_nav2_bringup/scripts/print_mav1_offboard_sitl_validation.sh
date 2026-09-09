#!/usr/bin/env bash
# 印出 MAV1 Gazebo SITL 的手動驗證步驟。
#
# 前提：使用者已在 ros2_humble Docker 環境，且已執行：
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash

set -eu

cat <<'EOF'
MAV1 固定高度 Nav2 Offboard SITL 手動驗證
前提：你已在 ros2_humble Docker 環境，且已 source ROS 與 workspace。

[0] 任一終端：建置 Issue #5、#6 使用的兩個 package。
指令：
cd /home/ncrl/docker_ubuntu22/px4_ws
colcon build --packages-select drone_px4_nav2_bridge drone_nav2_bringup
觀察：
兩個 package 都成功完成建置。

[1] 終端 A：啟動 DDS Agent。
指令：
MicroXRCEAgent udp4 -p 8888
觀察：
Agent 顯示正在監聽 UDP port 8888。此終端必須保持執行。

[2] 終端 B：啟動一台 MAV1 的 Gazebo 與 PX4 SITL。
指令：
cd /home/ncrl/docker_ubuntu22/px4_ws/src/drone_nav2_apriltag
GZ_PARTITION=mav1_nav2_issue5 DRONES=1 HEADLESS=1 ./scripts/start_arena_sitl.sh
觀察：
MAV1 連線到 XRCE Agent，且 PX4 preflight parameter 設定成功。此終端必須保持執行。

[3] 終端 C：啟動 MAV1 Nav2 mission 與 RViz。
指令：
cd /home/ncrl/docker_ubuntu22/px4_ws
GZ_PARTITION=mav1_nav2_issue5 ros2 launch drone_nav2_bringup mav1_offboard_mission.launch.py use_sim_time:=true rviz:=true
觀察：
出現「MAV1 mission idle: Ready for a MAV1 Mission Goal」。RViz 載入
drone_nav2_bringup/rviz/mav1_offboard_mission.rviz，Fixed Frame 是 map；它顯示
static map、/MAV1/plan、MAV1 TF、arena walls 與 route graph。
這個 RViz 的唯一 2D Goal Pose 發布到 /MAV1/formation_goal_pose，會經過
Formation Goal Adapter 成為正式 Mission Goal。Plan-Only 使用另一個
mav1_plan_only.rviz，其 /MAV1/goal_pose 不會開始飛行。

[4] 終端 D：監控公開 mission lifecycle。
指令：
ros2 topic echo /MAV1/mission_status
觀察：
送出 Mission Goal 後，狀態順序為 preflight、warmup、takeoff、navigate、land、idle。

[5] 終端 E 或 RViz：選擇 Formation Goal。
指令：
ros2 topic pub --once /MAV1/formation_goal_node_id std_msgs/msg/Int32 "{data: 4}"
觀察：
此指令選擇 GeoJSON node 4，Formation Goal Adapter 解析其 map 座標
(1.0, -6.5) 後才發布 /MAV1/mission_goal。若要以座標選擇目的地，請在終端 C
的 RViz 使用 2D Goal Pose；它發布 /MAV1/formation_goal_pose。MAV1 在 warmup
完成後才 arm，起飛到 ENU Flight Level 3.0 m，依 Nav2 path 在固定高度飛行，
最後降落並顯示「idle: MAV1 landed; mission complete」。

[6] 所有終端：依序清理 runtime。
操作：
先在終端 C 按 Ctrl+C，停止 mission launch、RViz、Nav2、mission manager 與 bridge。
再在終端 B 按 Ctrl+C，停止 PX4 SITL 與 Gazebo。
再在終端 A 按 Ctrl+C，停止 DDS Agent。
最後在終端 D 與 E 按 Ctrl+C，停止監控或尚未完成的指令。
確認指令：
ros2 topic info /MAV1/mission_phase
ros2 topic info /MAV1/cmd_vel
觀察：
兩個確認指令都顯示「Unknown topic」，表示 MAV1 mission runtime 已清除。
EOF
