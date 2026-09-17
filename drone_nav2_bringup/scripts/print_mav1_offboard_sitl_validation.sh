#!/usr/bin/env bash
# 印出 Issue #7 三機 Gazebo SITL 的手動驗證步驟。
#
# 前提：使用者已在 ros2_humble Docker 環境，且已執行：
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash

set -eu

cat <<'EOF'
Issue #7 三機 Fixed V-Slot Formation Mission SITL 手動驗證
前提：你已在 ros2_humble Docker 環境，且已 source ROS 與 workspace。

[0] 任一終端：建置本次驗證使用的 packages。
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

[2] 終端 B：啟動三台 Gazebo 與 PX4 SITL。
指令：
cd /home/ncrl/docker_ubuntu22/px4_ws/src/drone_nav2_apriltag
GZ_PARTITION=issue7_manual_validation DRONES=3 ./scripts/start_arena_sitl.sh
觀察：
MAV1、MAV2、MAV3 都連線到 XRCE Agent，且三架的 PX4 preflight parameter
設定成功。此終端必須保持執行。

[3] 終端 C：啟動 Formation Mission；MAV1 有 Nav2，MAV2/MAV3 只跑 followers。
指令：
cd /home/ncrl/docker_ubuntu22/px4_ws
GZ_PARTITION=issue7_manual_validation ros2 launch drone_nav2_bringup formation_mission.launch.py use_sim_time:=true rviz:=true
觀察：
出現「Formation mission idle: Ready for a Formation Mission Goal」。確認：
ros2 lifecycle get /MAV1/controller_server 顯示 active [3]；
ros2 lifecycle get /MAV2/follower_mppi_controller_server 與
ros2 lifecycle get /MAV3/follower_mppi_controller_server 都顯示 active [3]；
/MAV2/cmd_vel 與 /MAV3/cmd_vel 都各有一個 publisher，且皆為各自的
follower_mppi_controller_server；PX4 bridge 是各 topic 唯一 subscriber。
ros2 topic echo --once /MAV2/cooperative_obstacles 與
ros2 topic echo --once /MAV3/cooperative_obstacles 可確認各 follower 有 peer odometry obstacle 點雲。
另外確認 /MAV2/depth/raw/points 的 frame_id 是 camera_link，且
/MAV2/depth/points 的 frame_id 是 MAV2/depth_camera_link；兩者 camera pose 都是
base_link 前方 0.13233 m、上方 0.26078 m，避免錯誤 frame 造成靜態障礙漂移。
RViz 載入 drone_nav2_bringup/rviz/mav1_offboard_mission.rviz，Fixed Frame 是 map；
它顯示 static map、/MAV1/plan、MAV1 TF、arena walls 與 route graph。
這個 RViz 的唯一 2D Goal Pose 發布到 /MAV1/formation_goal_pose，會經過
Formation Goal Adapter 成為正式 Mission Goal。Plan-Only 使用另一個
mav1_plan_only.rviz，其 /MAV1/goal_pose 不會開始飛行。

[4] 終端 D：監控公開 Formation Mission lifecycle。
指令：
ros2 topic echo /MAV1/mission_status
觀察：
正常路徑應依序出現 preflight、warmup、takeoff_all、form_up、navigate_leader、
land_all、idle。另開終端可觀察 follower 控制：
ros2 topic echo /MAV2/cmd_vel
ros2 topic echo /MAV3/cmd_vel

[4a] 終端 E：建立 MAV2 預期 slot 路徑上的單一靜態障礙。
指令（此例放在通往 node 9 前、約 (18.9, 9.0) 的 MAV2 左後 slot；每次驗證前只建立一次）：
gz service -s /world/nav2_arena/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 --req 'sdf: "<sdf version=\"1.9\"><model name=\"mav2_slot_obstacle\"><static>true</static><pose>18.9 9.0 1.5 0 0 0</pose><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.5 0.5 3.0</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.5 0.5 3.0</size></box></geometry></visual></link></model></sdf>"'
觀察：/MAV2/depth/points 可看到障礙；MPPI 繞行後回到 slot，且未觸發 abort_hold。


[5] 終端 E 或 RViz：選擇一個 Formation Goal。每次任務完成、狀態回到 idle 後，
才可送下一個目標。
方式 A（GeoJSON node ID，重送三次可避開 DDS discovery 時序）：
ros2 topic pub --times 3 -r 2 /MAV1/formation_goal_node_id std_msgs/msg/Int32 "{data: 9}"
方式 B（直接模擬 RViz 的座標 Goal；不開 GUI 時使用）：
ros2 topic pub --once /MAV1/formation_goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 6.5, y: -6.5, z: 0.0}, orientation: {w: 1.0}}}"
方式 C（RViz）：
在終端 C 開啟的 RViz 工具列選擇唯一的「2D Goal Pose」，再於地圖上點選座標；
它會發布 /MAV1/formation_goal_pose。
觀察：
方式 A 的 node 9 由 Formation Goal Adapter 解析為 map 座標 (25.0, 14.0)，方式 B
與 C 則直接使用 map 座標；三者都只會由 Adapter 發布正式的 /MAV1/mission_goal。
合格條件是三架到 ENU Flight Level 3.0 m、MAV2/MAV3 在 MAV1 yaw-relative 固定 V-slot
收斂、MAV1 成功導航，最後三架 LandAll。若看到 abort_hold，記錄 status 的具體原因，
它代表安全 gate 正確拒絕正常完成。

歷史驗證（2026-09-09、Issue 13 前）曾出現 MAV1/MAV3 最小距離 0.60 m 而觸發
abort_hold；完成 MAV3 MPPI 與 Cooperative Obstacle 後，必須以本流程重新驗證，不能將
abort_hold 視為正常完成。

[6] 所有終端：依序清理 runtime。
操作：
先在終端 C 按 Ctrl+C，停止 Formation launch、RViz、Nav2、manager、followers 與 bridges。
再在終端 B 按 Ctrl+C，停止 PX4 SITL 與 Gazebo。
再在終端 A 按 Ctrl+C，停止 DDS Agent。
最後在終端 D 與 E 按 Ctrl+C，停止監控或尚未完成的指令。
若終端 C 的 Ctrl+C 因背景執行而沒有停止 launch，先找出 launch 父程序，再只對該
父程序送出 SIGINT（不要逐一殺 controller_server 或 bt_navigator）：
pgrep -af "ros2 launch drone_nav2_bringup formation_mission"
kill -INT <上一步顯示的 launch_PID>
確認指令：
ros2 topic info /MAV1/mission_phase
ros2 topic info /MAV2/cmd_vel
ros2 topic info /MAV3/cmd_vel
觀察：
三個確認指令都顯示「Unknown topic」，表示 Formation Mission runtime 已清除。
pgrep -af '[M]icroXRCEAgent|[b]uild/px4_sitl_default/bin/px4|[g]z sim|[g]zserver|[r]os2 (launch|run)'
EOF
