cmake_minimum_required(VERSION 3.8)
project(drone_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/formation_goal_adapter.py
    scripts/fixed_slot_follower_controller.py
    scripts/formation_mission_manager.py
    scripts/mav1_offboard_mission_manager.py
    scripts/plan_goal_bridge.py
    scripts/print_mav1_offboard_sitl_validation.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_plan_only_launch test/test_plan_only_launch.py
    ENV ROS_DOMAIN_ID=102
  )
  ament_add_pytest_test(
    test_plan_goal_bridge test/test_plan_goal_bridge.py
    ENV ROS_DOMAIN_ID=103
  )
  ament_add_pytest_test(
    test_plan_only_sim_time_launch test/test_plan_only_sim_time_launch.py
    ENV ROS_DOMAIN_ID=104
  )
  ament_add_pytest_test(
    test_control_only_launch test/test_control_only_launch.py
    ENV ROS_DOMAIN_ID=106
  )
  ament_add_pytest_test(
    test_mav1_offboard_mission_launch test/test_mav1_offboard_mission_launch.py
    ENV ROS_DOMAIN_ID=109
  )
  ament_add_pytest_test(
    test_formation_goal_adapter test/test_formation_goal_adapter.py
    ENV ROS_DOMAIN_ID=110
  )
  ament_add_pytest_test(
    test_fixed_slot_follower_controller test/test_fixed_slot_follower_controller.py
    ENV ROS_DOMAIN_ID=111
  )
  ament_add_pytest_test(
    test_fixed_slot_follower_controller_launch test/test_fixed_slot_follower_controller_launch.py
    ENV ROS_DOMAIN_ID=112
  )
  ament_add_pytest_test(
    test_formation_mission_launch test/test_formation_mission_launch.py
    ENV ROS_DOMAIN_ID=113
  )
endif()

ament_package()
