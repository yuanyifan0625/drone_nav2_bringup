cmake_minimum_required(VERSION 3.8)
project(drone_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/plan_goal_bridge.py
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
endif()

ament_package()
