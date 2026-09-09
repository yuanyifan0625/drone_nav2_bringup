"""Launch the MAV1 Gazebo-SITL Offboard mission profile."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start MAV1 Nav2 control, mission lifecycle and the sole PX4 interface."""

    bringup_share = get_package_share_directory("drone_nav2_bringup")
    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    vehicle_prefix = LaunchConfiguration("vehicle_prefix")
    flight_level = LaunchConfiguration("flight_level")
    px4_local_position_topic = LaunchConfiguration("px4_local_position_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument("vehicle_namespace", default_value="MAV1"),
        DeclareLaunchArgument("vehicle_prefix", default_value="MAV1"),
        DeclareLaunchArgument("flight_level", default_value="3.0"),
        DeclareLaunchArgument(
            "px4_local_position_topic",
            default_value="/MAV1/fmu/out/vehicle_local_position_v1",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "gazebo_clock_topic", default_value="/world/nav2_arena/clock"
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
    ]

    control_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{bringup_share}/launch/control_only.launch.py"
        ),
        launch_arguments={
            "vehicle_namespace": vehicle_namespace,
            "vehicle_prefix": vehicle_prefix,
            "px4_topic": px4_local_position_topic,
            "use_sim_time": use_sim_time,
            "gazebo_clock_topic": LaunchConfiguration("gazebo_clock_topic"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )
    mission_params = f"{bringup_share}/config/mav1_offboard_mission.yaml"
    mission_manager = Node(
        package="drone_nav2_bringup",
        executable="mav1_offboard_mission_manager.py",
        name="mav1_offboard_mission_manager",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            mission_params,
            {
                "vehicle_namespace": vehicle_namespace,
                "flight_level": flight_level,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    px4_interface = Node(
        package="drone_px4_nav2_bridge",
        executable="cmd_vel_to_px4_offboard_bridge",
        name="cmd_vel_to_px4_offboard_bridge",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            mission_params,
            {
                "vehicle_namespace": vehicle_namespace,
                "px4_local_position_topic": px4_local_position_topic,
                "flight_level": flight_level,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(arguments + [control_only, mission_manager, px4_interface])
