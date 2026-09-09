"""Launch MAV1 native Nav2 control without a PX4 command interface."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Create the command-only MAV1 Nav2 control graph."""

    bringup_share = get_package_share_directory("drone_nav2_bringup")
    arena_share = get_package_share_directory("drone_nav2_apriltag")

    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    vehicle_prefix = LaunchConfiguration("vehicle_prefix")
    map_frame = LaunchConfiguration("map_frame")
    map_yaml = LaunchConfiguration("map_yaml")
    px4_topic = LaunchConfiguration("px4_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    planner_plugin = LaunchConfiguration("planner_plugin")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gazebo_clock_topic = LaunchConfiguration("gazebo_clock_topic")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    odom_frame = PythonExpression(["'", vehicle_prefix, "/odom'"])
    base_frame = PythonExpression(["'", vehicle_prefix, "/base_link'"])

    control_params = ParameterFile(
        RewrittenYaml(
            source_file=f"{bringup_share}/config/control_only.yaml",
            root_key=vehicle_namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "global_frame": map_frame,
                "robot_base_frame": base_frame,
                "odom_topic": odom_topic,
                "map_topic": "/map",
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    arguments = [
        DeclareLaunchArgument("vehicle_namespace", default_value="MAV1"),
        DeclareLaunchArgument("vehicle_prefix", default_value="MAV1"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument(
            "map_yaml", default_value=f"{arena_share}/maps/nav2_arena.yaml"
        ),
        DeclareLaunchArgument(
            "px4_topic",
            default_value="/MAV1/fmu/out/vehicle_local_position_v1",
        ),
        DeclareLaunchArgument("odom_topic", default_value="/MAV1/odom"),
        DeclareLaunchArgument(
            "planner_plugin", default_value="nav2_navfn_planner/NavfnPlanner"
        ),
        DeclareLaunchArgument("spawn_x", default_value="-2.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "gazebo_clock_topic", default_value="/world/nav2_arena/clock"
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config", default_value=f"{bringup_share}/rviz/mav1_plan_only.rviz"
        ),
    ]

    plan_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{bringup_share}/launch/plan_only.launch.py"),
        launch_arguments={
            "vehicle_namespace": vehicle_namespace,
            "vehicle_prefix": vehicle_prefix,
            "map_frame": map_frame,
            "map_yaml": map_yaml,
            "px4_topic": px4_topic,
            "odom_topic": odom_topic,
            "planner_plugin": planner_plugin,
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "use_sim_time": use_sim_time,
            "gazebo_clock_topic": gazebo_clock_topic,
            "rviz": rviz,
            "rviz_config": rviz_config,
        }.items(),
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[control_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            control_params,
            {
                "default_nav_to_pose_bt_xml": (
                    f"{bringup_share}/behavior_trees/"
                    "navigate_to_pose_control_only.xml"
                ),
                "default_nav_through_poses_bt_xml": (
                    f"{bringup_share}/behavior_trees/"
                    "navigate_through_poses_control_only.xml"
                ),
            },
        ],
    )

    control_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="control_lifecycle_manager",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["controller_server", "bt_navigator"],
            }
        ],
    )

    return LaunchDescription(
        arguments + [plan_only, controller_server, bt_navigator, control_lifecycle_manager]
    )
