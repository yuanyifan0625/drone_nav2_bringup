"""Launch MAV1 Static Occupancy Map and Nav2 global planning only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Create the MAV1 Plan-Only graph without vehicle-control components."""

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
    odom_frame = PythonExpression(["'", vehicle_prefix, "/odom'"])
    base_frame = PythonExpression(["'", vehicle_prefix, "/base_link'"])

    params_file = f"{bringup_share}/config/plan_only.yaml"
    planner_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=vehicle_namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "global_frame": map_frame,
                "robot_base_frame": base_frame,
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
            "map_yaml",
            default_value=f"{arena_share}/maps/nav2_arena.yaml",
        ),
        DeclareLaunchArgument(
            "px4_topic",
            default_value="/MAV1/fmu/out/vehicle_local_position_v1",
        ),
        DeclareLaunchArgument("odom_topic", default_value="/MAV1/odom"),
        DeclareLaunchArgument(
            "planner_plugin",
            default_value="nav2_navfn_planner/NavfnPlanner",
        ),
        DeclareLaunchArgument("spawn_x", default_value="-2.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=f"{bringup_share}/rviz/mav1_plan_only.rviz",
        ),
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
                "topic_name": "/map",
                "frame_id": map_frame,
            }
        ],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="map_lifecycle_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=[
            "--x",
            LaunchConfiguration("spawn_x"),
            "--y",
            LaunchConfiguration("spawn_y"),
            "--z",
            LaunchConfiguration("spawn_z"),
            "--yaw",
            "0.0",
            "--pitch",
            "0.0",
            "--roll",
            "0.0",
            "--frame-id",
            map_frame,
            "--child-frame-id",
            odom_frame,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    px4_odometry_bridge = Node(
        package="drone_px4_nav2_bridge",
        executable="px4_odometry_bridge",
        name="px4_odometry_bridge",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            {
                "px4_topic": px4_topic,
                "odom_topic": odom_topic,
                "vehicle_prefix": vehicle_prefix,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            planner_params,
            {
                "GridBased.plugin": planner_plugin,
            },
        ],
    )

    planner_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="planner_lifecycle_manager",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["planner_server"],
            }
        ],
    )

    plan_goal_bridge = Node(
        package="drone_nav2_bringup",
        executable="plan_goal_bridge.py",
        name="plan_goal_bridge",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "vehicle_prefix": vehicle_prefix,
                "map_frame": map_frame,
                "base_frame": base_frame,
                "goal_topic": PythonExpression(["'/", vehicle_prefix, "/goal_pose'"]),
                "planner_action": PythonExpression(
                    ["'/", vehicle_prefix, "/compute_path_to_pose'"]
                ),
            }
        ],
    )

    arena_markers = Node(
        package="drone_nav2_apriltag",
        executable="graph_markers.py",
        name="graph_markers",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "world_file": f"{arena_share}/gz/worlds/nav2_arena.sdf",
                "graph_file": f"{arena_share}/graphs/nav2_arena.geojson",
                "frame_id": map_frame,
                "px4_namespace": PythonExpression(["'/", vehicle_prefix, "'"]),
                "track_vehicle": True,
                "origin_x": ParameterValue(
                    LaunchConfiguration("spawn_x"), value_type=float
                ),
                "origin_y": ParameterValue(
                    LaunchConfiguration("spawn_y"), value_type=float
                ),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(
                msg=[
                    "Plan-Only uses a temporary static alignment ",
                    map_frame,
                    " -> ",
                    odom_frame,
                    " at ENU spawn (",
                    LaunchConfiguration("spawn_x"),
                    ", ",
                    LaunchConfiguration("spawn_y"),
                    ", ",
                    LaunchConfiguration("spawn_z"),
                    "). Replace this publisher with SLAM or AprilTag localization later.",
                ]
            ),
            map_server,
            map_lifecycle_manager,
            static_map_to_odom,
            px4_odometry_bridge,
            planner_server,
            planner_lifecycle_manager,
            plan_goal_bridge,
            arena_markers,
            rviz,
        ]
    )
