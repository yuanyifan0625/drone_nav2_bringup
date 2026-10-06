"""Launch one Leader Nav2 stack with two Fixed V-Slot Followers."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _follower_odometry_and_tf(
    *, bringup_share: str, vehicle_namespace: str, spawn_x: str, spawn_y: str
):
    """Return the minimal map-aligned odometry graph for a non-Nav2 follower."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_map_to_{vehicle_namespace.lower()}_odom",
            output="screen",
            arguments=[
                "--x", spawn_x, "--y", spawn_y, "--z", "0.0", "--yaw", "0.0",
                "--pitch", "0.0", "--roll", "0.0", "--frame-id", "map",
                "--child-frame-id", f"{vehicle_namespace}/odom",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="drone_px4_nav2_bridge",
            executable="px4_odometry_bridge",
            name="px4_odometry_bridge",
            namespace=vehicle_namespace,
            output="screen",
            parameters=[
                {
                    "px4_topic": f"/{vehicle_namespace}/fmu/out/vehicle_local_position_v1",
                    "odom_topic": f"/{vehicle_namespace}/odom",
                    "vehicle_prefix": vehicle_namespace,
                    "odom_frame": f"{vehicle_namespace}/odom",
                    "base_frame": f"{vehicle_namespace}/base_link",
                    "map_origin_x": float(spawn_x),
                    "map_origin_y": float(spawn_y),
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
    ]


def _px4_bridge(
    vehicle_namespace: str, target_system: int, cmd_vel_topic=None
) -> Node:
    """Create the only PX4 input authority for one vehicle."""
    return Node(
        package="drone_px4_nav2_bridge",
        executable="cmd_vel_to_px4_offboard_bridge",
        name="cmd_vel_to_px4_offboard_bridge",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            {
                "vehicle_namespace": vehicle_namespace,
                "target_system": target_system,
                "cmd_vel_topic": cmd_vel_topic or f"/{vehicle_namespace}/cmd_vel",
                "flight_level": LaunchConfiguration("flight_level"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )


def _follower_controller(vehicle_namespace: str, slot_forward, slot_left) -> Node:
    """Create one vehicle-scoped Fixed V-Slot controller."""
    return Node(
        package="drone_nav2_bringup",
        executable="fixed_slot_follower_controller.py",
        name="fixed_slot_follower_controller",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            {
                "vehicle_namespace": vehicle_namespace,
                "leader_namespace": "MAV1",
                "slot_forward": slot_forward,
                "slot_left": slot_left,
                "telemetry_timeout": LaunchConfiguration("telemetry_timeout"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )


def _follower_local_control(bringup_share: str, vehicle_namespace: str):
    """Return one follower's sole local controller, owner, and path adapter."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    fov_guard = LaunchConfiguration("fov_guard")
    fov_guard_enabled = PythonExpression(
        ["'", fov_guard, "'.lower() in ('true', '1')"]
    )
    mppi_parameters = RewrittenYaml(
        source_file=f"{bringup_share}/config/{vehicle_namespace.lower()}_follower_mppi.yaml",
        root_key=vehicle_namespace,
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )
    return [
        Node(
            package="nav2_controller", executable="controller_server",
            name="follower_mppi_controller_server", namespace=vehicle_namespace,
            output="screen", parameters=[mppi_parameters],
            remappings=[
                (
                    "cmd_vel",
                    PythonExpression(
                        ["'mppi_cmd_vel' if ", fov_guard_enabled, " else 'cmd_vel'"]
                    ),
                )
            ],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="follower_local_lifecycle_manager", namespace=vehicle_namespace,
            output="screen",
            parameters=[{"autostart": True,
                         "node_names": ["follower_mppi_controller_server"],
                         "use_sim_time": use_sim_time}],
        ),
        Node(
            package="drone_nav2_bringup", executable="follower_path_adapter.py",
            name="follower_path_adapter", namespace=vehicle_namespace, output="screen",
            parameters=[{
                "target_update_threshold": 0.10,
                "minimum_replacement_interval": 0.20,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="drone_nav2_bringup", executable="follower_fov_motion_guard.py",
            name="fov_motion_guard", namespace=vehicle_namespace, output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(fov_guard_enabled),
        ),
        Node(
            package="drone_nav2_bringup", executable="cooperative_obstacle_publisher.py",
            name="cooperative_obstacle_publisher", namespace=vehicle_namespace,
            output="screen",
            parameters=[{
                "vehicle_namespace": vehicle_namespace,
                "excluded_vehicles": ["MAV1"],
                "use_sim_time": use_sim_time,
            }],
        ),
    ]


def generate_launch_description():
    """Start MAV1 Nav2 and only odometry/follower control for MAV2/MAV3."""
    bringup_share = get_package_share_directory("drone_nav2_bringup")
    arena_share = get_package_share_directory("drone_nav2_apriltag")
    use_sim_time = LaunchConfiguration("use_sim_time")
    flight_level = LaunchConfiguration("flight_level")
    formation_center_enabled = LaunchConfiguration("formation_center_enabled")
    formation_envelope_radius = LaunchConfiguration("formation_envelope_radius")
    nav2_base_frame = PythonExpression(
        ["'MAV1/formation_center' if '", formation_center_enabled,
         "'.lower() in ('true', '1') else 'MAV1/base_link'"]
    )
    nav2_odom_topic = PythonExpression(
        ["'/MAV1/formation_center/map_odom' if '", formation_center_enabled,
         "'.lower() in ('true', '1') else '/MAV1/odom'"]
    )
    nav2_cmd_vel_topic = PythonExpression(
        ["'formation_center/cmd_vel' if '", formation_center_enabled,
         "'.lower() in ('true', '1') else 'cmd_vel'"]
    )
    bridge_cmd_vel_topic = PythonExpression(
        ["'/MAV1/formation_center/physical_cmd_vel' if '",
         formation_center_enabled,
         "'.lower() in ('true', '1') else '/MAV1/cmd_vel'"]
    )
    nav2_robot_radius = PythonExpression(
        [formation_envelope_radius, " if '", formation_center_enabled,
         "'.lower() in ('true', '1') else 0.4"]
    )
    nav2_inflation_radius = PythonExpression(
        [formation_envelope_radius, " if '", formation_center_enabled,
         "'.lower() in ('true', '1') else 0.75"]
    )
    leader_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{bringup_share}/launch/control_only.launch.py"
        ),
        launch_arguments={
            "vehicle_namespace": "MAV1",
            "vehicle_prefix": "MAV1",
            "px4_topic": "/MAV1/fmu/out/vehicle_local_position_v1",
            "odom_topic": "/MAV1/odom",
            "nav2_odom_topic": nav2_odom_topic,
            "robot_base_frame": nav2_base_frame,
            "robot_radius": nav2_robot_radius,
            "inflation_radius": nav2_inflation_radius,
            "cmd_vel_topic": nav2_cmd_vel_topic,
            "spawn_x": "-2.0",
            "spawn_y": "0.0",
            "use_sim_time": use_sim_time,
            "gazebo_clock_topic": LaunchConfiguration("gazebo_clock_topic"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": f"{bringup_share}/rviz/mav1_offboard_mission.rviz",
        }.items(),
    )
    depth_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{bringup_share}/launch/depth_sensors.launch.py"
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    manager = Node(
        package="drone_nav2_bringup",
        executable="formation_mission_manager.py",
        name="formation_mission_manager",
        output="screen",
        parameters=[
            f"{bringup_share}/config/formation_mission.yaml",
            {
                "flight_level": flight_level,
                "slot_tolerance": LaunchConfiguration("slot_tolerance"),
                "minimum_separation": LaunchConfiguration("minimum_separation"),
                "telemetry_timeout": LaunchConfiguration("telemetry_timeout"),
                "mav2_slot_forward": LaunchConfiguration("mav2_slot_forward"),
                "mav2_slot_left": LaunchConfiguration("mav2_slot_left"),
                "mav3_slot_forward": LaunchConfiguration("mav3_slot_forward"),
                "mav3_slot_left": LaunchConfiguration("mav3_slot_left"),
                "use_sim_time": use_sim_time,
            },
        ],
    )
    formation_center = Node(
        package="drone_nav2_bringup",
        executable="formation_center_state.py",
        name="formation_center_state",
        namespace="MAV1",
        output="screen",
        parameters=[
            {
                "mav2_slot_forward": LaunchConfiguration("mav2_slot_forward"),
                "mav2_slot_left": LaunchConfiguration("mav2_slot_left"),
                "mav3_slot_forward": LaunchConfiguration("mav3_slot_forward"),
                "mav3_slot_left": LaunchConfiguration("mav3_slot_left"),
                "vehicle_radius": LaunchConfiguration("formation_vehicle_radius"),
                "safety_margin": LaunchConfiguration("formation_safety_margin"),
                "envelope_rounding_increment": LaunchConfiguration(
                    "formation_envelope_rounding_increment"
                ),
                "envelope_radius": formation_envelope_radius,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(formation_center_enabled),
    )
    goal_adapter = Node(
        package="drone_nav2_bringup",
        executable="formation_goal_adapter.py",
        name="formation_goal_adapter",
        namespace="MAV1",
        output="screen",
        parameters=[
            {
                "vehicle_namespace": "MAV1",
                "map_frame": "map",
                "graph_file": f"{arena_share}/graphs/nav2_arena.geojson",
                "use_sim_time": use_sim_time,
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("flight_level", default_value="3.0"),
            DeclareLaunchArgument("mav2_slot_forward", default_value="-0.8"),
            DeclareLaunchArgument("mav2_slot_left", default_value="0.8"),
            DeclareLaunchArgument("mav3_slot_forward", default_value="-0.8"),
            DeclareLaunchArgument("mav3_slot_left", default_value="-0.8"),
            DeclareLaunchArgument("slot_tolerance", default_value="0.25"),
            DeclareLaunchArgument("minimum_separation", default_value="0.7"),
            DeclareLaunchArgument("telemetry_timeout", default_value="0.5"),
            DeclareLaunchArgument("fov_guard", default_value="false"),
            DeclareLaunchArgument("formation_center_enabled", default_value="true"),
            DeclareLaunchArgument("formation_envelope_radius", default_value="1.45"),
            DeclareLaunchArgument("formation_vehicle_radius", default_value="0.4"),
            DeclareLaunchArgument("formation_safety_margin", default_value="0.2"),
            DeclareLaunchArgument(
                "formation_envelope_rounding_increment", default_value="0.05"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "gazebo_clock_topic", default_value="/world/nav2_arena/clock"
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            leader_control,
            depth_sensors,
            *_follower_odometry_and_tf(
                bringup_share=bringup_share,
                vehicle_namespace="MAV2",
                spawn_x="-2.0",
                spawn_y="3.0",
            ),
            *_follower_odometry_and_tf(
                bringup_share=bringup_share,
                vehicle_namespace="MAV3",
                spawn_x="-2.0",
                spawn_y="-3.0",
            ),
            goal_adapter,
            manager,
            formation_center,
            _px4_bridge("MAV1", 1, bridge_cmd_vel_topic),
            _px4_bridge("MAV2", 2),
            _px4_bridge("MAV3", 3),
            _follower_controller(
                "MAV2",
                LaunchConfiguration("mav2_slot_forward"),
                LaunchConfiguration("mav2_slot_left"),
            ),
            _follower_controller(
                "MAV3",
                LaunchConfiguration("mav3_slot_forward"),
                LaunchConfiguration("mav3_slot_left"),
            ),
            *_follower_local_control(bringup_share, "MAV2"),
            *_follower_local_control(bringup_share, "MAV3"),
        ]
    )
