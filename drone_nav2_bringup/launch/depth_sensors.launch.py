"""Bridge PX4 Gazebo x500_depth sensors into stable vehicle ROS topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORLD = "nav2_arena"
SENSOR = "StereoOV7251"
# Keep this combined sensor pose in sync with gz/models/x500_depth/model.sdf.
CAMERA_POSE = ("0.13233", "0.0", "0.26078")


def _gazebo_topic(model_name: str, suffix: str) -> str:
    return (
        f"/world/{WORLD}/model/{model_name}/link/camera_link/"
        f"sensor/{SENSOR}/{suffix}"
    )


def _vehicle_depth_nodes(vehicle_namespace: str, model_name: str):
    image_topic = _gazebo_topic(model_name, "depth_image")
    camera_info_topic = _gazebo_topic(model_name, "camera_info")
    points_topic = _gazebo_topic(model_name, "depth_image/points")
    frame_id = f"{vehicle_namespace}/depth_camera_link"
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="depth_gazebo_bridge",
            namespace=vehicle_namespace,
            output="screen",
            arguments=[
                f"{image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
                f"{camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                f"{points_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ],
            remappings=[
                (image_topic, "depth/raw/image"),
                (camera_info_topic, "depth/raw/camera_info"),
                (points_topic, "depth/raw/points"),
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="drone_nav2_bringup",
            executable="depth_sensor_adapter.py",
            name="depth_sensor_adapter",
            namespace=vehicle_namespace,
            output="screen",
            parameters=[
                {
                    "frame_id": frame_id,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{vehicle_namespace.lower()}_depth_camera_tf",
            output="screen",
            arguments=[
                "--x", CAMERA_POSE[0], "--y", CAMERA_POSE[1], "--z", CAMERA_POSE[2],
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", f"{vehicle_namespace}/base_link",
                "--child-frame-id", frame_id,
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ]


def generate_launch_description():
    """Expose depth image, camera info, points, and TF for all three vehicles."""

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            *_vehicle_depth_nodes("MAV1", "x500_depth_0"),
            *_vehicle_depth_nodes("MAV2", "x500_depth_1"),
            *_vehicle_depth_nodes("MAV3", "x500_depth_2"),
        ]
    )
