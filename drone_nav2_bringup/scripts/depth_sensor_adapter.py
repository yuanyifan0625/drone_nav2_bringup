#!/usr/bin/env python3
"""Normalize one Gazebo depth camera into a vehicle-scoped ROS interface."""

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class DepthSensorAdapter(Node):
    """Relay sensor data while assigning the vehicle-scoped camera frame."""

    def __init__(self) -> None:
        super().__init__("depth_sensor_adapter")
        frame_id = self.declare_parameter("frame_id", "MAV1/depth_camera_link").value
        image_input = self.declare_parameter("image_input", "depth/raw/image").value
        camera_info_input = self.declare_parameter(
            "camera_info_input", "depth/raw/camera_info"
        ).value
        points_input = self.declare_parameter("points_input", "depth/raw/points").value

        self._frame_id = str(frame_id)
        self._image_publisher = self.create_publisher(
            Image, "depth/image_raw", qos_profile_sensor_data
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, "depth/camera_info", qos_profile_sensor_data
        )
        self._points_publisher = self.create_publisher(
            PointCloud2, "depth/points", qos_profile_sensor_data
        )
        self.create_subscription(
            Image, str(image_input), self._relay_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            str(camera_info_input),
            self._relay_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, str(points_input), self._relay_points, qos_profile_sensor_data
        )

    def _relay_image(self, message: Image) -> None:
        message.header.frame_id = self._frame_id
        self._image_publisher.publish(message)

    def _relay_camera_info(self, message: CameraInfo) -> None:
        message.header.frame_id = self._frame_id
        self._camera_info_publisher.publish(message)

    def _relay_points(self, message: PointCloud2) -> None:
        message.header.frame_id = self._frame_id
        self._points_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = DepthSensorAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
