"""Mock camera that renders an ArUco docking marker, so the vision pipeline can
run with no real hardware.

This mirrors the rest of the stack's mock-first philosophy: the node publishes a
``sensor_msgs/Image`` and matching ``sensor_msgs/CameraInfo`` showing a single
marker placed in front of the robot. With ``approach_speed_mps`` the marker
slowly closes in, simulating the robot driving toward the dock so downstream
docking errors change over time.

Marker placement parameters are given in the robot base frame (x forward,
y left, z up) and converted to the camera optical frame for rendering.
"""
from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from amr_vision import aruco_compat, marker_docking


class MockDockCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_dock_camera")

        self.width_ = int(self.declare_parameter("image_width", 640).value)
        self.height_ = int(self.declare_parameter("image_height", 480).value)
        self.hfov_deg_ = float(self.declare_parameter("horizontal_fov_deg", 70.0).value)
        self.frame_id_ = str(
            self.declare_parameter("frame_id", "camera_optical_frame").value
        )
        self.publish_rate_hz_ = float(
            self.declare_parameter("publish_rate_hz", 15.0).value
        )
        self.marker_id_ = int(self.declare_parameter("marker_id", 3).value)
        self.marker_length_m_ = float(
            self.declare_parameter("marker_length_m", 0.20).value
        )
        self.dictionary_name_ = str(
            self.declare_parameter("dictionary", aruco_compat.DEFAULT_DICTIONARY).value
        )
        # Marker placement in the robot base frame.
        self.marker_forward_m_ = float(
            self.declare_parameter("marker_forward_m", 2.0).value
        )
        self.marker_left_m_ = float(self.declare_parameter("marker_left_m", 0.0).value)
        self.marker_up_m_ = float(self.declare_parameter("marker_up_m", 0.0).value)
        self.marker_yaw_deg_ = float(
            self.declare_parameter("marker_yaw_deg", 0.0).value
        )
        self.approach_speed_mps_ = float(
            self.declare_parameter("approach_speed_mps", 0.0).value
        )
        self.min_forward_m_ = float(self.declare_parameter("min_forward_m", 0.35).value)

        if self.publish_rate_hz_ <= 0.0:
            raise RuntimeError("publish_rate_hz must be positive")

        self.camera_matrix_ = marker_docking.default_camera_matrix(
            self.width_, self.height_, self.hfov_deg_
        )
        self.dictionary_ = aruco_compat.get_dictionary(self.dictionary_name_)
        self.bridge_ = CvBridge()

        self.image_pub_ = self.create_publisher(Image, "image", 10)
        self.camera_info_pub_ = self.create_publisher(CameraInfo, "camera_info", 10)
        self.timer_ = self.create_timer(
            1.0 / self.publish_rate_hz_, self.on_timer
        )

        self.get_logger().info(
            f"Mock dock camera started: marker {self.marker_id_} at "
            f"{self.marker_forward_m_:.2f} m forward"
        )

    def on_timer(self) -> None:
        if self.approach_speed_mps_ > 0.0:
            step = self.approach_speed_mps_ / self.publish_rate_hz_
            self.marker_forward_m_ = max(
                self.min_forward_m_, self.marker_forward_m_ - step
            )

        # Base frame (x forward, y left, z up) -> optical (x right, y down, z fwd).
        center_optical = (
            -self.marker_left_m_,
            -self.marker_up_m_,
            self.marker_forward_m_,
        )
        image = marker_docking.synthesize_marker_image(
            self.camera_matrix_,
            self.width_,
            self.height_,
            self.marker_id_,
            self.marker_length_m_,
            center_optical=center_optical,
            yaw_rad=np.deg2rad(self.marker_yaw_deg_),
            dictionary=self.dictionary_,
        )

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge_.cv2_to_imgmsg(image, encoding="mono8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id_
        self.image_pub_.publish(image_msg)
        self.camera_info_pub_.publish(self.build_camera_info(stamp))

    def build_camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id_
        info.width = self.width_
        info.height = self.height_
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        k = self.camera_matrix_.reshape(-1).tolist()
        info.k = k
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            k[0], k[1], k[2], 0.0,
            k[3], k[4], k[5], 0.0,
            k[6], k[7], k[8], 0.0,
        ]
        return info


def main() -> None:
    rclpy.init()
    node = MockDockCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
