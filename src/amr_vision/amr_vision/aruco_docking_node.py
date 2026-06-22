"""ArUco docking detector node.

Subscribes to a camera ``Image`` (and optional ``CameraInfo`` for intrinsics),
detects the configured docking marker with OpenCV, and publishes an
``amr_interfaces/DockingState`` describing the base-frame docking error plus a
``diagnostic_msgs/DiagnosticArray`` consistent with the rest of the AMR stack.
An optional annotated debug image is published for RViz / rqt_image_view.

The detection math lives in :mod:`amr_vision.marker_docking` (ROS-free and unit
tested); this node is the thin ROS wrapper around it.
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from amr_interfaces.msg import DockingState
from amr_vision import aruco_compat, marker_docking


class ArucoDockingNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_docking")

        self.marker_id_ = int(self.declare_parameter("marker_id", 3).value)
        self.marker_length_m_ = float(
            self.declare_parameter("marker_length_m", 0.20).value
        )
        self.dictionary_name_ = str(
            self.declare_parameter("dictionary", aruco_compat.DEFAULT_DICTIONARY).value
        )
        self.publish_debug_image_ = bool(
            self.declare_parameter("publish_debug_image", True).value
        )
        self.default_hfov_deg_ = float(
            self.declare_parameter("default_horizontal_fov_deg", 70.0).value
        )
        self.max_range_m_ = float(self.declare_parameter("aligned_max_range_m", 0.6).value)
        self.min_range_m_ = float(self.declare_parameter("aligned_min_range_m", 0.15).value)
        self.max_lateral_m_ = float(
            self.declare_parameter("aligned_max_lateral_m", 0.03).value
        )
        self.max_bearing_rad_ = float(
            self.declare_parameter("aligned_max_bearing_rad", 0.05).value
        )

        self.dictionary_ = aruco_compat.get_dictionary(self.dictionary_name_)
        self.bridge_ = CvBridge()
        self.camera_matrix_ = None
        self.dist_coeffs_ = None

        self.docking_pub_ = self.create_publisher(DockingState, "docking_state", 10)
        self.diagnostics_pub_ = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.debug_image_pub_ = (
            self.create_publisher(Image, "docking_debug_image", 1)
            if self.publish_debug_image_
            else None
        )

        self.create_subscription(CameraInfo, "camera_info", self.on_camera_info, 10)
        self.create_subscription(Image, "image", self.on_image, 10)

        self.get_logger().info(
            f"ArUco docking detector started for marker {self.marker_id_}"
        )

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix_ = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # Some drivers publish an empty distortion vector; treat it as no
        # distortion so solvePnP receives a usable coefficient array.
        self.dist_coeffs_ = (
            np.array(msg.d, dtype=np.float64).reshape(-1, 1) if len(msg.d) else None
        )

    def on_image(self, msg: Image) -> None:
        image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="mono8")

        if self.camera_matrix_ is None:
            # No CameraInfo yet: fall back to intrinsics from image size + FOV.
            self.camera_matrix_ = marker_docking.default_camera_matrix(
                msg.width, msg.height, self.default_hfov_deg_
            )

        error = marker_docking.detect_docking_error(
            image,
            self.marker_id_,
            self.marker_length_m_,
            self.camera_matrix_,
            self.dist_coeffs_,
            self.dictionary_,
        )
        aligned = marker_docking.is_aligned(
            error,
            max_range_m=self.max_range_m_,
            min_range_m=self.min_range_m_,
            max_lateral_m=self.max_lateral_m_,
            max_bearing_rad=self.max_bearing_rad_,
        )

        self.publish_docking_state(msg.header, error, aligned)
        self.publish_diagnostics(error, aligned)
        if self.debug_image_pub_ is not None:
            self.publish_debug_image(image, msg.header, error, aligned)

    def publish_docking_state(self, header, error, aligned) -> None:
        state = DockingState()
        state.header = header
        state.detected = error.detected
        state.marker_id = error.marker_id
        state.range_m = error.range_m
        state.lateral_offset_m = error.lateral_offset_m
        state.bearing_rad = error.bearing_rad
        state.aligned = aligned
        self.docking_pub_.publish(state)

    def publish_diagnostics(self, error, aligned) -> None:
        status = DiagnosticStatus()
        status.name = "amr_vision: docking"
        status.hardware_id = "mock_dock_camera"
        if not error.detected:
            status.level = DiagnosticStatus.WARN
            status.message = "docking marker not detected"
        elif aligned:
            status.level = DiagnosticStatus.OK
            status.message = "docked and aligned"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "marker tracked"
        status.values = [
            KeyValue(key="detected", value=str(error.detected)),
            KeyValue(key="marker_id", value=str(error.marker_id)),
            KeyValue(key="range_m", value=f"{error.range_m:.3f}"),
            KeyValue(key="lateral_offset_m", value=f"{error.lateral_offset_m:.3f}"),
            KeyValue(key="bearing_rad", value=f"{error.bearing_rad:.3f}"),
            KeyValue(key="aligned", value=str(aligned)),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub_.publish(array)

    def publish_debug_image(self, gray, header, error, aligned) -> None:
        canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if error.detected:
            color = (0, 200, 0) if aligned else (0, 180, 220)
            text = (
                f"id {error.marker_id}  range {error.range_m:.2f} m  "
                f"lat {error.lateral_offset_m:+.2f} m  "
                f"{'ALIGNED' if aligned else 'tracking'}"
            )
        else:
            color = (60, 60, 220)
            text = "marker not detected"
        cv2.putText(
            canvas, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA
        )
        debug_msg = self.bridge_.cv2_to_imgmsg(canvas, encoding="bgr8")
        debug_msg.header = header
        self.debug_image_pub_.publish(debug_msg)


def main() -> None:
    rclpy.init()
    node = ArucoDockingNode()
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
