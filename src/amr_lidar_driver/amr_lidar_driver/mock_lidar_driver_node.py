"""Mock 2D LiDAR that publishes a ``sensor_msgs/LaserScan``.

A device simulator (per the project's "Python for device simulators" split) that
lets the stack produce ``/scan`` with no real sensor - useful for RViz and as a
Nav2 input while the real vendor driver is out of scope. The ray-casting model
lives in :mod:`amr_lidar_driver.scan_model` (ROS-free and unit tested); this node
is the thin ROS wrapper.

With ``use_odom`` the sensor moves through the modelled room as the robot drives,
so the scan changes with motion.
"""
from __future__ import annotations

import math
import random

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from amr_lidar_driver import scan_model


class MockLidarDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_lidar_driver")

        self.frame_id_ = str(self.declare_parameter("frame_id", "lidar_link").value)
        self.hardware_id_ = str(
            self.declare_parameter("hardware_id", "mock_lidar").value
        )
        self.publish_rate_hz_ = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )
        self.angle_min_ = float(self.declare_parameter("angle_min", -math.pi).value)
        self.angle_max_ = float(self.declare_parameter("angle_max", math.pi).value)
        self.samples_ = int(self.declare_parameter("samples", 360).value)
        self.range_min_ = float(self.declare_parameter("range_min", 0.08).value)
        self.range_max_ = float(self.declare_parameter("range_max", 8.0).value)
        self.room_half_x_ = float(self.declare_parameter("room_half_x", 5.0).value)
        self.room_half_y_ = float(self.declare_parameter("room_half_y", 5.0).value)
        # Flat [cx, cy, r, cx, cy, r, ...] list of circular obstacles.
        obstacle_values = list(
            self.declare_parameter("obstacles", [2.0, 1.0, 0.4]).value
        )
        self.noise_stddev_ = float(self.declare_parameter("noise_stddev", 0.01).value)
        self.use_odom_ = bool(self.declare_parameter("use_odom", False).value)

        if self.publish_rate_hz_ <= 0.0:
            raise RuntimeError("publish_rate_hz must be positive")
        if self.samples_ < 1:
            raise RuntimeError("samples must be >= 1")

        self.obstacles_ = [
            (obstacle_values[i], obstacle_values[i + 1], obstacle_values[i + 2])
            for i in range(0, len(obstacle_values) - 2, 3)
        ]
        self.angle_increment_ = (
            (self.angle_max_ - self.angle_min_) / (self.samples_ - 1)
            if self.samples_ > 1
            else 0.0
        )

        self.sensor_x_ = 0.0
        self.sensor_y_ = 0.0
        self.sensor_yaw_ = 0.0

        self.scan_pub_ = self.create_publisher(LaserScan, "scan", 10)
        self.diagnostics_pub_ = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        if self.use_odom_:
            self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.timer_ = self.create_timer(
            1.0 / self.publish_rate_hz_, self.on_timer
        )

        self.get_logger().info(
            f"Mock LiDAR started: {self.samples_} beams, "
            f"{len(self.obstacles_)} obstacle(s)"
        )

    def on_odom(self, msg: Odometry) -> None:
        self.sensor_x_ = msg.pose.pose.position.x
        self.sensor_y_ = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.sensor_yaw_ = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def on_timer(self) -> None:
        ranges = scan_model.compute_scan(
            self.sensor_x_,
            self.sensor_y_,
            self.sensor_yaw_,
            self.angle_min_,
            self.angle_max_,
            self.samples_,
            self.range_min_,
            self.range_max_,
            self.room_half_x_,
            self.room_half_y_,
            self.obstacles_,
        )
        if self.noise_stddev_ > 0.0:
            ranges = [
                min(self.range_max_, max(self.range_min_, r + random.gauss(0.0, self.noise_stddev_)))
                for r in ranges
            ]

        stamp = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.frame_id_
        scan.angle_min = self.angle_min_
        scan.angle_max = self.angle_max_
        scan.angle_increment = self.angle_increment_
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.publish_rate_hz_
        scan.range_min = self.range_min_
        scan.range_max = self.range_max_
        scan.ranges = [float(r) for r in ranges]
        scan.intensities = []
        self.scan_pub_.publish(scan)
        self.publish_diagnostics(ranges)

    def publish_diagnostics(self, ranges) -> None:
        closest = min(ranges) if ranges else float("nan")
        status = DiagnosticStatus()
        status.name = "amr_lidar_driver: mock_lidar"
        status.hardware_id = self.hardware_id_
        status.level = DiagnosticStatus.OK
        status.message = "scan publishing"
        status.values = [
            KeyValue(key="beams", value=str(len(ranges))),
            KeyValue(key="closest_range_m", value=f"{closest:.3f}"),
            KeyValue(key="range_max_m", value=f"{self.range_max_:.3f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub_.publish(array)


def main() -> None:
    rclpy.init()
    node = MockLidarDriverNode()
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
