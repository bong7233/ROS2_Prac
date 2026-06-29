"""Waypoint follower node.

Drives the robot through a list of odom-frame waypoints by publishing ``/cmd_vel``
(through the usual safety-monitor path). Exercises the ``AUTO_RUNNING`` style of
operation without a full Nav2 stack.

The control law lives in :mod:`amr_navigation.waypoint_follower` (ROS-free and
unit tested); this node is the thin ROS wrapper handling enable/disable, odom
staleness, looping, and diagnostics.
"""
from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import SetBool

from amr_navigation.waypoint_follower import (
    WaypointFollowerConfig,
    follow_waypoints,
)


class WaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        self.control_rate_hz_ = float(
            self.declare_parameter("control_rate_hz", 20.0).value
        )
        self.command_timeout_ms_ = float(
            self.declare_parameter("command_timeout_ms", 500.0).value
        )
        self.enabled_ = bool(self.declare_parameter("auto_start", False).value)
        self.loop_ = bool(self.declare_parameter("loop", False).value)

        self.config_ = WaypointFollowerConfig(
            max_linear_mps=float(self.declare_parameter("max_linear_mps", 0.4).value),
            max_angular_radps=float(
                self.declare_parameter("max_angular_radps", 1.0).value
            ),
            heading_gain=float(self.declare_parameter("heading_gain", 1.5).value),
            distance_gain=float(self.declare_parameter("distance_gain", 0.8).value),
            waypoint_tolerance_m=float(
                self.declare_parameter("waypoint_tolerance_m", 0.15).value
            ),
            turn_in_place_rad=float(
                self.declare_parameter("turn_in_place_rad", 0.5).value
            ),
        )

        # Flat [x0, y0, x1, y1, ...] list of odom-frame waypoints.
        flat = list(self.declare_parameter("waypoints", [1.0, 0.0, 1.0, 1.0]).value)
        self.waypoints_ = [
            (flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)
        ]

        if self.control_rate_hz_ <= 0.0:
            raise RuntimeError("control_rate_hz must be positive")

        self.target_index_ = 0
        self.have_odom_ = False
        self.last_odom_time_ = None
        self.robot_x_ = 0.0
        self.robot_y_ = 0.0
        self.robot_yaw_ = 0.0
        self.last_phase_ = "IDLE"

        self.cmd_pub_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.diagnostics_pub_ = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.enable_srv_ = self.create_service(
            SetBool, "enable_navigation", self.on_enable
        )
        self.timer_ = self.create_timer(
            1.0 / self.control_rate_hz_, self.on_timer
        )

        self.get_logger().info(
            f"Waypoint follower started: {len(self.waypoints_)} waypoint(s), "
            f"auto_start={self.enabled_}, loop={self.loop_}"
        )

    def on_odom(self, msg: Odometry) -> None:
        self.have_odom_ = True
        self.last_odom_time_ = self.get_clock().now()
        self.robot_x_ = msg.pose.pose.position.x
        self.robot_y_ = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw_ = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def on_enable(self, request, response):
        self.enabled_ = request.data
        if self.enabled_:
            self.target_index_ = 0  # restart the route when (re)enabled
        else:
            self.cmd_pub_.publish(Twist())
        response.success = True
        response.message = (
            "navigation enabled" if self.enabled_ else "navigation disabled"
        )
        self.get_logger().info(response.message)
        return response

    def odom_is_stale(self) -> bool:
        if self.last_odom_time_ is None:
            return True
        age_ms = (self.get_clock().now() - self.last_odom_time_).nanoseconds / 1e6
        return age_ms > self.command_timeout_ms_

    def on_timer(self) -> None:
        if not self.enabled_:
            return

        if not self.have_odom_ or self.odom_is_stale():
            self.cmd_pub_.publish(Twist())
            self.publish_status("STALE", 0.0, 0.0, 0.0)
            return

        command = follow_waypoints(
            self.robot_x_,
            self.robot_y_,
            self.robot_yaw_,
            self.waypoints_,
            self.target_index_,
            self.config_,
        )
        self.target_index_ = command.target_index

        if command.done:
            self.cmd_pub_.publish(Twist())
            self.publish_status("DONE", 0.0, 0.0, 0.0)
            if self.loop_:
                self.target_index_ = 0
            else:
                self.enabled_ = False
            return

        twist = Twist()
        twist.linear.x = command.linear_x
        twist.angular.z = command.angular_z
        self.cmd_pub_.publish(twist)
        self.publish_status(
            "FOLLOWING", command.linear_x, command.angular_z, command.distance_to_target
        )

    def publish_status(self, phase, linear_x, angular_z, distance) -> None:
        if phase != self.last_phase_:
            self.get_logger().info(f"navigation phase: {phase}")
            self.last_phase_ = phase
        status = DiagnosticStatus()
        status.name = "amr_navigation: waypoint_follower"
        status.hardware_id = "waypoint_follower"
        status.level = (
            DiagnosticStatus.WARN if phase == "STALE" else DiagnosticStatus.OK
        )
        status.message = phase
        status.values = [
            KeyValue(key="enabled", value=str(self.enabled_)),
            KeyValue(key="phase", value=phase),
            KeyValue(key="target_index", value=str(self.target_index_)),
            KeyValue(key="waypoint_count", value=str(len(self.waypoints_))),
            KeyValue(key="distance_to_target", value=f"{distance:.3f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub_.publish(array)


def main() -> None:
    rclpy.init()
    node = WaypointFollowerNode()
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
