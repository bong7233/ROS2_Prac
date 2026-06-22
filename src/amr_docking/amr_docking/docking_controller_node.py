"""Docking controller node.

Subscribes to ``amr_interfaces/DockingState`` (from ``amr_vision``) and drives
``/cmd_vel`` to align with and approach a docking marker. The command goes
through the same ``/cmd_vel`` entry point as manual jog and Nav2, so the safety
monitor still gates it.

The control law lives in :mod:`amr_docking.docking_controller` (ROS-free and
unit tested); this node is the thin ROS wrapper: it owns enable/disable,
docking-state staleness handling, the publish loop, and diagnostics.
"""
from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool

from amr_interfaces.msg import DockingState
from amr_docking.docking_controller import (
    DockingControllerConfig,
    compute_docking_command,
)


class DockingControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("docking_controller")

        self.control_rate_hz_ = float(
            self.declare_parameter("control_rate_hz", 20.0).value
        )
        self.command_timeout_ms_ = float(
            self.declare_parameter("command_timeout_ms", 500.0).value
        )
        self.enabled_ = bool(self.declare_parameter("auto_start", False).value)

        self.config_ = DockingControllerConfig(
            max_linear_mps=float(self.declare_parameter("max_linear_mps", 0.15).value),
            max_angular_radps=float(
                self.declare_parameter("max_angular_radps", 0.6).value
            ),
            range_gain=float(self.declare_parameter("range_gain", 0.3).value),
            bearing_gain=float(self.declare_parameter("bearing_gain", 1.2).value),
            align_first_bearing_rad=float(
                self.declare_parameter("align_first_bearing_rad", 0.20).value
            ),
            search_yaw_rate_radps=float(
                self.declare_parameter("search_yaw_rate_radps", 0.0).value
            ),
        )

        if self.control_rate_hz_ <= 0.0:
            raise RuntimeError("control_rate_hz must be positive")

        self.latest_state_ = None
        self.last_state_time_ = None
        self.last_phase_ = "IDLE"

        self.cmd_pub_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.diagnostics_pub_ = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_subscription(
            DockingState, "docking_state", self.on_docking_state, 10
        )
        self.enable_srv_ = self.create_service(
            SetBool, "enable_docking", self.on_enable
        )
        self.timer_ = self.create_timer(
            1.0 / self.control_rate_hz_, self.on_timer
        )

        self.get_logger().info(
            f"Docking controller started (auto_start={self.enabled_})"
        )

    def on_docking_state(self, msg: DockingState) -> None:
        self.latest_state_ = msg
        self.last_state_time_ = self.get_clock().now()

    def on_enable(self, request, response):
        self.enabled_ = request.data
        if not self.enabled_:
            self.cmd_pub_.publish(Twist())  # stop the robot on disable
        response.success = True
        response.message = "docking enabled" if self.enabled_ else "docking disabled"
        self.get_logger().info(response.message)
        return response

    def on_timer(self) -> None:
        if not self.enabled_:
            return

        if self.latest_state_ is None or self.state_is_stale():
            # No fresh perception: stop and wait rather than drive blind.
            self.cmd_pub_.publish(Twist())
            self.publish_status("STALE", 0.0, 0.0)
            return

        state = self.latest_state_
        command = compute_docking_command(
            detected=state.detected,
            range_m=state.range_m,
            lateral_offset_m=state.lateral_offset_m,
            bearing_rad=state.bearing_rad,
            aligned=state.aligned,
            config=self.config_,
        )

        twist = Twist()
        twist.linear.x = command.linear_x
        twist.angular.z = command.angular_z
        self.cmd_pub_.publish(twist)

        if command.phase != self.last_phase_:
            self.get_logger().info(f"docking phase: {command.phase}")
            self.last_phase_ = command.phase
        self.publish_status(command.phase, command.linear_x, command.angular_z)

    def state_is_stale(self) -> bool:
        if self.last_state_time_ is None:
            return True
        age_ms = (self.get_clock().now() - self.last_state_time_).nanoseconds / 1e6
        return age_ms > self.command_timeout_ms_

    def publish_status(self, phase: str, linear_x: float, angular_z: float) -> None:
        status = DiagnosticStatus()
        status.name = "amr_docking: controller"
        status.hardware_id = "docking_controller"
        status.level = (
            DiagnosticStatus.WARN if phase in ("STALE", "SEARCH") else DiagnosticStatus.OK
        )
        status.message = phase
        status.values = [
            KeyValue(key="enabled", value=str(self.enabled_)),
            KeyValue(key="phase", value=phase),
            KeyValue(key="cmd_linear_x", value=f"{linear_x:.3f}"),
            KeyValue(key="cmd_angular_z", value=f"{angular_z:.3f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub_.publish(array)


def main() -> None:
    rclpy.init()
    node = DockingControllerNode()
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
