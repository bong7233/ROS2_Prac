"""Mission coordinator node: battery-aware patrol/charge autonomy.

Watches the battery and dock status and enables/disables the waypoint follower
and the docking controller (via their ``std_srvs/SetBool`` services) to run a
simple mission: patrol until low, return to the dock and charge, then resume.

The decision logic lives in :mod:`amr_mission.mission_coordinator` (ROS-free and
unit tested); this node is the thin ROS wrapper that watches topics and drives
the controllers' enable services.
"""
from __future__ import annotations

import rclpy
from amr_interfaces.msg import RobotState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool

from amr_mission.mission_coordinator import PATROL, MissionConfig, decide


class MissionCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_coordinator")

        self.update_rate_hz_ = float(
            self.declare_parameter("update_rate_hz", 2.0).value
        )
        self.enabled_ = bool(self.declare_parameter("auto_start", False).value)
        self.config_ = MissionConfig(
            low_battery_pct=float(
                self.declare_parameter("low_battery_pct", 0.30).value
            ),
            full_battery_pct=float(
                self.declare_parameter("full_battery_pct", 0.90).value
            ),
        )

        if self.update_rate_hz_ <= 0.0:
            raise RuntimeError("update_rate_hz must be positive")

        self.state_ = PATROL
        self.have_battery_ = False
        self.battery_pct_ = 1.0
        self.docked_ = False
        self.sent_nav_ = None  # last enable value actually sent
        self.sent_dock_ = None

        self.create_subscription(BatteryState, "battery_state", self.on_battery, 10)
        self.create_subscription(RobotState, "robot_state", self.on_robot_state, 10)
        self.nav_client_ = self.create_client(SetBool, "enable_navigation")
        self.dock_client_ = self.create_client(SetBool, "enable_docking")
        self.diagnostics_pub_ = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.timer_ = self.create_timer(
            1.0 / self.update_rate_hz_, self.on_timer
        )

        self.get_logger().info(
            f"Mission coordinator started (auto_start={self.enabled_})"
        )

    def on_battery(self, msg: BatteryState) -> None:
        if msg.percentage == msg.percentage:  # not NaN
            self.battery_pct_ = msg.percentage
            self.have_battery_ = True

    def on_robot_state(self, msg: RobotState) -> None:
        self.docked_ = msg.docked

    def on_timer(self) -> None:
        if not self.enabled_ or not self.have_battery_:
            return

        decision = decide(self.state_, self.battery_pct_, self.docked_, self.config_)
        if decision.state != self.state_:
            self.get_logger().info(
                f"mission: {self.state_} -> {decision.state} "
                f"(battery {self.battery_pct_ * 100.0:.0f}%, docked={self.docked_})"
            )
            self.state_ = decision.state

        self._send_enable(self.nav_client_, "sent_nav_", decision.enable_navigation)
        self._send_enable(self.dock_client_, "sent_dock_", decision.enable_docking)
        self.publish_diagnostics(decision)

    def _send_enable(self, client, attr: str, value: bool) -> None:
        # Only call the service when the desired value changes and the server is up.
        if getattr(self, attr) == value:
            return
        if not client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = value
        client.call_async(request)
        setattr(self, attr, value)

    def publish_diagnostics(self, decision) -> None:
        status = DiagnosticStatus()
        status.name = "amr_mission: coordinator"
        status.hardware_id = "mission_coordinator"
        status.level = DiagnosticStatus.OK
        status.message = decision.state
        status.values = [
            KeyValue(key="state", value=decision.state),
            KeyValue(key="battery_pct", value=f"{self.battery_pct_:.2f}"),
            KeyValue(key="docked", value=str(self.docked_)),
            KeyValue(key="enable_navigation", value=str(decision.enable_navigation)),
            KeyValue(key="enable_docking", value=str(decision.enable_docking)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub_.publish(array)


def main() -> None:
    rclpy.init()
    node = MissionCoordinatorNode()
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
