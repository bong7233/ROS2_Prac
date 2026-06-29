"""Twist multiplexer node.

Subscribes to several ``geometry_msgs/Twist`` input topics, each with a priority
and a timeout, and republishes the highest-priority recently-active one to a
single output topic (``/cmd_vel`` by default). This lets manual jog, docking, and
the waypoint follower coexist without fighting over the command.

The selection logic lives in :mod:`amr_twist_mux.twist_mux` (ROS-free and unit
tested); this node is the thin ROS wrapper.
"""
from __future__ import annotations

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node

from amr_twist_mux.twist_mux import TwistInput, select_command


class TwistMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("twist_mux")

        self.output_topic_ = str(self.declare_parameter("output_topic", "cmd_vel").value)
        self.publish_rate_hz_ = float(
            self.declare_parameter("publish_rate_hz", 30.0).value
        )
        names = list(self.declare_parameter("input_names", ["teleop"]).value)
        topics = list(
            self.declare_parameter("input_topics", ["cmd_vel_teleop"]).value
        )
        priorities = list(self.declare_parameter("input_priorities", [100]).value)
        timeouts = list(self.declare_parameter("input_timeouts", [0.5]).value)

        if not (len(names) == len(topics) == len(priorities) == len(timeouts)):
            raise RuntimeError(
                "input_names, input_topics, input_priorities, input_timeouts "
                "must have equal length"
            )
        if self.publish_rate_hz_ <= 0.0:
            raise RuntimeError("publish_rate_hz must be positive")

        self.inputs_ = [
            TwistInput(str(names[i]), int(priorities[i]), float(timeouts[i]))
            for i in range(len(names))
        ]
        self.subs_ = [
            self.create_subscription(
                Twist, str(topics[i]), self._make_callback(i), 10
            )
            for i in range(len(names))
        ]

        self.cmd_pub_ = self.create_publisher(Twist, self.output_topic_, 10)
        self.diagnostics_pub_ = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.timer_ = self.create_timer(
            1.0 / self.publish_rate_hz_, self.on_timer
        )

        self.get_logger().info(
            f"Twist mux started: {len(self.inputs_)} input(s) -> /{self.output_topic_}"
        )

    def _make_callback(self, index: int):
        def callback(msg: Twist) -> None:
            item = self.inputs_[index]
            item.linear_x = msg.linear.x
            item.angular_z = msg.angular.z
            item.stamp_s = self._now_s()
        return callback

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_timer(self) -> None:
        name, linear_x, angular_z = select_command(self.inputs_, self._now_s())

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub_.publish(twist)
        self.publish_diagnostics(name, linear_x, angular_z)

    def publish_diagnostics(self, active, linear_x, angular_z) -> None:
        status = DiagnosticStatus()
        status.name = "amr_twist_mux: mux"
        status.hardware_id = "twist_mux"
        status.level = DiagnosticStatus.OK
        status.message = f"active: {active}"
        status.values = [
            KeyValue(key="active_source", value=active),
            KeyValue(key="output_topic", value=self.output_topic_),
            KeyValue(key="linear_x", value=f"{linear_x:.3f}"),
            KeyValue(key="angular_z", value=f"{angular_z:.3f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_pub_.publish(array)


def main() -> None:
    rclpy.init()
    node = TwistMuxNode()
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
