import argparse
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import rclpy
from amr_interfaces.msg import IoState, MotorState, RobotState, SafetyState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState


MODE_NAMES = {
    RobotState.MODE_BOOT: "BOOT",
    RobotState.MODE_INIT: "INIT",
    RobotState.MODE_MANUAL: "MANUAL",
    RobotState.MODE_AUTO_READY: "AUTO_READY",
    RobotState.MODE_AUTO_RUNNING: "AUTO_RUNNING",
    RobotState.MODE_PAUSED: "PAUSED",
    RobotState.MODE_CHARGING: "CHARGING",
    RobotState.MODE_FAULT: "FAULT",
    RobotState.MODE_ESTOP: "ESTOP",
}


@dataclass
class Sample:
    message: Any
    received_monotonic: float


class HealthReport(Node):
    def __init__(self) -> None:
        super().__init__("amr_health_report")
        self.samples: Dict[str, Sample] = {}

        self.create_subscription(BatteryState, "battery_state", self._store("battery_state"), 10)
        self.create_subscription(IoState, "io_state", self._store("io_state"), 10)
        self.create_subscription(MotorState, "motor_state", self._store("motor_state"), 10)
        self.create_subscription(SafetyState, "safety_state", self._store("safety_state"), 10)
        self.create_subscription(RobotState, "robot_state", self._store("robot_state"), 10)
        self.create_subscription(DiagnosticArray, "diagnostics", self._store("diagnostics"), 10)

    def _store(self, name: str):
        def callback(message: Any) -> None:
            self.samples[name] = Sample(message=message, received_monotonic=time.monotonic())

        return callback

    def sample(self, name: str) -> Optional[Any]:
        sample = self.samples.get(name)
        return None if sample is None else sample.message

    def age_seconds(self, name: str) -> Optional[float]:
        sample = self.samples.get(name)
        if sample is None:
            return None
        return time.monotonic() - sample.received_monotonic

    def print_report(self, topic_timeout_s: float) -> None:
        print("AMR health report")
        print("=================")
        self._print_topic_liveness(topic_timeout_s)
        self._print_robot_state()
        self._print_battery()
        self._print_io()
        self._print_motor()
        self._print_safety()
        self._print_diagnostics()

    def _print_topic_liveness(self, topic_timeout_s: float) -> None:
        print("\nTopic liveness")
        for name in [
            "battery_state",
            "io_state",
            "motor_state",
            "safety_state",
            "robot_state",
            "diagnostics",
        ]:
            age = self.age_seconds(name)
            if age is None:
                print(f"- /{name}: missing")
            elif age > topic_timeout_s:
                print(f"- /{name}: stale ({age:.2f}s)")
            else:
                print(f"- /{name}: ok ({age:.2f}s)")

    def _print_robot_state(self) -> None:
        msg = self.sample("robot_state")
        if msg is None:
            return
        mode_name = MODE_NAMES.get(msg.mode, f"UNKNOWN({msg.mode})")
        print("\nRobot state")
        print(f"- mode: {mode_name}")
        print(f"- fault_active: {msg.fault_active}")
        print(f"- message: {msg.message}")

    def _print_battery(self) -> None:
        msg = self.sample("battery_state")
        if msg is None:
            return
        percentage = "nan" if msg.percentage != msg.percentage else f"{msg.percentage * 100.0:.1f}%"
        print("\nBattery")
        print(f"- voltage: {msg.voltage:.2f} V")
        print(f"- current: {msg.current:.2f} A")
        print(f"- percentage: {percentage}")
        print(f"- health: {msg.power_supply_health}")

    def _print_io(self) -> None:
        msg = self.sample("io_state")
        if msg is None:
            return
        active_inputs = [
            name
            for name, value in zip(msg.input_names, msg.inputs)
            if value
        ]
        active_outputs = [
            name
            for name, value in zip(msg.output_names, msg.outputs)
            if value
        ]
        print("\nIO")
        print(f"- communication_ok: {msg.communication_ok}")
        print(f"- estop_active: {msg.estop_active}")
        print(f"- protective_stop_active: {msg.protective_stop_active}")
        print(f"- active_inputs: {active_inputs or 'none'}")
        print(f"- active_outputs: {active_outputs or 'none'}")

    def _print_motor(self) -> None:
        msg = self.sample("motor_state")
        if msg is None:
            return
        print("\nMotor")
        print(f"- communication_ok: {msg.communication_ok}")
        print(f"- drive_enabled: {msg.drive_enabled}")
        print(f"- fault_active: {msg.fault_active}")
        print(f"- left_velocity: {msg.left_wheel_velocity_radps:.3f} rad/s")
        print(f"- right_velocity: {msg.right_wheel_velocity_radps:.3f} rad/s")
        print(f"- command_age: {msg.command_age_ms:.1f} ms")

    def _print_safety(self) -> None:
        msg = self.sample("safety_state")
        if msg is None:
            return
        print("\nSafety")
        print(f"- command_allowed: {msg.command_allowed}")
        print(f"- command_timeout: {msg.command_timeout}")
        print(f"- communication_fault: {msg.communication_fault}")
        print(f"- reason: {msg.active_reason}")

    def _print_diagnostics(self) -> None:
        msg = self.sample("diagnostics")
        if msg is None:
            return
        worst_level = DiagnosticStatus.OK
        worst_name = "none"
        for status in msg.status:
            if status.level >= worst_level:
                worst_level = status.level
                worst_name = status.name
        level_name = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARN",
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE",
        }.get(worst_level, str(worst_level))
        print("\nDiagnostics")
        print(f"- status_count: {len(msg.status)}")
        print(f"- worst_level: {level_name}")
        print(f"- worst_name: {worst_name}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect a short AMR ROS 2 health report.")
    parser.add_argument("--duration", type=float, default=3.0, help="Collection duration in seconds.")
    parser.add_argument(
        "--topic-timeout",
        type=float,
        default=1.5,
        help="Topic age threshold reported as stale.",
    )
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main() -> None:
    args = parse_args()
    rclpy.init(args=sys.argv)
    node = HealthReport()
    try:
        deadline = time.monotonic() + max(args.duration, 0.1)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.print_report(topic_timeout_s=args.topic_timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

