import argparse
import sys
import time
from typing import Any, Type

import rclpy
from amr_interfaces.srv import InjectMotorFault, SetBatteryPercentage, SetDigitalInput
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool, Trigger


class FaultScenarioClient(Node):
    def __init__(self, service_timeout_s: float) -> None:
        super().__init__("amr_fault_scenario")
        self.service_timeout_s = service_timeout_s

    def call(self, service_type: Type[Any], service_name: str, request: Any) -> Any:
        client = self.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=self.service_timeout_s):
            raise RuntimeError(f"service not available: {service_name}")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout_s)
        if not future.done():
            raise TimeoutError(f"service call timed out: {service_name}")

        result = future.result()
        if result is None:
            raise RuntimeError(f"service call failed: {service_name}")
        return result

    def set_input(self, channel: int, value: bool) -> None:
        request = SetDigitalInput.Request()
        request.channel = channel
        request.value = value
        response = self.call(SetDigitalInput, "set_input", request)
        self._print_response("set_input", response)

    def set_battery_percentage(self, percentage: float) -> None:
        request = SetBatteryPercentage.Request()
        request.percentage = percentage
        response = self.call(SetBatteryPercentage, "set_battery_percentage", request)
        self._print_response("set_battery_percentage", response)

    def inject_motor_fault(self, fault_code: int, description: str) -> None:
        request = InjectMotorFault.Request()
        request.fault_code = fault_code
        request.description = description
        response = self.call(InjectMotorFault, "inject_motor_fault", request)
        self._print_response("inject_motor_fault", response)

    def clear_motor_fault(self) -> None:
        response = self.call(Trigger, "clear_motor_fault", Trigger.Request())
        self._print_response("clear_motor_fault", response)

    def reset_system_fault(self) -> None:
        response = self.call(Trigger, "reset_fault", Trigger.Request())
        self._print_response("reset_fault", response)

    def motor_enable(self, value: bool) -> None:
        request = SetBool.Request()
        request.data = value
        response = self.call(SetBool, "motor_enable", request)
        self._print_response("motor_enable", response)

    def settle(self, duration_s: float = 0.3) -> None:
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    @staticmethod
    def _print_response(name: str, response: Any) -> None:
        status = "OK" if getattr(response, "success", False) else "FAIL"
        print(f"{name}: {status} - {getattr(response, 'message', '')}")


def run_scenario(node: FaultScenarioClient, args: argparse.Namespace) -> None:
    if args.scenario == "estop-on":
        node.set_input(args.estop_channel, True)
    elif args.scenario == "estop-off":
        node.set_input(args.estop_channel, False)
    elif args.scenario == "protective-on":
        node.set_input(args.protective_channel, True)
    elif args.scenario == "protective-off":
        node.set_input(args.protective_channel, False)
    elif args.scenario == "battery-normal":
        node.set_battery_percentage(0.85)
    elif args.scenario == "battery-low":
        node.set_battery_percentage(0.18)
    elif args.scenario == "battery-critical":
        node.set_battery_percentage(0.08)
    elif args.scenario == "motor-fault":
        node.inject_motor_fault(args.fault_code, args.description)
    elif args.scenario == "motor-clear":
        node.clear_motor_fault()
        node.motor_enable(True)
    elif args.scenario == "recover":
        node.set_input(args.estop_channel, False)
        node.set_input(args.protective_channel, False)
        node.set_battery_percentage(0.85)
        node.clear_motor_fault()
        node.motor_enable(True)
        node.settle()
        node.reset_system_fault()
    else:
        raise ValueError(f"unknown scenario: {args.scenario}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run AMR fault-injection scenarios against the mock robot stack."
    )
    parser.add_argument(
        "scenario",
        choices=[
            "estop-on",
            "estop-off",
            "protective-on",
            "protective-off",
            "battery-normal",
            "battery-low",
            "battery-critical",
            "motor-fault",
            "motor-clear",
            "recover",
        ],
    )
    parser.add_argument("--estop-channel", type=int, default=0)
    parser.add_argument("--protective-channel", type=int, default=1)
    parser.add_argument("--fault-code", type=int, default=2310)
    parser.add_argument("--description", default="mock drive overcurrent")
    parser.add_argument("--service-timeout", type=float, default=2.0)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main() -> None:
    args = parse_args()
    rclpy.init(args=sys.argv)
    node = FaultScenarioClient(service_timeout_s=args.service_timeout)
    try:
        run_scenario(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
