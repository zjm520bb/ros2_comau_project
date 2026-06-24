#!/usr/bin/env python3

import threading
import time
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from arm_tcp_bridge_interfaces.action import ExecuteCommand as ArmCommand
from diagnostic_msgs.msg import KeyValue
from peripheral_interfaces.action import ExecutePeripheralCommand
from peripheral_interfaces.msg import PeripheralEvent, PeripheralState
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


def _key_values(values: dict[str, Any]) -> list[KeyValue]:
    return [
        KeyValue(key=str(key), value=str(value))
        for key, value in sorted(values.items())
    ]


def _parse_value(value: Any) -> Any:
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, (int, float)):
        return float(value)

    text = str(value).strip()
    lower = text.lower()
    if lower == "true":
        return True
    if lower == "false":
        return False
    try:
        return float(text)
    except ValueError:
        return text


def _state_values(message: PeripheralState) -> dict[str, Any]:
    values = {
        "state": message.state,
        "ready": message.ready,
        "fault": message.fault,
        "mode": message.mode,
        "device_type": message.device_type,
        "display_name": message.display_name,
    }
    for item in message.values:
        values[item.key] = _parse_value(item.value)
    return values


class ProcessOrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("process_orchestrator")

        package_share = get_package_share_directory("robot_arm3_peripherals")
        self.declare_parameter(
            "process_file",
            f"{package_share}/config/process_demo.yaml",
        )
        self.declare_parameter("arm_action_name", "/sim/arm/execute")
        self.declare_parameter("start_service_name", "/process/start_demo")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("action_timeout_s", 120.0)

        self._callback_group = ReentrantCallbackGroup()
        self._arm_client = ActionClient(
            self,
            ArmCommand,
            str(self.get_parameter("arm_action_name").value),
            callback_group=self._callback_group,
        )
        self._peripheral_clients = {}
        self._state_subscriptions = {}
        self._latest_states: dict[str, PeripheralState] = {}
        self._running = False
        self._run_lock = threading.Lock()
        self._state_lock = threading.Lock()

        self._event_publisher = self.create_publisher(
            PeripheralEvent,
            "/peripherals/events",
            10,
        )

        self._process = self._load_process(
            str(self.get_parameter("process_file").value)
        )

        self.create_service(
            Trigger,
            str(self.get_parameter("start_service_name").value),
            self._handle_start,
            callback_group=self._callback_group,
        )

        if bool(self.get_parameter("auto_start").value):
            self.create_timer(
                1.0,
                self._auto_start_once,
                callback_group=self._callback_group,
            )

        self.get_logger().info(
            "Process orchestrator ready for process "
            f"{self._process['process_name']!r}"
        )

    @staticmethod
    def _load_process(process_file: str) -> dict[str, Any]:
        with open(process_file, encoding="utf-8") as stream:
            process = yaml.safe_load(stream)
        if not isinstance(process, dict):
            raise ValueError("Process config must be a mapping")
        if not isinstance(process.get("steps"), list):
            raise ValueError("Process config must contain a steps list")
        process.setdefault("process_name", "unnamed_process")
        return process

    def _auto_start_once(self) -> None:
        if getattr(self, "_auto_started", False):
            return
        self._auto_started = True
        self._start_background_run()

    def _handle_start(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        started = self._start_background_run()
        response.success = started
        response.message = (
            "Process started" if started else "Process is already running"
        )
        return response

    def _start_background_run(self) -> bool:
        with self._run_lock:
            if self._running:
                return False
            self._running = True

        thread = threading.Thread(
            target=self._run_process_thread,
            name="process_orchestrator_worker",
            daemon=True,
        )
        thread.start()
        return True

    def _run_process_thread(self) -> None:
        process_name = str(self._process["process_name"])
        self._publish_process_event(
            "process_started",
            True,
            f"Process {process_name} started",
        )
        try:
            for index, step in enumerate(self._process["steps"], start=1):
                self._execute_step(index, step)
            self._publish_process_event(
                "process_finished",
                True,
                f"Process {process_name} finished",
            )
        except Exception as error:
            self.get_logger().error(f"Process failed: {error}")
            self._publish_process_event(
                "process_failed",
                False,
                str(error),
            )
        finally:
            with self._run_lock:
                self._running = False

    def _execute_step(self, index: int, step: dict[str, Any]) -> None:
        step_type = str(step.get("type", "")).strip()
        self.get_logger().info(f"Executing process step {index}: {step}")

        if step_type == "peripheral":
            self._execute_peripheral_step(step)
            return
        if step_type == "robot":
            self._execute_robot_step(step)
            return
        if step_type == "wait_state":
            self._execute_wait_state_step(step)
            return
        if step_type == "delay":
            time.sleep(float(step.get("seconds", 0.0)))
            return

        raise RuntimeError(f"Unsupported process step type: {step_type!r}")

    def _execute_peripheral_step(self, step: dict[str, Any]) -> None:
        device_id = str(step.get("device_id", "")).strip()
        command = str(step.get("command", "")).strip()
        parameters = step.get("parameters", {})
        if not isinstance(parameters, dict):
            raise RuntimeError("Peripheral step parameters must be a mapping")
        if not device_id or not command:
            raise RuntimeError("Peripheral step requires device_id and command")

        client = self._peripheral_client(device_id)
        timeout_s = float(self.get_parameter("action_timeout_s").value)
        if not client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError(
                f"Peripheral action server is unavailable: {device_id}"
            )

        goal = ExecutePeripheralCommand.Goal()
        goal.device_id = device_id
        goal.command = command
        goal.parameters = _key_values(parameters)

        goal_handle = self._wait_future(
            client.send_goal_async(goal),
            timeout_s,
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"Peripheral rejected command: {device_id}")

        wrapped_result = self._wait_future(
            goal_handle.get_result_async(),
            timeout_s,
        )
        if not wrapped_result.result.success:
            raise RuntimeError(
                f"Peripheral command failed on {device_id}: "
                f"{wrapped_result.result.message}"
            )

    def _execute_robot_step(self, step: dict[str, Any]) -> None:
        command = str(step.get("command", "")).strip()
        if not command:
            raise RuntimeError("Robot step requires command")

        timeout_s = float(self.get_parameter("action_timeout_s").value)
        if not self._arm_client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError("Robot command action server is unavailable")

        goal = ArmCommand.Goal()
        goal.command = command
        goal_handle = self._wait_future(
            self._arm_client.send_goal_async(goal),
            timeout_s,
        )
        if not goal_handle.accepted:
            raise RuntimeError("Robot command was rejected")

        wrapped_result = self._wait_future(
            goal_handle.get_result_async(),
            timeout_s,
        )
        if not wrapped_result.result.success:
            raise RuntimeError(
                "Robot command failed: " + wrapped_result.result.message
            )

    def _execute_wait_state_step(self, step: dict[str, Any]) -> None:
        device_id = str(step.get("device_id", "")).strip()
        key = str(step.get("key", "")).strip()
        timeout_s = float(step.get("timeout_s", 5.0))
        if not device_id or not key:
            raise RuntimeError("wait_state requires device_id and key")
        if timeout_s <= 0.0:
            raise RuntimeError("wait_state timeout_s must be positive")

        self._ensure_state_subscription(device_id)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok():
            with self._state_lock:
                latest = self._latest_states.get(device_id)
            if latest is not None and self._state_matches(latest, step):
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timed out waiting for {device_id}.{key} "
                    f"to satisfy {step}"
                )
            time.sleep(0.05)

    def _state_matches(
        self,
        message: PeripheralState,
        step: dict[str, Any],
    ) -> bool:
        key = str(step["key"])
        values = _state_values(message)
        if key not in values:
            return False

        actual = values[key]
        if "value" in step:
            expected = _parse_value(step["value"])
            return actual == expected

        if "min" in step or "max" in step:
            try:
                actual_number = float(actual)
            except (TypeError, ValueError):
                return False
            if "min" in step and actual_number < float(step["min"]):
                return False
            if "max" in step and actual_number > float(step["max"]):
                return False
            return True

        raise RuntimeError("wait_state requires value, min, or max")

    def _ensure_state_subscription(self, device_id: str) -> None:
        if device_id in self._state_subscriptions:
            return

        self._state_subscriptions[device_id] = self.create_subscription(
            PeripheralState,
            f"/peripherals/{device_id}/state",
            lambda message, stored_id=device_id:
            self._state_callback(stored_id, message),
            10,
            callback_group=self._callback_group,
        )

    def _state_callback(
        self,
        device_id: str,
        message: PeripheralState,
    ) -> None:
        with self._state_lock:
            self._latest_states[device_id] = message

    def _peripheral_client(self, device_id: str) -> ActionClient:
        if device_id not in self._peripheral_clients:
            self._peripheral_clients[device_id] = ActionClient(
                self,
                ExecutePeripheralCommand,
                f"/peripherals/{device_id}/execute",
                callback_group=self._callback_group,
            )
        return self._peripheral_clients[device_id]

    @staticmethod
    def _wait_future(future, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for action response")
            time.sleep(0.05)
        return future.result()

    def _publish_process_event(
        self,
        event_type: str,
        success: bool,
        detail: str,
    ) -> None:
        message = PeripheralEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.device_id = "process_orchestrator"
        message.display_name = str(self._process["process_name"])
        message.device_type = "process"
        message.event_type = event_type
        message.command = "run_process"
        message.success = success
        message.detail = detail
        self._event_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProcessOrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
