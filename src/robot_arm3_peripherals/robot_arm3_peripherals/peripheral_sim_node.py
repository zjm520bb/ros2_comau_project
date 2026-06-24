#!/usr/bin/env python3

import copy
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import KeyValue
from peripheral_interfaces.action import ExecutePeripheralCommand
from peripheral_interfaces.msg import PeripheralEvent, PeripheralState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def _string_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _key_values(values: dict[str, Any]) -> list[KeyValue]:
    return [
        KeyValue(key=str(key), value=_string_value(value))
        for key, value in sorted(values.items())
    ]


def _parameters_to_dict(parameters: list[KeyValue]) -> dict[str, str]:
    return {item.key: item.value for item in parameters}


@dataclass
class PeripheralDevice:
    device_id: str
    display_name: str
    device_type: str
    state: str
    mode: str
    values: dict[str, Any] = field(default_factory=dict)
    ready: bool = True
    fault: bool = False
    spindle_commanded_running: bool = False
    pneumatic_target_clamped: bool | None = None
    pneumatic_transition_until: float = 0.0


class PeripheralSimNode(Node):
    def __init__(self) -> None:
        super().__init__("peripheral_sim_node")

        package_share = get_package_share_directory("robot_arm3_peripherals")
        self.declare_parameter(
            "config_file",
            f"{package_share}/config/peripherals.yaml",
        )
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("simulation_rate_hz", 20.0)
        self.declare_parameter("spindle_ramp_rpm_s", 1500.0)
        self.declare_parameter("pneumatic_transition_s", 0.5)

        self._callback_group = ReentrantCallbackGroup()
        self._devices: dict[str, PeripheralDevice] = {}
        self._state_publishers = {}
        self._action_servers = []
        self._lock = threading.Lock()
        self._last_simulation_time = time.monotonic()

        self._event_publisher = self.create_publisher(
            PeripheralEvent,
            "/peripherals/events",
            10,
        )

        self._load_config(
            str(self.get_parameter("config_file").value)
        )

        publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_all_states,
            callback_group=self._callback_group,
        )
        simulation_rate_hz = float(
            self.get_parameter("simulation_rate_hz").value
        )
        if simulation_rate_hz <= 0.0:
            raise ValueError("simulation_rate_hz must be positive")
        self.create_timer(
            1.0 / simulation_rate_hz,
            self._update_simulated_devices,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "Peripheral simulation node started with devices: "
            + ", ".join(sorted(self._devices))
        )

    def _load_config(self, config_file: str) -> None:
        with open(config_file, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        peripherals = config.get("peripherals", [])
        if not isinstance(peripherals, list):
            raise ValueError("peripherals config must contain a list")

        for entry in peripherals:
            device = self._device_from_entry(entry)
            if device.device_id in self._devices:
                raise ValueError(
                    f"Duplicate peripheral id: {device.device_id}"
                )

            self._devices[device.device_id] = device
            self._state_publishers[device.device_id] = self.create_publisher(
                PeripheralState,
                f"/peripherals/{device.device_id}/state",
                10,
            )
            self._action_servers.append(
                ActionServer(
                    self,
                    ExecutePeripheralCommand,
                    f"/peripherals/{device.device_id}/execute",
                    execute_callback=(
                        lambda goal_handle, device_id=device.device_id:
                        self._execute_command(goal_handle, device_id)
                    ),
                    goal_callback=(
                        lambda goal, device_id=device.device_id:
                        self._goal_callback(goal, device_id)
                    ),
                    cancel_callback=self._cancel_callback,
                    callback_group=self._callback_group,
                )
            )

    @staticmethod
    def _device_from_entry(entry: dict[str, Any]) -> PeripheralDevice:
        device_id = str(entry.get("id", "")).strip()
        if not device_id:
            raise ValueError("Peripheral id cannot be empty")
        values = copy.deepcopy(entry.get("values", {}))
        return PeripheralDevice(
            device_id=device_id,
            display_name=str(entry.get("display_name", device_id)),
            device_type=str(entry.get("type", "generic")),
            state=str(entry.get("initial_state", "ready")),
            mode=str(entry.get("initial_mode", "simulated")),
            values=values,
            spindle_commanded_running=bool(values.get("running", False)),
            pneumatic_target_clamped=bool(values.get("clamped", False)),
        )

    def _goal_callback(
        self,
        goal: ExecutePeripheralCommand.Goal,
        device_id: str,
    ) -> GoalResponse:
        requested_id = goal.device_id.strip()
        command = goal.command.strip()
        if requested_id and requested_id != device_id:
            self.get_logger().warn(
                f"Rejecting command for {requested_id} on {device_id}"
            )
            return GoalResponse.REJECT
        if not command:
            self.get_logger().warn(
                f"Rejecting empty peripheral command on {device_id}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute_command(
        self,
        goal_handle,
        device_id: str,
    ) -> ExecutePeripheralCommand.Result:
        command = goal_handle.request.command.strip()
        parameters = _parameters_to_dict(goal_handle.request.parameters)

        self._publish_event(
            device_id,
            "command_started",
            command,
            True,
            "Command accepted by simulated peripheral",
        )
        self._publish_feedback(goal_handle, "executing", 0.1)

        result = ExecutePeripheralCommand.Result()
        try:
            message = self._dispatch(device_id, command, parameters)
            self._publish_feedback(goal_handle, "finished", 1.0)
            goal_handle.succeed()
            result.success = True
            result.message = message
            self._publish_event(
                device_id,
                "command_finished",
                command,
                True,
                message,
            )
        except Exception as error:
            goal_handle.abort()
            result.success = False
            result.message = str(error)
            self._publish_event(
                device_id,
                "command_failed",
                command,
                False,
                str(error),
            )

        self._publish_state(device_id)
        return result

    def _dispatch(
        self,
        device_id: str,
        command: str,
        parameters: dict[str, str],
    ) -> str:
        with self._lock:
            device = self._devices[device_id]
            if command == "wait_ready":
                if device.fault or not device.ready:
                    raise RuntimeError(f"{device_id} is not ready")
                return f"{device.display_name} is ready"
            if command == "reset_fault":
                device.fault = False
                device.ready = True
                device.state = "ready"
                return f"{device.display_name} fault reset"

            if device.device_type == "spindle":
                return self._handle_spindle(device, command, parameters)
            if device.device_type == "pneumatic_controller":
                return self._handle_pneumatic(device, command)
            if device.device_type == "passive_fixture":
                return self._handle_passive_fixture(device, command)

            raise RuntimeError(
                f"Unsupported command {command!r} for {device.device_type}"
            )

    @staticmethod
    def _handle_passive_fixture(
        device: PeripheralDevice,
        command: str,
    ) -> str:
        if command == "enable":
            device.ready = True
            device.state = "ready"
            return f"{device.display_name} enabled"
        if command == "disable":
            device.ready = False
            device.state = "disabled"
            return f"{device.display_name} disabled"
        raise RuntimeError(
            f"Unsupported command {command!r} for passive fixture"
        )

    def _handle_spindle(
        self,
        device: PeripheralDevice,
        command: str,
        parameters: dict[str, str],
    ) -> str:
        if command == "set_speed":
            rpm_text = parameters.get("rpm", parameters.get("target_rpm", ""))
            target_rpm = float(rpm_text)
            if target_rpm < 0.0:
                raise RuntimeError("Spindle rpm must be non-negative")
            device.values["target_rpm"] = int(round(target_rpm))
            if not device.spindle_commanded_running:
                device.values["actual_rpm"] = 0
                device.values["running"] = False
            elif float(device.values.get("actual_rpm", 0.0)) < target_rpm:
                device.state = "accelerating"
                device.values["running"] = False
            return f"{device.display_name} target speed set"

        if command == "start":
            target_rpm = float(device.values.get("target_rpm", 0.0))
            if target_rpm <= 0.0:
                raise RuntimeError("Set a positive spindle rpm before start")
            device.ready = True
            device.spindle_commanded_running = True
            device.state = "accelerating"
            device.values["running"] = False
            return f"{device.display_name} accelerating"

        if command == "stop":
            device.spindle_commanded_running = False
            device.state = "decelerating"
            device.values["running"] = False
            return f"{device.display_name} decelerating"

        raise RuntimeError(f"Unsupported spindle command {command!r}")

    def _handle_pneumatic(
        self,
        device: PeripheralDevice,
        command: str,
    ) -> str:
        transition_s = float(
            self.get_parameter("pneumatic_transition_s").value
        )
        now = time.monotonic()
        if command == "clamp":
            device.state = "clamping"
            device.pneumatic_target_clamped = True
            device.pneumatic_transition_until = now + transition_s
            device.values["valve_state"] = "clamp"
            return f"{device.display_name} clamping"
        if command == "release":
            device.state = "releasing"
            device.pneumatic_target_clamped = False
            device.pneumatic_transition_until = now + transition_s
            device.values["valve_state"] = "release"
            return f"{device.display_name} releasing"
        raise RuntimeError(f"Unsupported pneumatic command {command!r}")

    def _update_simulated_devices(self) -> None:
        now = time.monotonic()
        elapsed_s = now - self._last_simulation_time
        self._last_simulation_time = now
        if elapsed_s <= 0.0:
            return

        with self._lock:
            for device in self._devices.values():
                if device.device_type == "spindle":
                    self._update_spindle(device, elapsed_s)
                elif device.device_type == "pneumatic_controller":
                    self._update_pneumatic(device, now)

    def _update_spindle(
        self,
        device: PeripheralDevice,
        elapsed_s: float,
    ) -> None:
        ramp_rpm_s = float(
            self.get_parameter("spindle_ramp_rpm_s").value
        )
        if ramp_rpm_s <= 0.0:
            raise ValueError("spindle_ramp_rpm_s must be positive")

        target_rpm = float(device.values.get("target_rpm", 0.0))
        actual_rpm = float(device.values.get("actual_rpm", 0.0))
        step = ramp_rpm_s * elapsed_s

        if device.spindle_commanded_running:
            actual_rpm = min(target_rpm, actual_rpm + step)
            device.values["actual_rpm"] = int(round(actual_rpm))
            if target_rpm <= 0.0:
                device.state = "idle"
                device.values["running"] = False
            elif abs(target_rpm - actual_rpm) <= max(5.0, 0.02 * target_rpm):
                device.values["actual_rpm"] = int(round(target_rpm))
                device.state = "running"
                device.values["running"] = True
            else:
                device.state = "accelerating"
                device.values["running"] = False
            return

        actual_rpm = max(0.0, actual_rpm - step)
        device.values["actual_rpm"] = int(round(actual_rpm))
        device.values["running"] = False
        device.state = "idle" if actual_rpm <= 0.0 else "decelerating"

    @staticmethod
    def _update_pneumatic(
        device: PeripheralDevice,
        now: float,
    ) -> None:
        if device.pneumatic_target_clamped is None:
            return
        if device.pneumatic_transition_until > now:
            return

        if device.pneumatic_target_clamped:
            device.state = "clamped"
            device.values["clamped"] = True
            device.values["valve_state"] = "clamp"
        else:
            device.state = "released"
            device.values["clamped"] = False
            device.values["valve_state"] = "release"
        device.pneumatic_transition_until = 0.0

    def _publish_all_states(self) -> None:
        for device_id in self._devices:
            self._publish_state(device_id)

    def _publish_state(self, device_id: str) -> None:
        with self._lock:
            device = copy.deepcopy(self._devices[device_id])

        message = PeripheralState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.device_id = device.device_id
        message.display_name = device.display_name
        message.device_type = device.device_type
        message.state = device.state
        message.ready = device.ready and not device.fault
        message.fault = device.fault
        message.mode = device.mode
        message.values = _key_values(device.values)
        self._state_publishers[device_id].publish(message)

    def _publish_event(
        self,
        device_id: str,
        event_type: str,
        command: str,
        success: bool,
        detail: str,
    ) -> None:
        with self._lock:
            device = copy.deepcopy(self._devices[device_id])

        message = PeripheralEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.device_id = device.device_id
        message.display_name = device.display_name
        message.device_type = device.device_type
        message.event_type = event_type
        message.command = command
        message.success = success
        message.detail = detail
        message.values = _key_values(device.values)
        self._event_publisher.publish(message)

    @staticmethod
    def _publish_feedback(goal_handle, state: str, progress: float) -> None:
        feedback = ExecutePeripheralCommand.Feedback()
        feedback.state = state
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PeripheralSimNode()
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
