#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure


def _three_values(values: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{label} must contain exactly three values")
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{label} contains NaN or infinity")
    return converted


@dataclass
class SensorSpec:
    sensor_id: str
    display_name: str
    sensor_type: str
    topic: str
    frame_id: str
    publish_rate_hz: float
    options: dict[str, Any]


class SensorSimNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_sim_node")

        package_share = get_package_share_directory("robot_arm3_sensors")
        self.declare_parameter(
            "config_file",
            f"{package_share}/config/sensors.yaml",
        )

        self._start_time = time.monotonic()
        self._sensor_publishers = {}
        self._timers = []

        self._load_config(str(self.get_parameter("config_file").value))

        self.get_logger().info(
            "Sensor simulation node started with sensors: "
            + ", ".join(sorted(self._sensor_publishers))
        )

    def _load_config(self, config_file: str) -> None:
        with open(config_file, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        sensors = config.get("sensors", [])
        if not isinstance(sensors, list):
            raise ValueError("sensors config must contain a list")

        identifiers = set()
        for entry in sensors:
            spec = self._sensor_from_entry(entry)
            if spec.sensor_id in identifiers:
                raise ValueError(f"Duplicate sensor id: {spec.sensor_id}")
            identifiers.add(spec.sensor_id)

            if spec.publish_rate_hz <= 0.0:
                raise ValueError(
                    f"publish_rate_hz must be positive for {spec.sensor_id}"
                )

            if spec.sensor_type == "wrench":
                publisher = self.create_publisher(
                    WrenchStamped,
                    spec.topic,
                    10,
                )
                self._sensor_publishers[spec.sensor_id] = publisher
                self._timers.append(
                    self.create_timer(
                        1.0 / spec.publish_rate_hz,
                        lambda sensor=spec: self._publish_wrench(sensor),
                    )
                )
            elif spec.sensor_type == "fluid_pressure":
                publisher = self.create_publisher(
                    FluidPressure,
                    spec.topic,
                    10,
                )
                self._sensor_publishers[spec.sensor_id] = publisher
                self._timers.append(
                    self.create_timer(
                        1.0 / spec.publish_rate_hz,
                        lambda sensor=spec: self._publish_pressure(sensor),
                    )
                )
            else:
                raise ValueError(
                    f"Unsupported sensor type: {spec.sensor_type!r}"
                )

    @staticmethod
    def _sensor_from_entry(entry: dict[str, Any]) -> SensorSpec:
        sensor_id = str(entry.get("id", "")).strip()
        if not sensor_id:
            raise ValueError("Sensor id cannot be empty")
        sensor_type = str(entry.get("type", "")).strip()
        options_key = "wrench" if sensor_type == "wrench" else "pressure"
        return SensorSpec(
            sensor_id=sensor_id,
            display_name=str(entry.get("display_name", sensor_id)),
            sensor_type=sensor_type,
            topic=str(entry.get("topic", "")).strip(),
            frame_id=str(entry.get("frame_id", "")).strip(),
            publish_rate_hz=float(entry.get("publish_rate_hz", 1.0)),
            options=dict(entry.get(options_key, {})),
        )

    def _elapsed_s(self) -> float:
        return time.monotonic() - self._start_time

    def _publish_wrench(self, spec: SensorSpec) -> None:
        options = spec.options
        force_base = _three_values(
            options.get("force_base_n", [0.0, 0.0, 0.0]),
            f"{spec.sensor_id}.force_base_n",
        )
        force_amplitude = _three_values(
            options.get("force_amplitude_n", [0.0, 0.0, 0.0]),
            f"{spec.sensor_id}.force_amplitude_n",
        )
        torque_base = _three_values(
            options.get("torque_base_nm", [0.0, 0.0, 0.0]),
            f"{spec.sensor_id}.torque_base_nm",
        )
        torque_amplitude = _three_values(
            options.get("torque_amplitude_nm", [0.0, 0.0, 0.0]),
            f"{spec.sensor_id}.torque_amplitude_nm",
        )

        elapsed = self._elapsed_s()
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = spec.frame_id
        message.wrench.force.x = force_base[0] + force_amplitude[0] * math.sin(elapsed)
        message.wrench.force.y = force_base[1] + force_amplitude[1] * math.sin(0.7 * elapsed + 0.4)
        message.wrench.force.z = force_base[2] + force_amplitude[2] * math.sin(1.3 * elapsed + 0.8)
        message.wrench.torque.x = torque_base[0] + torque_amplitude[0] * math.sin(0.9 * elapsed)
        message.wrench.torque.y = torque_base[1] + torque_amplitude[1] * math.sin(1.1 * elapsed + 0.2)
        message.wrench.torque.z = torque_base[2] + torque_amplitude[2] * math.sin(1.4 * elapsed + 0.6)
        self._sensor_publishers[spec.sensor_id].publish(message)

    def _publish_pressure(self, spec: SensorSpec) -> None:
        options = spec.options
        base_pa = float(options.get("base_pa", 101325.0))
        amplitude_pa = float(options.get("amplitude_pa", 0.0))
        variance = float(options.get("variance", 0.0))
        if not all(math.isfinite(value) for value in [base_pa, amplitude_pa, variance]):
            raise ValueError(f"{spec.sensor_id} pressure options must be finite")

        elapsed = self._elapsed_s()
        message = FluidPressure()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = spec.frame_id
        message.fluid_pressure = base_pa + amplitude_pa * math.sin(0.5 * elapsed)
        message.variance = variance
        self._sensor_publishers[spec.sensor_id].publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
