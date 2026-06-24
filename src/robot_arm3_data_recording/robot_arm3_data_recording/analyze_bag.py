#!/usr/bin/env python3

import argparse
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


EVENT_TYPE = "peripheral_interfaces/msg/PeripheralEvent"
STATE_TYPE = "peripheral_interfaces/msg/PeripheralState"


def _parse_value(value: str) -> Any:
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


def _key_values(items) -> dict[str, Any]:
    return {item.key: _parse_value(item.value) for item in items}


def _seconds(timestamp_ns: int, start_ns: int) -> float:
    return (timestamp_ns - start_ns) / 1_000_000_000.0


class BagAnalysis:
    def __init__(self) -> None:
        self.topic_counts = Counter()
        self.topic_types = {}
        self.events = []
        self.state_samples = defaultdict(list)
        self.sensor_counts = Counter()
        self.start_ns = None
        self.end_ns = None

    def add_message(self, topic: str, type_name: str, message, timestamp_ns: int) -> None:
        if self.start_ns is None:
            self.start_ns = timestamp_ns
        self.end_ns = timestamp_ns
        self.topic_counts[topic] += 1
        self.topic_types[topic] = type_name

        if type_name == EVENT_TYPE:
            self.events.append(
                {
                    "time_s": _seconds(timestamp_ns, self.start_ns),
                    "device_id": message.device_id,
                    "event_type": message.event_type,
                    "command": message.command,
                    "success": message.success,
                    "detail": message.detail,
                }
            )
        elif type_name == STATE_TYPE:
            self.state_samples[topic].append(
                {
                    "time_s": _seconds(timestamp_ns, self.start_ns),
                    "device_id": message.device_id,
                    "state": message.state,
                    "ready": message.ready,
                    "fault": message.fault,
                    "values": _key_values(message.values),
                }
            )
        elif topic.startswith("/sensors/"):
            self.sensor_counts[topic] += 1

    def report(self) -> str:
        lines = []
        duration_s = 0.0
        if self.start_ns is not None and self.end_ns is not None:
            duration_s = _seconds(self.end_ns, self.start_ns)

        lines.append("Robot Arm3 Bag Analysis")
        lines.append("=======================")
        lines.append(f"Duration: {duration_s:.3f} s")
        lines.append("")

        lines.append("Recorded Topics")
        lines.append("---------------")
        for topic in sorted(self.topic_counts):
            lines.append(
                f"- {topic}: {self.topic_counts[topic]} message(s), "
                f"type={self.topic_types.get(topic, 'unknown')}"
            )
        lines.append("")

        lines.append("Process/Event Timeline")
        lines.append("----------------------")
        if not self.events:
            lines.append("- No PeripheralEvent messages found.")
        else:
            for event in self.events:
                ok = "ok" if event["success"] else "failed"
                lines.append(
                    f"- t={event['time_s']:.3f}s "
                    f"{event['device_id']} {event['event_type']} "
                    f"command={event['command']} result={ok}: "
                    f"{event['detail']}"
                )
        lines.append("")

        lines.append("Peripheral State Summary")
        lines.append("------------------------")
        if not self.state_samples:
            lines.append("- No PeripheralState messages found.")
        else:
            for topic in sorted(self.state_samples):
                samples = self.state_samples[topic]
                first = samples[0]
                last = samples[-1]
                lines.append(
                    f"- {topic}: {len(samples)} sample(s), "
                    f"first_state={first['state']}, last_state={last['state']}, "
                    f"last_ready={last['ready']}, last_fault={last['fault']}"
                )
                numeric_ranges = self._numeric_ranges(samples)
                for key, value_range in sorted(numeric_ranges.items()):
                    lines.append(
                        f"  - {key}: min={value_range[0]:.3f}, "
                        f"max={value_range[1]:.3f}"
                    )
        lines.append("")

        lines.append("Sensor Streams")
        lines.append("--------------")
        if not self.sensor_counts:
            lines.append("- No /sensors/* messages found.")
        else:
            for topic in sorted(self.sensor_counts):
                lines.append(f"- {topic}: {self.sensor_counts[topic]} sample(s)")

        return "\n".join(lines)

    @staticmethod
    def _numeric_ranges(samples) -> dict[str, tuple[float, float]]:
        ranges = {}
        for sample in samples:
            for key, value in sample["values"].items():
                if isinstance(value, bool):
                    continue
                try:
                    number = float(value)
                except (TypeError, ValueError):
                    continue
                if key not in ranges:
                    ranges[key] = [number, number]
                else:
                    ranges[key][0] = min(ranges[key][0], number)
                    ranges[key][1] = max(ranges[key][1], number)
        return {key: (value[0], value[1]) for key, value in ranges.items()}


def analyze_bag(bag_path: str, storage_id: str) -> BagAnalysis:
    bag_directory = Path(bag_path)
    if not bag_directory.exists():
        raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_directory),
        storage_id=storage_id,
    )
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
    }

    analysis = BagAnalysis()
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        type_name = topic_types[topic]
        message = deserialize_message(data, message_types[topic])
        analysis.add_message(topic, type_name, message, timestamp_ns)
    return analysis


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze robot_arm3 rosbag data for offline debugging."
    )
    parser.add_argument("bag_path", help="Path to a rosbag2 directory.")
    parser.add_argument(
        "--storage-id",
        default="sqlite3",
        help="rosbag2 storage id used by the bag.",
    )
    parser.add_argument(
        "--report-file",
        default="",
        help="Optional path to write the text report.",
    )
    args = parser.parse_args()

    report = analyze_bag(args.bag_path, args.storage_id).report()
    print(report)
    if args.report_file:
        Path(args.report_file).write_text(report + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
