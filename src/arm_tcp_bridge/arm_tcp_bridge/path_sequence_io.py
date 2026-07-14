from collections.abc import Mapping, Sequence
from numbers import Integral, Real
from pathlib import Path
from typing import Any

import yaml

from arm_tcp_bridge_interfaces.msg import (
    PathBlock,
    PathCondition,
    PathFrame,
    PathNode,
    PathSequence,
)


FORMAT_VERSION = 2
SUPPORTED_FORMAT_VERSIONS = (1, FORMAT_VERSION)


def _float_list(values, size: int, name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != size:
        raise ValueError(f"{name} must contain {size} values")
    return result


def sequence_to_dict(sequence: PathSequence) -> dict[str, Any]:
    return {
        "format_version": FORMAT_VERSION,
        "sequence_id": int(sequence.sequence_id),
        "name": sequence.name,
        "robot_model": sequence.robot_model,
        "coupling_offset": float(sequence.coupling_offset),
        "stop_on_error": bool(sequence.stop_on_error),
        "paths": [
            {
                "name": block.name,
                "path_id": int(block.path_id),
                "path_type": int(block.path_type),
                "start_index": int(block.start_index),
                "end_index": int(block.end_index),
                "expected_start_deg": [
                    float(value)
                    for value in block.expected_start_deg
                ],
                "expected_end_deg": [
                    float(value)
                    for value in block.expected_end_deg
                ],
                "wait_after": bool(block.wait_after),
                "frames": [
                    {
                        "index": int(frame.index),
                        "pose": [float(value) for value in frame.pose],
                    }
                    for frame in block.frames
                ],
                "conditions": [
                    {
                        "slot": int(condition.slot),
                        "handler_id": int(condition.handler_id),
                    }
                    for condition in block.conditions
                ],
                "nodes": [
                    dict(
                        target=[float(value) for value in node.target],
                        **{
                            field: getattr(node, field)
                            for field in (
                                "motion_type",
                                "linear_speed",
                                "rotational_speed",
                                "segment_override",
                                "termination_type",
                                "tolerance",
                                "segment_data",
                                "fly",
                                "fly_type",
                                "fly_percent",
                                "fly_distance_mm",
                                "fly_trajectory",
                                "stress_percent",
                                "reference_index",
                                "tool_index",
                                "condition_mask",
                                "condition_mask_back",
                                "wait",
                            )
                        },
                    )
                    for node in block.nodes
                ],
            }
            for block in sequence.paths
        ],
    }


def sequence_from_dict(data: dict[str, Any]) -> PathSequence:
    if int(data.get("format_version", 0)) not in SUPPORTED_FORMAT_VERSIONS:
        raise ValueError("unsupported PATH sequence format_version")
    sequence = PathSequence()
    sequence.sequence_id = int(data.get("sequence_id", 0))
    sequence.name = str(data.get("name", ""))
    sequence.robot_model = str(data.get("robot_model", "robot_arm3"))
    sequence.coupling_offset = float(data.get("coupling_offset", 1.5708))
    sequence.stop_on_error = bool(data.get("stop_on_error", True))

    for path_data in data.get("paths", []):
        block = PathBlock()
        block.name = str(path_data.get("name", ""))
        block.path_id = int(path_data.get("path_id", 0))
        block.path_type = int(path_data["path_type"])
        block.start_index = int(path_data.get("start_index", 1))
        block.end_index = int(path_data.get("end_index", 0))
        block.expected_start_deg = _float_list(
            path_data["expected_start_deg"], 6, "expected_start_deg"
        )
        block.expected_end_deg = _float_list(
            path_data["expected_end_deg"], 6, "expected_end_deg"
        )
        block.wait_after = bool(path_data.get("wait_after", False))
        for frame_data in path_data.get("frames", []):
            frame = PathFrame()
            frame.index = int(frame_data["index"])
            frame.pose = _float_list(frame_data["pose"], 6, "frame.pose")
            block.frames.append(frame)
        for condition_data in path_data.get("conditions", []):
            condition = PathCondition()
            condition.slot = int(condition_data["slot"])
            condition.handler_id = int(condition_data["handler_id"])
            block.conditions.append(condition)
        for node_data in path_data.get("nodes", []):
            node = PathNode()
            node.motion_type = int(node_data["motion_type"])
            node.target = _float_list(node_data["target"], 6, "node.target")
            for field in (
                "linear_speed",
                "rotational_speed",
                "segment_override",
                "tolerance",
                "fly_percent",
                "fly_distance_mm",
                "stress_percent",
            ):
                setattr(node, field, float(node_data.get(field, 0.0)))
            for field in (
                "termination_type",
                "fly_type",
                "fly_trajectory",
                "reference_index",
                "tool_index",
                "condition_mask",
                "condition_mask_back",
            ):
                setattr(node, field, int(node_data.get(field, 0)))
            for field in ("segment_data", "fly", "wait"):
                setattr(node, field, bool(node_data.get(field, False)))
            block.nodes.append(node)
        if not block.nodes:
            raise ValueError(f"PATH {block.name!r} contains no nodes")
        if block.end_index == 0:
            block.end_index = len(block.nodes)
        sequence.paths.append(block)
    if not sequence.paths:
        raise ValueError("PATH sequence contains no paths")
    return sequence


def save_sequence(path: str, sequence: PathSequence) -> None:
    def plain_yaml_value(value):
        if isinstance(value, Mapping):
            return {
                str(key): plain_yaml_value(item)
                for key, item in value.items()
            }
        if isinstance(value, Sequence) and not isinstance(
            value, (str, bytes)
        ):
            return [plain_yaml_value(item) for item in value]
        if isinstance(value, bool):
            return value
        if isinstance(value, Integral):
            return int(value)
        if isinstance(value, Real):
            return float(value)
        return value

    Path(path).write_text(
        yaml.safe_dump(
            plain_yaml_value(sequence_to_dict(sequence)),
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )


def load_sequence(path: str) -> PathSequence:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("PATH sequence file must contain a YAML mapping")
    return sequence_from_dict(data)
