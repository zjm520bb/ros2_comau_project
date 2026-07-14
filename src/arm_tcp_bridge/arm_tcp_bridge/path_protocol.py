import math
from dataclasses import dataclass
from typing import Iterable


PATH_CARTESIAN = 0
PATH_JOINT = 1

MOTION_JOINT = 0
MOTION_LINEAR = 1
MOTION_CIRCULAR = 2
MOTION_SEG_VIA = 3


class PathValidationError(ValueError):
    pass


def _finite(value, name: str) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise PathValidationError(f"{name} must be finite")
    return converted


def _number(value) -> str:
    converted = _finite(value, "PATH numeric value")
    text = f"{converted:.6f}".rstrip("0").rstrip(".")
    return text if text not in ("", "-0") else "0"


def _command(name: str, values: Iterable[object]) -> str:
    return name + ":" + ",".join(
        _number(value) for value in values
    )


@dataclass(frozen=True)
class ValidatedPath:
    path_id: int
    path_type: int
    start_index: int
    end_index: int
    nodes: tuple
    frames: tuple
    conditions: tuple


def validate_path_goal(goal, max_nodes: int) -> ValidatedPath:
    path_id = int(goal.path_id)
    path_type = int(goal.path_type)
    nodes = tuple(goal.nodes)
    frames = tuple(goal.frames)
    conditions = tuple(goal.conditions)

    if not 0 < path_id <= 2147483647:
        raise PathValidationError(
            "path_id must fit a positive C4G INTEGER"
        )
    if path_type not in (PATH_CARTESIAN, PATH_JOINT):
        raise PathValidationError("path_type must be CARTESIAN or JOINT")
    if not nodes:
        raise PathValidationError("PATH must contain at least one node")
    if len(nodes) > max_nodes:
        raise PathValidationError(
            f"PATH has {len(nodes)} nodes; maximum is {max_nodes}"
        )

    start_index = int(goal.start_index) or 1
    end_index = int(goal.end_index) or len(nodes)
    if not 1 <= start_index <= len(nodes):
        raise PathValidationError("start_index is outside the PATH")
    if not 1 <= end_index <= len(nodes):
        raise PathValidationError("end_index is outside the PATH")

    frame_indexes = set()
    for frame in frames:
        index = int(frame.index)
        if not 1 <= index <= 7:
            raise PathValidationError("frame index must be within 1..7")
        if index in frame_indexes:
            raise PathValidationError(f"duplicate frame index {index}")
        frame_indexes.add(index)
        if len(frame.pose) != 6:
            raise PathValidationError("frame pose must contain six values")
        for value in frame.pose:
            _finite(value, "frame pose")

    condition_slots = set()
    for condition in conditions:
        slot = int(condition.slot)
        if not 1 <= slot <= 32:
            raise PathValidationError("condition slot must be within 1..32")
        if slot in condition_slots:
            raise PathValidationError(f"duplicate condition slot {slot}")
        if int(condition.handler_id) <= 0:
            raise PathValidationError("condition handler_id must be positive")
        condition_slots.add(slot)

    previous_type = None
    for index, node in enumerate(nodes, start=1):
        motion_type = int(node.motion_type)
        if motion_type not in (
            MOTION_JOINT,
            MOTION_LINEAR,
            MOTION_CIRCULAR,
            MOTION_SEG_VIA,
        ):
            raise PathValidationError(
                f"node {index} has an invalid motion_type"
            )
        if path_type == PATH_JOINT and motion_type != MOTION_JOINT:
            raise PathValidationError(
                "JOINT PATH may contain only JOINT nodes"
            )
        if path_type == PATH_CARTESIAN:
            if motion_type == MOTION_CIRCULAR and previous_type != MOTION_SEG_VIA:
                raise PathValidationError(
                    f"CIRCULAR node {index} must follow a SEG_VIA node"
                )
            if previous_type == MOTION_SEG_VIA and motion_type != MOTION_CIRCULAR:
                raise PathValidationError(
                    f"SEG_VIA node {index - 1} must be followed by CIRCULAR"
                )
        if len(node.target) != 6:
            raise PathValidationError(
                f"node {index} target must contain six values"
            )
        for value in node.target:
            _finite(value, f"node {index} target")
        if not 0.0 < float(node.segment_override) <= 100.0:
            raise PathValidationError(
                f"node {index} segment_override must be within (0, 100]"
            )
        if not 0 <= int(node.termination_type) <= 4:
            raise PathValidationError(
                f"node {index} termination_type must be within 0..4"
            )
        if not 0 <= int(node.fly_type) <= 1:
            raise PathValidationError(
                f"node {index} fly_type must be 0 or 1"
            )
        if not 0 <= int(node.fly_trajectory) <= 3:
            raise PathValidationError(
                f"node {index} fly_trajectory must be within 0..3"
            )
        if node.fly and not 1.0 <= float(node.fly_percent) <= 100.0:
            raise PathValidationError(
                f"node {index} fly_percent must be within 1..100"
            )
        if float(node.fly_distance_mm) < 0.0:
            raise PathValidationError(
                f"node {index} fly_distance_mm must be non-negative"
            )
        if not 0 <= int(node.reference_index) <= 7:
            raise PathValidationError(
                f"node {index} reference_index must be within 0..7"
            )
        if not 0 <= int(node.tool_index) <= 7:
            raise PathValidationError(
                f"node {index} tool_index must be within 0..7"
            )
        if path_type == PATH_JOINT and (
            int(node.reference_index) != 0 or int(node.tool_index) != 0
        ):
            raise PathValidationError(
                "JOINT PATH nodes cannot select reference or tool frames"
            )
        if (
            int(node.reference_index) != 0
            and int(node.reference_index) not in frame_indexes
        ):
            raise PathValidationError(
                f"node {index} references an undefined frame"
            )
        if (
            int(node.tool_index) != 0
            and int(node.tool_index) not in frame_indexes
        ):
            raise PathValidationError(
                f"node {index} references an undefined tool frame"
            )
        for mask_name, mask_value in (
            ("condition_mask", int(node.condition_mask)),
            ("condition_mask_back", int(node.condition_mask_back)),
        ):
            unsigned_mask = mask_value & 0xFFFFFFFF
            for bit in range(32):
                if (
                    unsigned_mask & (1 << bit)
                    and bit + 1 not in condition_slots
                ):
                    raise PathValidationError(
                        f"node {index} {mask_name} selects "
                        f"undefined condition slot {bit + 1}"
                    )
        if path_type == PATH_JOINT and node.fly and int(node.fly_type) != 0:
            raise PathValidationError(
                "JOINT PATH supports only FLY_NORM"
            )
        previous_type = motion_type

    if path_type == PATH_CARTESIAN and previous_type == MOTION_SEG_VIA:
        raise PathValidationError("PATH cannot end with SEG_VIA")
    if nodes[start_index - 1].motion_type == MOTION_CIRCULAR:
        raise PathValidationError(
            "execution range cannot start at a CIRCULAR node"
        )
    if nodes[end_index - 1].motion_type == MOTION_SEG_VIA:
        raise PathValidationError(
            "execution range cannot end at a SEG_VIA node"
        )

    return ValidatedPath(
        path_id=path_id,
        path_type=path_type,
        start_index=start_index,
        end_index=end_index,
        nodes=nodes,
        frames=frames,
        conditions=conditions,
    )


def build_upload_commands(path: ValidatedPath) -> list[str]:
    commands = [
        _command(
            "beginPath",
            (
                path.path_id,
                path.path_type,
                len(path.nodes),
                path.start_index,
                path.end_index,
            ),
        )
    ]

    for frame in path.frames:
        commands.append(
            _command(
                "setPathFrame",
                (path.path_id, frame.index, *frame.pose),
            )
        )

    for condition in path.conditions:
        commands.append(
            _command(
                "setPathCondition",
                (
                    path.path_id,
                    condition.slot,
                    condition.handler_id,
                ),
            )
        )

    for index, node in enumerate(path.nodes, start=1):
        commands.extend(
            [
                _command(
                    "pathNodeTarget",
                    (
                        path.path_id,
                        index,
                        node.motion_type,
                        *node.target,
                    ),
                ),
                _command(
                    "pathNodeMotion",
                    (
                        path.path_id,
                        index,
                        node.linear_speed,
                        node.rotational_speed,
                        node.segment_override,
                        node.termination_type,
                        node.tolerance,
                        int(node.segment_data),
                    ),
                ),
                _command(
                    "pathNodeBlend",
                    (
                        path.path_id,
                        index,
                        int(node.fly),
                        node.fly_type,
                        node.fly_percent,
                        node.fly_distance_mm,
                        node.fly_trajectory,
                        node.stress_percent,
                    ),
                ),
                _command(
                    "pathNodeSync",
                    (
                        path.path_id,
                        index,
                        node.reference_index,
                        node.tool_index,
                        node.condition_mask,
                        node.condition_mask_back,
                        int(node.wait),
                    ),
                ),
                _command("commitPathNode", (path.path_id, index)),
            ]
        )

    commands.append(
        _command(
            "commitPath",
            (path.path_id, len(path.nodes)),
        )
    )
    return commands


def execute_path_command(path: ValidatedPath) -> str:
    return _command(
        "executePath",
        (path.path_id, path.start_index, path.end_index),
    )
