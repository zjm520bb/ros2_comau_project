
import math
from collections.abc import Sequence


def _validate_finite(
    values: Sequence[float],
    expected_length: int,
    name: str,
) -> list[float]:
    if len(values) != expected_length:
        raise ValueError(
            f"{name} requires exactly "
            f"{expected_length} values"
        )

    converted = [float(value) for value in values]

    if not all(
        math.isfinite(value)
        for value in converted
    ):
        raise ValueError(
            f"{name} contains NaN or infinity"
        )

    return converted


def _format_values(
    values: Sequence[float],
) -> str:
    return ",".join(
        f"{float(value):.6f}"
        for value in values
    )


def hello() -> str:
    return "Hello"


def set_base(
    pose: Sequence[float],
) -> str:
    values = _validate_finite(
        pose,
        6,
        "Base pose",
    )
    return f"setBase:{_format_values(values)}"


def set_tool(
    pose: Sequence[float],
) -> str:
    values = _validate_finite(
        pose,
        6,
        "Tool pose",
    )
    return f"setTool:{_format_values(values)}"


def set_uframe(
    pose: Sequence[float],
) -> str:
    values = _validate_finite(
        pose,
        6,
        "User frame",
    )
    return f"setUframe:{_format_values(values)}"


def set_joint_speed(
    percent: float,
) -> str:
    percent = float(percent)

    if not 1.0 <= percent <= 100.0:
        raise ValueError(
            "Joint speed override must be "
            "between 1 and 100 percent"
        )

    return f"setSpeedJnt:{percent:.3f}"


def set_joint_overrides(
    percentages: Sequence[float],
) -> str:
    values = _validate_finite(
        percentages,
        6,
        "Joint overrides",
    )

    if not all(
        1.0 <= value <= 100.0
        for value in values
    ):
        raise ValueError(
            "Every joint override must be "
            "between 1 and 100 percent"
        )

    return (
        "setJointOverrides:"
        f"{_format_values(values)}"
    )


def set_linear_speed(
    speed_mps: float,
) -> str:
    speed_mps = float(speed_mps)

    if (
        not math.isfinite(speed_mps)
        or speed_mps <= 0
    ):
        raise ValueError(
            "Linear speed must be greater than zero"
        )

    return f"setSpeedLin:{speed_mps:.6f}"


def set_acceleration(
    percent: float,
) -> str:
    percent = float(percent)

    if not 1.0 <= percent <= 100.0:
        raise ValueError(
            "Acceleration override must be "
            "between 1 and 100 percent"
        )

    return f"setAcceleration:{percent:.3f}"


def set_deceleration(
    percent: float,
) -> str:
    percent = float(percent)

    if not 1.0 <= percent <= 100.0:
        raise ValueError(
            "Deceleration override must be "
            "between 1 and 100 percent"
        )

    return f"setDeceleration:{percent:.3f}"


def set_orientation(
    mode: int,
) -> str:
    if mode not in (0, 1, 2, 3):
        raise ValueError(
            "Orientation mode must be 0, 1, 2 or 3"
        )

    return f"setOrientation:{mode}"


def set_termination(
    mode: int,
) -> str:
    if mode not in (0, 1, 2, 3, 4):
        raise ValueError(
            "Termination mode must be "
            "0, 1, 2, 3 or 4"
        )

    return f"setTermination:{mode}"


def set_fly_norm(
    fly_percent: float,
) -> str:
    fly_percent = float(fly_percent)

    if not 1.0 <= fly_percent <= 100.0:
        raise ValueError(
            "FLY percentage must be "
            "between 1 and 100"
        )

    return f"setFlyNorm:{fly_percent:.3f}"


def set_fly_cart(
    stress_percent: float,
    trajectory_mode: int,
    fly_distance_mm: float,
) -> str:
    stress_percent = float(stress_percent)
    fly_distance_mm = float(fly_distance_mm)

    if not 1.0 <= stress_percent <= 100.0:
        raise ValueError(
            "Stress percentage must be "
            "between 1 and 100"
        )

    if trajectory_mode not in (0, 1, 2, 3):
        raise ValueError(
            "FLY trajectory mode must be "
            "0, 1, 2 or 3"
        )

    if (
        not math.isfinite(fly_distance_mm)
        or fly_distance_mm < 0
    ):
        raise ValueError(
            "FLY distance must be non-negative"
        )

    return (
        f"setFlyCart:{stress_percent:.3f},"
        f"{trajectory_mode},"
        f"{fly_distance_mm:.3f}"
    )


def move_joint(
    joints_deg: Sequence[float],
) -> str:
    values = _validate_finite(
        joints_deg,
        6,
        "Joint target",
    )

    return f"moveJoint:{_format_values(values)}"


def move_linear(
    pose: Sequence[float],
) -> str:
    values = _validate_finite(
        pose,
        6,
        "Linear target pose",
    )

    return f"moveLin:{_format_values(values)}"


def move_circular(
    via_pose: Sequence[float],
    target_pose: Sequence[float],
) -> str:
    via_values = _validate_finite(
        via_pose,
        6,
        "Circular via pose",
    )
    target_values = _validate_finite(
        target_pose,
        6,
        "Circular target pose",
    )

    values = via_values + target_values

    return (
        f"moveCircular:{_format_values(values)}"
    )


def clear_fly_queue() -> str:
    return "clearFlyQueue"


def add_fly_linear(
    pose: Sequence[float],
) -> str:
    values = _validate_finite(
        pose,
        6,
        "FLY linear point",
    )

    return f"addFlyLin:{_format_values(values)}"


def add_fly_joint(
    joints_deg: Sequence[float],
) -> str:
    values = _validate_finite(
        joints_deg,
        6,
        "FLY joint point",
    )

    return (
        f"addFlyJoint:{_format_values(values)}"
    )


def execute_fly_queue() -> str:
    return "executeFlyQueue"
