import importlib.util
from pathlib import Path

import pytest


PACKAGE_PATH = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_PATH / "scripts" / "moveit_environment_boxes.py"
SPEC = importlib.util.spec_from_file_location("moveit_environment_boxes", SCRIPT_PATH)
ENVIRONMENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENVIRONMENT)


def _load():
    return ENVIRONMENT.load_environment(
        str(PACKAGE_PATH / "config" / "environment_boxes.yaml"),
        str(PACKAGE_PATH / "urdf"),
    )


def test_environment_contains_unique_boxes_in_c4g_base():
    frame_id, padding, boxes = _load()
    identifiers = {box.identifier for box in boxes}

    assert frame_id == "c4g_base"
    assert padding == pytest.approx(0.02)
    assert len(boxes) == 6
    assert len(identifiers) == len(boxes)
    assert "fliehkraftgleitschleifanlage" in identifiers
    assert "festo_pneumatiksteuerung" in identifiers


def test_motorspindel_touches_ground_before_planning_padding():
    _, padding, boxes = _load()
    motorspindel = next(box for box in boxes if box.identifier == "motorspindel")

    assert motorspindel.pose[:3] == pytest.approx((-2.5, 2.5, 0.5))
    assert motorspindel.size == pytest.approx((1.5 + 2 * padding, 0.8 + 2 * padding, 1.0 + 2 * padding))
    physical_bottom = motorspindel.pose[2] - (motorspindel.size[2] - 2 * padding) / 2.0
    assert physical_bottom == pytest.approx(0.0)


def test_all_environment_boxes_are_rotated_into_c4g_base():
    _, _, boxes = _load()
    for box in boxes:
        assert box.pose[3:] == pytest.approx((0.0, 0.0, 1.0, 0.0), abs=1e-12)


def test_box_spec_becomes_moveit_collision_object():
    frame_id, _, boxes = _load()
    box = boxes[0]
    collision = ENVIRONMENT.collision_object(box, frame_id)

    assert collision.header.frame_id == "c4g_base"
    assert collision.id == box.identifier
    assert collision.operation == collision.ADD
    assert len(collision.primitives) == 1
    assert collision.primitives[0].type == collision.primitives[0].BOX
    assert collision.primitives[0].dimensions == pytest.approx(box.size)
