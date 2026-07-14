from arm_tcp_bridge.path_sequence_io import (
    load_sequence,
    save_sequence,
    sequence_from_dict,
    sequence_to_dict,
)


def _data():
    return {
        "format_version": 1,
        "sequence_id": 7,
        "name": "mixed",
        "robot_model": "robot_arm3",
        "coupling_offset": 1.5708,
        "stop_on_error": True,
        "paths": [
            {
                "name": "auto",
                "path_id": 0,
                "path_type": 1,
                "start_index": 1,
                "end_index": 1,
                "expected_start_deg": [0, 0, -90, 0, 0, 0],
                "expected_end_deg": [1, 0, -90, 0, 0, 0],
                "wait_after": True,
                "frames": [],
                "conditions": [],
                "nodes": [
                    {
                        "motion_type": 0,
                        "target": [1, 0, -90, 0, 0, 0],
                        "linear_speed": 0.0,
                        "rotational_speed": 0.0,
                        "segment_override": 5.0,
                        "termination_type": 3,
                        "tolerance": 0.0,
                        "segment_data": True,
                        "fly": False,
                        "fly_type": 0,
                        "fly_percent": 0.0,
                        "fly_distance_mm": 0.0,
                        "fly_trajectory": 0,
                        "stress_percent": 0.0,
                        "reference_index": 0,
                        "tool_index": 0,
                        "condition_mask": 0,
                        "condition_mask_back": 0,
                        "wait": False,
                    }
                ],
            }
        ],
    }


def test_sequence_yaml_model_round_trip():
    message = sequence_from_dict(_data())
    output = sequence_to_dict(message)
    assert output["sequence_id"] == 7
    assert output["paths"][0]["expected_start_deg"][2] == -90.0
    assert output["paths"][0]["nodes"][0]["segment_override"] == 5.0
    assert output["paths"][0]["wait_after"] is True


def test_version_one_defaults_wait_after_to_false():
    data = _data()
    data["format_version"] = 1
    del data["paths"][0]["wait_after"]
    message = sequence_from_dict(data)
    assert message.paths[0].wait_after is False


def test_save_converts_ros_numpy_fixed_arrays(tmp_path):
    destination = tmp_path / "sequence.yaml"
    save_sequence(str(destination), sequence_from_dict(_data()))
    restored = load_sequence(str(destination))
    assert restored.paths[0].wait_after is True


def test_sequence_rejects_empty_paths():
    data = _data()
    data["paths"] = []
    try:
        sequence_from_dict(data)
    except ValueError as exc:
        assert "contains no paths" in str(exc)
    else:
        raise AssertionError("empty sequence was accepted")
