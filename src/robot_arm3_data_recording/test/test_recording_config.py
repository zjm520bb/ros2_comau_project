from pathlib import Path

import yaml


PACKAGE_PATH = Path(__file__).resolve().parents[1]


def _load_config(name):
    with open(PACKAGE_PATH / "config" / name, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_sim_recording_includes_process_and_sensor_topics():
    topics = set(_load_config("record_sim_topics.yaml")["recording"]["topics"])

    assert "/peripherals/events" in topics
    assert "/peripherals/motorspindel/state" in topics
    assert "/peripherals/fliehkraftgleitschleifanlage/state" in topics
    assert "/sensors/tool_wrench" in topics
    assert "/sensors/process_pressure" in topics
    assert "/sim/arm/execute/_action/status" in topics


def test_real_recording_includes_real_robot_topics():
    topics = set(_load_config("record_real_topics.yaml")["recording"]["topics"])

    assert "/c4g/joint_states" in topics
    assert "/arm/execute/_action/status" in topics
    assert "/peripherals/events" in topics
    assert "/peripherals/fliehkraftgleitschleifanlage/state" in topics
    assert "/sensors/process_pressure" in topics


def test_recording_topic_lists_are_non_empty_strings():
    for config_name in ["record_sim_topics.yaml", "record_real_topics.yaml"]:
        topics = _load_config(config_name)["recording"]["topics"]
        assert topics
        assert all(isinstance(topic, str) and topic.startswith("/") for topic in topics)
