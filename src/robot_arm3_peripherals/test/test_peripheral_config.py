from pathlib import Path

import yaml


PACKAGE_PATH = Path(__file__).resolve().parents[1]


def _load_config():
    with open(PACKAGE_PATH / "config" / "peripherals.yaml", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _load_schema():
    with open(PACKAGE_PATH / "config" / "peripheral_value_schema.yaml", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _load_process():
    with open(PACKAGE_PATH / "config" / "process_demo.yaml", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_config_contains_named_peripherals():
    config = _load_config()
    peripherals = {
        entry["id"]: entry
        for entry in config["peripherals"]
    }

    assert peripherals["schleiftrog"]["display_name"] == "Schleiftrog"
    assert peripherals["motorspindel"]["display_name"] == "Motorspindel"
    assert (
        peripherals["fliehkraftgleitschleifanlage"]["display_name"]
        == "Fliehkraftgleitschleifanlage"
    )
    assert (
        peripherals["festo_pneumatiksteuerung"]["display_name"]
        == "Festo Pneumatiksteuerung"
    )


def test_collision_objects_are_bound_to_peripherals():
    config = _load_config()
    peripherals = {
        entry["id"]: entry
        for entry in config["peripherals"]
    }

    assert peripherals["schleiftrog"]["collision_objects"] == [
        "schleiftrog_a",
        "schleiftrog_b",
    ]
    assert peripherals["motorspindel"]["collision_objects"] == [
        "motorspindel",
    ]
    assert peripherals["fliehkraftgleitschleifanlage"]["collision_objects"] == [
        "fliehkraftgleitschleifanlage",
    ]
    assert peripherals["festo_pneumatiksteuerung"]["collision_objects"] == [
        "festo_pneumatiksteuerung",
    ]


def test_peripheral_values_follow_schema_keys():
    config = _load_config()
    schema = _load_schema()["schemas"]

    for peripheral in config["peripherals"]:
        device_type = peripheral["type"]
        fixed_keys = set(schema[device_type]["fixed_keys"])
        configured_keys = set(peripheral.get("values", {}))
        assert configured_keys == fixed_keys


def test_process_demo_uses_state_waits_after_async_commands():
    process = _load_process()
    steps = process["steps"]

    assert any(
        step.get("type") == "wait_state"
        and step.get("device_id") == "fliehkraftgleitschleifanlage"
        and step.get("key") == "safe"
        and step.get("value") is True
        for step in steps
    )
    assert any(
        step.get("type") == "wait_state"
        and step.get("device_id") == "festo_pneumatiksteuerung"
        and step.get("key") == "clamped"
        and step.get("value") is True
        for step in steps
    )
    assert any(
        step.get("type") == "wait_state"
        and step.get("device_id") == "motorspindel"
        and step.get("key") == "actual_rpm"
        and step.get("min") == 2900
        and step.get("max") == 3100
        for step in steps
    )
