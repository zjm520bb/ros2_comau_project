from pathlib import Path

import yaml


PACKAGE_PATH = Path(__file__).resolve().parents[1]


def _load_config():
    with open(PACKAGE_PATH / "config" / "sensors.yaml", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_config_contains_virtual_sensor_topics():
    config = _load_config()
    sensors = {
        entry["id"]: entry
        for entry in config["sensors"]
    }

    assert sensors["tool_wrench"]["type"] == "wrench"
    assert sensors["tool_wrench"]["topic"] == "/sensors/tool_wrench"
    assert sensors["process_pressure"]["type"] == "fluid_pressure"
    assert sensors["process_pressure"]["topic"] == "/sensors/process_pressure"


def test_virtual_sensor_rates_are_positive():
    config = _load_config()

    for sensor in config["sensors"]:
        assert float(sensor["publish_rate_hz"]) > 0.0
