from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

import pytest
from pydantic import ValidationError


@dataclass
class _FakeRegistry:
    definitions: list[object] | None = None

    def register_robot(self, definition: object) -> None:
        if self.definitions is None:
            self.definitions = []
        self.definitions.append(definition)

    def register_many(self, definitions: Sequence[object]) -> None:
        if self.definitions is None:
            self.definitions = []
        self.definitions.extend(definitions)


def test_register_plugin() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert registry.definitions is not None
    assert len(registry.definitions) == 2

    types = [(d.type, d.role) for d in registry.definitions]
    assert ("LeKiwi_Follower", "follower") in types
    assert ("LeKiwi_Leader", "leader") in types

    defn = registry.definitions[0]
    assert defn.asset is not None
    assert defn.asset.urdf_relative_path == Path("lekiwi/urdf/LeKiwi.urdf")
    assert defn.asset.packages == {"lekiwi": Path("lekiwi")}
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    assert defn.robot_payload is LeKiwiPayload


def test_payload_model() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    payload = LeKiwiPayload(serial_number="12345")
    assert payload.serial_number == "12345"
    assert payload.baudrate == 1_000_000
    assert payload.disable_torque_on_disconnect is False
    assert payload.connection_string == ""


def test_payload_requires_serial_or_connection_string() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    with pytest.raises(ValidationError):
        LeKiwiPayload()

    payload = LeKiwiPayload(connection_string="/dev/ttyACM0")
    assert payload.connection_string == "/dev/ttyACM0"
    assert payload.serial_number == ""


def test_payload_json_schema_has_ui_metadata() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    schema = LeKiwiPayload.model_json_schema()
    assert "x-physicalai-ui" in schema
    ui = schema["x-physicalai-ui"]
    assert "groups" in ui
    assert "connection" in ui["groups"]
    assert ui["groups"]["connection"]["device_discovery"] is True

    conn_string_field = schema["properties"]["connection_string"]
    assert "x-physicalai-ui" in conn_string_field
    assert conn_string_field["x-physicalai-ui"]["group"] == "connection"

    serial_field = schema["properties"]["serial_number"]
    assert "x-physicalai-ui" in serial_field
    assert serial_field["x-physicalai-ui"]["group"] == "connection"


def test_payload_with_calibration() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    payload = LeKiwiPayload(
        serial_number="12345",
        calibration={
            "arm_shoulder_pan": {
                "id": 1,
                "drive_mode": 0,
                "homing_offset": 2048,
                "range_min": 100,
                "range_max": 4000,
            },
            "arm_shoulder_lift": {
                "id": 2,
                "drive_mode": 1,
                "homing_offset": 1024,
                "range_min": 200,
                "range_max": 3900,
            },
            "arm_elbow_flex": {
                "id": 3,
                "drive_mode": 0,
                "homing_offset": 1024,
                "range_min": 150,
                "range_max": 3800,
            },
            "arm_wrist_flex": {
                "id": 4,
                "drive_mode": 1,
                "homing_offset": 1024,
                "range_min": 150,
                "range_max": 3800,
            },
            "arm_wrist_roll": {
                "id": 5,
                "drive_mode": 0,
                "homing_offset": 1024,
                "range_min": 150,
                "range_max": 3800,
            },
            "arm_gripper": {
                "id": 6,
                "drive_mode": 1,
                "homing_offset": 1024,
                "range_min": 300,
                "range_max": 3500,
            },
        },
    )

    assert payload.calibration is not None
    assert payload.calibration["arm_shoulder_pan"].id == 1


def test_payload_requires_serial() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    with pytest.raises(ValidationError):
        LeKiwiPayload()


def test_payload_model_rebuild() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    LeKiwiPayload.model_rebuild(raise_errors=True)


def test_urdf_path_exists() -> None:
    from physicalai_lekiwi_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
    urdf_file = path / "lekiwi" / "urdf" / "LeKiwi.urdf"
    assert urdf_file.exists(), f"URDF file not found at {urdf_file}"
    assert urdf_file.stat().st_size > 0
