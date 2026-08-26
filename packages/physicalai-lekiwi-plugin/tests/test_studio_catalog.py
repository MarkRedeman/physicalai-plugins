from __future__ import annotations

import asyncio
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

from pydantic import BaseModel

_RETIRED_UI_KEYS = {"groups", "group", "widget", "connection_key", "serial_number_key"}
_CONNECTION_UI = [
    {
        "kind": "connection",
        "label": "Select robot",
        "device_discovery": True,
        "bind": {"connection": "connection_string", "serial_number": "serial_number"},
    },
]


def _assert_no_retired_ui_keys(value: object) -> None:
    if isinstance(value, dict):
        assert not _RETIRED_UI_KEYS.intersection(value)
        for nested_value in value.values():
            _assert_no_retired_ui_keys(nested_value)
    elif isinstance(value, list):
        for nested_value in value:
            _assert_no_retired_ui_keys(nested_value)


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


@dataclass
class _PayloadContainer:
    payload: object


class _FakeFactory:
    async def find_port(self, port: object) -> str:
        _ = port
        return "/dev/ttyACM9"


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
    assert defn.asset.root_resolver is not None
    assert (defn.asset.root_resolver() / defn.asset.urdf_relative_path).is_file()
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    assert defn.robot_payload is LeKiwiPayload


def test_payload_model() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    payload = LeKiwiPayload(serial_number="12345")
    assert payload.serial_number == "12345"
    assert payload.baudrate == 1_000_000
    assert payload.disable_torque_on_disconnect is True
    assert payload.connection_string == ""


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


def test_payload_allows_manual_connection_path() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    payload = LeKiwiPayload(connection_string="/dev/ttyACM0")
    assert payload.connection_string == "/dev/ttyACM0"
    assert payload.serial_number == ""


def test_payload_model_rebuild() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    LeKiwiPayload.model_rebuild(raise_errors=True)


def test_payload_schema_configures_serial_connection_picker() -> None:
    from physicalai_studio_plugin import validate_robot_payload_ui

    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    validate_robot_payload_ui(LeKiwiPayload)
    schema = LeKiwiPayload.model_json_schema()

    assert schema["x-physicalai-ui"] == _CONNECTION_UI
    _assert_no_retired_ui_keys(schema)


def test_follower_builder_validates_cross_identity_payload() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import _build_lekiwi_driver

    class OtherPayload(BaseModel):
        connection_string: str
        serial_number: str
        disable_torque_on_disconnect: bool

    driver = asyncio.run(
        _build_lekiwi_driver(
            _PayloadContainer(
                OtherPayload(
                    connection_string="/dev/ttyACM0",
                    serial_number="12345",
                    disable_torque_on_disconnect=True,
                ),
            ),
            _FakeFactory(),
        ),
    )

    assert driver.port == "/dev/ttyACM9"
    assert driver.disable_torque_on_disconnect is True


def test_urdf_path_exists() -> None:
    from physicalai_lekiwi_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
    urdf_file = path / "lekiwi" / "urdf" / "LeKiwi.urdf"
    assert urdf_file.exists(), f"URDF file not found at {urdf_file}"
    assert urdf_file.stat().st_size > 0
