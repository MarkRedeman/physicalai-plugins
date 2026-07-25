from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

import pytest


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
    from physicalai_lerobot_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert registry.definitions is not None
    assert len(registry.definitions) == 2

    types = {}
    for d in registry.definitions:
        types[d.type] = d.role
    assert types == {"LeRobot_Follower": "follower", "LeRobot_Leader": "leader"}


def test_payload_defaults() -> None:
    from physicalai_lerobot_plugin.studio_catalog import LeRobotPayload

    payload = LeRobotPayload(
        robot_type="so100_follower",
        port="/dev/ttyACM0",
        joint_order=["shoulder_pan", "shoulder_lift", "elbow_flex"],
    )
    assert payload.robot_type == "so100_follower"
    assert payload.port == "/dev/ttyACM0"
    assert payload.joint_order == ["shoulder_pan", "shoulder_lift", "elbow_flex"]
    assert payload.obs_position_keys is None
    assert payload.act_position_keys is None
    assert payload.disable_torque_on_disconnect is True
    assert payload.serial_number == ""


def test_payload_accepts_serial_number() -> None:
    from physicalai_lerobot_plugin.studio_catalog import LeRobotPayload

    payload = LeRobotPayload(
        robot_type="so101_follower",
        port="/dev/ttyACM1",
        serial_number="SN-001",
        joint_order=["shoulder_pan", "shoulder_lift", "elbow_flex"],
    )
    assert payload.serial_number == "SN-001"


def test_payload_requires_fields() -> None:
    from pydantic import ValidationError

    from physicalai_lerobot_plugin.studio_catalog import LeRobotPayload

    with pytest.raises(ValidationError):
        LeRobotPayload()

    with pytest.raises(ValidationError):
        LeRobotPayload(robot_type="so100_follower")

    with pytest.raises(ValidationError):
        LeRobotPayload(robot_type="so100_follower", port="/dev/ttyACM0")


def test_payload_rejects_invalid_robot_type() -> None:
    from pydantic import ValidationError

    from physicalai_lerobot_plugin.studio_catalog import LeRobotPayload

    with pytest.raises(ValidationError):
        LeRobotPayload(
            robot_type="invalid_type",  # type: ignore[arg-type]
            port="/dev/ttyACM0",
            joint_order=["a", "b", "c"],
        )


def test_payload_json_schema_has_ui_metadata() -> None:
    from physicalai_lerobot_plugin.studio_catalog import LeRobotPayload

    schema = LeRobotPayload.model_json_schema()
    assert "x-physicalai-ui" in schema
    ui = schema["x-physicalai-ui"]
    assert "groups" in ui
    assert "connection" in ui["groups"]

    port_field = schema["properties"]["port"]
    assert "x-physicalai-ui" in port_field
    assert port_field["x-physicalai-ui"]["group"] == "connection"

    robot_type_field = schema["properties"]["robot_type"]
    assert "enum" in robot_type_field
    assert robot_type_field["enum"] == ["so100_follower", "so101_follower"]


def test_payload_model_rebuild() -> None:
    from physicalai_lerobot_plugin.studio_catalog import LeRobotPayload

    LeRobotPayload.model_rebuild(raise_errors=True)


def test_urdf_path_exists() -> None:
    from physicalai_lerobot_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
    urdf_file = path / "lerobot" / "urdf" / "lerobot.urdf"
    assert urdf_file.exists(), f"URDF file not found at {urdf_file}"
    assert urdf_file.stat().st_size > 0
