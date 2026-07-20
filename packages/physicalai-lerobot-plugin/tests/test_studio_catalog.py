from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

import pytest


@dataclass
class _FakeRegistry:
    definitions: list[object] | None = None

    def register(self, definition: object) -> None:
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


def test_payload_requires_fields() -> None:
    from pydantic import ValidationError

    from physicalai_lerobot_plugin.studio_catalog import LeRobotPayload

    with pytest.raises(ValidationError):
        LeRobotPayload()

    with pytest.raises(ValidationError):
        LeRobotPayload(robot_type="so100_follower")

    with pytest.raises(ValidationError):
        LeRobotPayload(robot_type="so100_follower", port="/dev/ttyACM0")


def test_urdf_path_exists() -> None:
    from physicalai_lerobot_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
    urdf_file = path / "lerobot" / "urdf" / "lerobot.urdf"
    assert urdf_file.exists(), f"URDF file not found at {urdf_file}"
    assert urdf_file.stat().st_size > 0
