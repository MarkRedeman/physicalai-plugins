from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
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
    assert defn.robot_model is not None


def test_payload_model() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    payload = LeKiwiPayload(serial_number="12345")
    assert payload.serial_number == "12345"
    assert payload.baudrate == 1_000_000
    assert payload.disable_torque_on_disconnect is True
    assert payload.connection_string == ""


def test_payload_requires_serial() -> None:
    from physicalai_lekiwi_plugin.studio_catalog import LeKiwiPayload

    with pytest.raises(Exception):  # noqa: B017, PT011
        LeKiwiPayload()


def test_urdf_path_exists() -> None:
    from physicalai_lekiwi_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
    urdf_file = path / "lekiwi" / "urdf" / "LeKiwi.urdf"
    assert urdf_file.exists(), f"URDF file not found at {urdf_file}"
    assert urdf_file.stat().st_size > 0
