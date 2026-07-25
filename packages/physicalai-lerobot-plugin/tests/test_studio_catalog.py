from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

import pytest
from pydantic import BaseModel, ValidationError


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


# ── Registration tests ─────────────────────────────────────────────────────


def test_register_plugin() -> None:
    from physicalai_lerobot_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert registry.definitions is not None
    types = {d.type for d in registry.definitions}
    assert "LeRobot_so100_follower" in types
    assert "LeRobot_so101_follower" in types
    assert "LeRobot_koch_follower" in types
    assert "LeRobot_omx_follower" in types
    assert "LeRobot_hope_jr_hand" in types
    assert "LeRobot_hope_jr_arm" in types
    assert "LeRobot_openarm_follower" in types
    assert "LeRobot_rebot_b601_follower" in types
    assert "LeRobot_reachy2" in types
    assert "LeRobot_earthrover_mini_plus" in types


def test_skip_list_excludes_bimanual_and_deferred() -> None:
    from physicalai_lerobot_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert registry.definitions is not None
    types = {d.type for d in registry.definitions}
    assert "LeRobot_bi_so_follower" not in types
    assert "LeRobot_bi_rebot_b601_follower" not in types
    assert "LeRobot_bi_openarm_follower" not in types
    assert "LeRobot_lekiwi" not in types
    assert "LeRobot_unitree_g1" not in types


# ── Dynamic payload model tests ────────────────────────────────────────────


def test_make_payload_model_for_so100() -> None:
    from lerobot.robots import so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["so100_follower"]
    model = _make_payload_model(config_cls)

    assert issubclass(model, BaseModel)
    assert model.__name__ == "SOFollowerRobotConfigPayload"

    payload = model(serial_number="SN-001", port="/dev/ttyACM0")
    assert payload.serial_number == "SN-001"
    assert payload.port == "/dev/ttyACM0"
    assert payload.disable_torque_on_disconnect is True
    assert payload.use_degrees is True


def test_make_payload_model_for_hope_jr_hand() -> None:
    from lerobot.robots import hope_jr  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["hope_jr_hand"]
    model = _make_payload_model(config_cls)

    payload = model(serial_number="SN-002", port="/dev/ttyACM1", side="left")
    assert payload.port == "/dev/ttyACM1"
    assert payload.side == "left"


def test_make_payload_model_requires_serial_or_connection_string() -> None:
    from lerobot.robots import so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["so100_follower"]
    model = _make_payload_model(config_cls)

    with pytest.raises(ValidationError):
        model()


def test_make_payload_model_skips_complex_fields() -> None:
    from lerobot.robots import hope_jr  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["hope_jr_hand"]
    model = _make_payload_model(config_cls)

    assert "cameras" not in model.model_fields
    assert "calibration_dir" not in model.model_fields


def test_make_payload_model_json_schema_has_ui_metadata() -> None:
    from lerobot.robots import so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["so100_follower"]
    model = _make_payload_model(config_cls)
    schema = model.model_json_schema()

    assert "x-physicalai-ui" in schema
    ui = schema["x-physicalai-ui"]
    assert "groups" in ui
    assert "connection" in ui["groups"]
    assert ui["groups"]["connection"]["device_discovery"] is True

    conn_string_field = schema["properties"]["connection_string"]
    assert "x-physicalai-ui" in conn_string_field
    assert conn_string_field["x-physicalai-ui"]["widget"] == "device-selector"


def test_make_payload_model_for_reachy2() -> None:
    from lerobot.robots import reachy2  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["reachy2"]
    model = _make_payload_model(config_cls)

    assert "port" in model.model_fields
    assert "with_mobile_base" in model.model_fields
    assert "cameras" not in model.model_fields


# ── Builder tests ──────────────────────────────────────────────────────────


def test_make_builder_config_kwargs() -> None:
    from lerobot.robots import so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_builder, _make_payload_model

    config_cls = RobotConfig.get_known_choices()["so100_follower"]
    payload_cls = _make_payload_model(config_cls)
    builder = _make_builder(config_cls, payload_cls)

    assert callable(builder)


# ── URDF path test ─────────────────────────────────────────────────────────


def test_urdf_path_exists() -> None:
    from physicalai_lerobot_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
    urdf_file = path / "lerobot" / "urdf" / "lerobot.urdf"
    assert urdf_file.exists(), f"URDF file not found at {urdf_file}"
    assert urdf_file.stat().st_size > 0
