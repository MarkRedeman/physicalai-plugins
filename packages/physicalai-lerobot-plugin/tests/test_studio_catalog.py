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


class _StubFactory:
    def __init__(self) -> None:
        self.calls: list[object] = []

    async def find_port(self, serial_info: object) -> str | None:
        self.calls.append(serial_info)
        return getattr(serial_info, "connection_string", None)


class _StubRobot:
    def __init__(self, payload: object) -> None:
        self.payload = payload


# ── Registration tests ─────────────────────────────────────────────────────


def test_register_plugin_includes_all_registered_robot_and_teleoperator_types() -> None:
    from lerobot.robots.config import RobotConfig
    from lerobot.teleoperators.config import TeleoperatorConfig

    from physicalai_lerobot_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert registry.definitions is not None
    definitions_by_type = {definition.type: definition for definition in registry.definitions}
    expected_follower_types = {
        f"LeRobot_{type_str}_Follower"
        for type_str in RobotConfig.get_known_choices()
        if type_str != "mock_robot"
    }
    expected_leader_types = {
        f"LeRobot_{type_str}_Leader" for type_str in TeleoperatorConfig.get_known_choices()
    }

    assert set(definitions_by_type) == expected_follower_types | expected_leader_types
    assert len(definitions_by_type) == len(registry.definitions)
    for type_str in expected_follower_types:
        assert definitions_by_type[type_str].role == "follower"
    for type_str in expected_leader_types:
        assert definitions_by_type[type_str].role == "leader"


def test_skip_list_excludes_test_only_robots() -> None:
    from physicalai_lerobot_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert registry.definitions is not None
    types = {d.type for d in registry.definitions}
    for skipped in ("LeRobot_mock_robot_Follower",):
        assert skipped not in types


def test_uses_lerobot_native_third_party_plugin_discovery(monkeypatch: pytest.MonkeyPatch) -> None:
    from lerobot.utils import import_utils

    import physicalai_lerobot_plugin.studio_catalog as catalog

    discovered: list[bool] = []
    monkeypatch.setattr(import_utils, "register_third_party_plugins", lambda: discovered.append(True))
    monkeypatch.setattr(catalog, "_LEROBOT_THIRD_PARTY_PLUGINS_IMPORTED", False)

    catalog._ensure_lerobot_third_party_plugins_imported()  # noqa: SLF001
    catalog._ensure_lerobot_third_party_plugins_imported()  # noqa: SLF001

    assert discovered == [True]


def test_each_definition_has_correct_role() -> None:
    from physicalai_lerobot_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert registry.definitions is not None
    for d in registry.definitions:
        role = d.role
        assert role in {"follower", "leader"}
        assert d.type.startswith("LeRobot_")
        assert d.type.endswith("_Follower" if role == "follower" else "_Leader")


# ── Dynamic payload model tests ────────────────────────────────────────────


def test_make_payload_model_for_so100() -> None:
    from lerobot.robots import so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["so100_follower"]
    model = _make_payload_model(config_cls)

    assert issubclass(model, BaseModel)
    assert model.__name__ == "SOFollowerRobotConfigPayload"

    payload = model(port="/dev/ttyACM0")
    assert payload.port == "/dev/ttyACM0"
    assert payload.disable_torque_on_disconnect is True
    assert payload.use_degrees is True


def test_make_payload_model_for_hope_jr_hand() -> None:
    from lerobot.robots import hope_jr  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["hope_jr_hand"]
    model = _make_payload_model(config_cls)

    payload = model(port="/dev/ttyACM1", side="left")
    assert payload.port == "/dev/ttyACM1"
    assert payload.side == "left"


def test_make_payload_model_requires_native_required_fields() -> None:
    from lerobot.robots import so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["so100_follower"]
    model = _make_payload_model(config_cls)

    with pytest.raises(ValidationError):
        model()


def test_make_payload_model_includes_complex_fields_for_bimanual() -> None:
    from lerobot.robots import bi_so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["bi_so_follower"]
    model = _make_payload_model(config_cls)

    assert "left_arm_config" in model.model_fields
    assert "right_arm_config" in model.model_fields
    assert "cameras" in model.model_fields


def test_make_payload_model_for_bimanual_so100() -> None:
    from lerobot.robots import bi_so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["bi_so_follower"]
    model = _make_payload_model(config_cls)

    payload = model(
        left_arm_config={"port": "/dev/ttyACM0"},
        right_arm_config={"port": "/dev/ttyACM1"},
    )
    assert payload.left_arm_config.port == "/dev/ttyACM0"
    assert payload.right_arm_config.port == "/dev/ttyACM1"


def test_make_payload_model_for_reachy2() -> None:
    from lerobot.robots import reachy2  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model

    config_cls = RobotConfig.get_known_choices()["reachy2"]
    model = _make_payload_model(config_cls)

    assert "ip_address" in model.model_fields
    assert "with_mobile_base" in model.model_fields
    assert "cameras" in model.model_fields


@pytest.mark.anyio
async def test_builder_resolves_nested_ports_for_bimanual_config() -> None:
    from unittest.mock import MagicMock, patch

    from lerobot.robots import bi_so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_builder, _make_payload_model

    config_cls = RobotConfig.get_known_choices()["bi_so_follower"]
    payload_cls = _make_payload_model(config_cls)
    builder = _make_builder(config_cls, payload_cls, role="follower")

    payload = payload_cls(
        left_arm_config={"port": "/dev/ttyACM0"},
        right_arm_config={"port": "/dev/ttyACM1"},
    )

    factory = _StubFactory()
    robot = _StubRobot(payload)
    fake_lerobot = MagicMock()
    with patch("lerobot.robots.make_robot_from_config", return_value=fake_lerobot):
        built = await builder(robot, factory)

    assert callable(builder)
    assert len(factory.calls) == 2
    assert built is not None


# ── Builder tests ──────────────────────────────────────────────────────────


def test_make_builder_config_kwargs() -> None:
    from lerobot.robots import so_follower  # noqa: F401
    from lerobot.robots.config import RobotConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_builder, _make_payload_model

    config_cls = RobotConfig.get_known_choices()["so100_follower"]
    payload_cls = _make_payload_model(config_cls)
    follower_builder = _make_builder(config_cls, payload_cls, role="follower")
    assert callable(follower_builder)


def test_make_teleop_builder_config_kwargs() -> None:
    import importlib
    import pkgutil

    import lerobot.teleoperators

    for _importer, modname, is_pkg in pkgutil.walk_packages(
        lerobot.teleoperators.__path__,
        prefix="lerobot.teleoperators.",
    ):
        if "config" in modname and not is_pkg:
            importlib.import_module(modname)

    from lerobot.teleoperators.config import TeleoperatorConfig

    from physicalai_lerobot_plugin.studio_catalog import _make_payload_model, _make_teleop_builder

    teleop_config_cls = TeleoperatorConfig.get_known_choices()["so100_leader"]
    payload_cls = _make_payload_model(teleop_config_cls)
    builder = _make_teleop_builder(teleop_config_cls, payload_cls, role="leader")
    assert callable(builder)


# ── URDF path test ─────────────────────────────────────────────────────────


def test_urdf_path_exists() -> None:
    from physicalai_lerobot_plugin import get_urdf_path

    path = get_urdf_path()
    assert path.exists()
    urdf_file = path / "lerobot" / "urdf" / "lerobot.urdf"
    assert urdf_file.exists(), f"URDF file not found at {urdf_file}"
    assert urdf_file.stat().st_size > 0
