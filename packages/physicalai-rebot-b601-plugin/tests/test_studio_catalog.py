from __future__ import annotations

from pathlib import Path

import pytest

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


class _StubFactory:
    def __init__(self, port: str | None = "/dev/ttyACM0") -> None:
        self._port = port

    async def find_port(self, serial_info: object) -> str | None:
        _ = serial_info
        return self._port


class _StubRobot:
    def __init__(self, payload: object) -> None:
        self.payload = payload


class _FakeRegistry:
    def __init__(self) -> None:
        self.definitions: list = []

    def register_robot(self, definition: object) -> None:
        self.definitions.append(definition)


def test_definitions_count() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    defs = _definitions()
    assert len(defs) == 2


def test_definitions_have_expected_types() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    types = {d.type for d in _definitions()}
    assert types == {"ReBot_B601_DM_Follower", "ReBot_Arm102_Leader"}


def test_register_physicalai_studio_plugin() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)
    assert len(registry.definitions) == 2


def test_dm_follower_structure() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    dm = next(d for d in _definitions() if d.type == "ReBot_B601_DM_Follower")

    assert dm.display_name == "ReBot B601 DM Follower"
    assert dm.role == "follower"
    assert dm.asset is not None
    assert dm.asset.urdf_relative_path == Path("rebot-b601-dm/urdf/reBot-DevArm_fixend.urdf")
    assert dm.asset.packages == {"rebot-b601-dm": Path("rebot-b601-dm")}
    assert dm.asset.root_resolver is not None
    assert (dm.asset.root_resolver() / dm.asset.urdf_relative_path).is_file()


def test_arm102_leader_structure() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    arm102 = next(d for d in _definitions() if d.type == "ReBot_Arm102_Leader")

    assert arm102.display_name == "ReBot Arm102 Leader"
    assert arm102.role == "leader"
    assert arm102.asset is not None
    assert arm102.asset.urdf_relative_path == Path("stararm102/urdf/stararm102_description.urdf")
    assert arm102.asset.packages == {"stararm102": Path("stararm102")}
    assert arm102.asset.root_resolver is not None
    assert (arm102.asset.root_resolver() / arm102.asset.urdf_relative_path).is_file()


def test_definition_robot_type_property() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    for d in _definitions():
        assert d.type in {"ReBot_B601_DM_Follower", "ReBot_Arm102_Leader"}


def test_dm_follower_has_robot_builder() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    dm = next(d for d in _definitions() if d.type == "ReBot_B601_DM_Follower")
    assert callable(dm.robot_builder)
    from physicalai_rebot_b601_plugin.studio_catalog import ReBotB601DMPayload

    assert dm.robot_payload is ReBotB601DMPayload
    assert dm.adapter_options.include_velocities is True
    assert dm.adapter_options.external_effort_gain is None
    assert dm.adapter_options.goal_time_scale == 1.0


def test_arm102_leader_has_robot_builder() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    arm102 = next(d for d in _definitions() if d.type == "ReBot_Arm102_Leader")
    assert callable(arm102.robot_builder)
    from physicalai_rebot_b601_plugin.studio_catalog import ReBotArm102Payload

    assert arm102.robot_payload is ReBotArm102Payload
    assert arm102.adapter_options.include_velocities is False
    assert arm102.adapter_options.external_effort_gain is None
    assert arm102.adapter_options.goal_time_scale == 1.0


def test_rebot_b601_dm_payload_defaults() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import ReBotB601DMPayload

    payload = ReBotB601DMPayload(serial_number="SN-001")
    assert payload.connection_string == ""
    assert payload.serial_number == "SN-001"
    assert payload.can_adapter == "damiao"
    assert payload.dm_serial_baud == 921600
    assert payload.disable_torque_on_disconnect is True
    assert payload.force_pos_torque_ratio == 0.1
    assert payload.max_velocity == 1.0


@pytest.mark.parametrize("max_velocity", [1.5, 2.0, 0.0, -0.5])
def test_rebot_b601_dm_payload_rejects_out_of_range_max_velocity(max_velocity: float) -> None:
    from pydantic import ValidationError

    from physicalai_rebot_b601_plugin.studio_catalog import ReBotB601DMPayload

    # 1.0 is the arm's rated ceiling; a Studio config must not exceed it.
    with pytest.raises(ValidationError):
        ReBotB601DMPayload(serial_number="SN-001", max_velocity=max_velocity)


def test_rebot_b601_dm_payload_accepts_scaling_down() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import ReBotB601DMPayload

    assert ReBotB601DMPayload(serial_number="SN-001", max_velocity=0.25).max_velocity == 0.25


def test_rebot_arm102_payload_defaults() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import ReBotArm102Payload

    payload = ReBotArm102Payload(serial_number="SN-002")
    assert payload.connection_string == ""
    assert payload.serial_number == "SN-002"
    assert payload.baudrate == 1_000_000
    assert payload.unlock_on_connect is True
    assert payload.reset_multi_turn_on_connect is True
    assert payload.zero_on_connect is False


def test_payload_models_rebuild() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import ReBotArm102Payload, ReBotB601DMPayload

    ReBotB601DMPayload.model_rebuild(raise_errors=True)
    ReBotArm102Payload.model_rebuild(raise_errors=True)


@pytest.mark.parametrize(
    "payload_cls",
    ["ReBotB601DMPayload", "ReBotArm102Payload"],
)
def test_payload_requires_connection_identifier(payload_cls: str) -> None:
    from pydantic import ValidationError

    from physicalai_rebot_b601_plugin.studio_catalog import ReBotArm102Payload, ReBotB601DMPayload

    payload_models = {
        "ReBotB601DMPayload": ReBotB601DMPayload,
        "ReBotArm102Payload": ReBotArm102Payload,
    }
    with pytest.raises(ValidationError, match="connection_string or serial_number"):
        payload_models[payload_cls]()


@pytest.mark.parametrize("payload_name", ["ReBotB601DMPayload", "ReBotArm102Payload"])
def test_payload_schemas_configure_serial_connection_picker(payload_name: str) -> None:
    from physicalai_studio_plugin import validate_robot_payload_ui

    from physicalai_rebot_b601_plugin.studio_catalog import ReBotArm102Payload, ReBotB601DMPayload

    payload_models = {
        "ReBotB601DMPayload": ReBotB601DMPayload,
        "ReBotArm102Payload": ReBotArm102Payload,
    }
    payload_model = payload_models[payload_name]

    validate_robot_payload_ui(payload_model)
    schema = payload_model.model_json_schema()

    assert schema["x-physicalai-ui"] == _CONNECTION_UI
    _assert_no_retired_ui_keys(schema)


@pytest.mark.anyio
async def test_build_rebot_b601_dm_from_pydantic_payload() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import (
        ReBotB601DMPayload,
        _build_rebot_b601_dm_driver,
    )

    payload = ReBotB601DMPayload(serial_number="DM-001", can_adapter="socketcan")
    robot = _StubRobot(payload)
    factory = _StubFactory(port="/dev/ttyACM0")
    driver = await _build_rebot_b601_dm_driver(robot, factory)
    assert driver is not None


@pytest.mark.anyio
async def test_build_rebot_b601_dm_from_dict_payload() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import (
        _build_rebot_b601_dm_driver,
    )

    payload: dict[str, object] = {
        "serial_number": "DM-002",
        "can_adapter": "damiao",
        "force_pos_torque_ratio": 0.2,
        "max_velocity": 0.5,
    }
    robot = _StubRobot(payload)
    factory = _StubFactory(port="/dev/ttyACM1")
    driver = await _build_rebot_b601_dm_driver(robot, factory)
    assert driver is not None
    assert driver.max_velocity == 0.5


@pytest.mark.anyio
async def test_build_rebot_b601_dm_port_not_found() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import (
        ReBotB601DMPayload,
        _build_rebot_b601_dm_driver,
    )

    payload = ReBotB601DMPayload(serial_number="DM-MISSING")
    robot = _StubRobot(payload)
    factory = _StubFactory(port=None)
    with pytest.raises(RuntimeError, match="Robot not found"):
        await _build_rebot_b601_dm_driver(robot, factory)


@pytest.mark.anyio
async def test_build_rebot_arm102_from_pydantic_payload() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import (
        ReBotArm102Payload,
        _build_rebot_arm102_driver,
    )

    payload = ReBotArm102Payload(serial_number="LDR-001", baudrate=115200)
    robot = _StubRobot(payload)
    factory = _StubFactory(port="/dev/ttyUSB0")
    driver = await _build_rebot_arm102_driver(robot, factory)
    assert driver is not None


@pytest.mark.anyio
async def test_build_rebot_arm102_from_dict_payload() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import (
        _build_rebot_arm102_driver,
    )

    payload: dict[str, object] = {
        "serial_number": "LDR-002",
        "baudrate": 1000000,
        "unlock_on_connect": False,
    }
    robot = _StubRobot(payload)
    factory = _StubFactory(port="/dev/ttyUSB1")
    driver = await _build_rebot_arm102_driver(robot, factory)
    assert driver is not None


@pytest.mark.anyio
async def test_build_rebot_arm102_port_not_found() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import (
        ReBotArm102Payload,
        _build_rebot_arm102_driver,
    )

    payload = ReBotArm102Payload(serial_number="LDR-MISSING")
    robot = _StubRobot(payload)
    factory = _StubFactory(port=None)
    with pytest.raises(RuntimeError, match="Robot not found"):
        await _build_rebot_arm102_driver(robot, factory)


def test_get_rebot_urdf_root_returns_path() -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _get_rebot_urdf_root

    root = _get_rebot_urdf_root()
    assert isinstance(root, Path)
    assert root.exists()
