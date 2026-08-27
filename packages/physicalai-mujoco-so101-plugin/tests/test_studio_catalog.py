from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from physicalai.config import to_config

from physicalai_mujoco_so101_plugin.constants import (
    BIMANUAL_SO101_JOINT_ORDER,
    DEFAULT_BIMANUAL_MUJOCO_OWNER_NAME,
    DEFAULT_MUJOCO_OWNER_NAME,
    SO101_JOINT_ORDER,
)
from physicalai_mujoco_so101_plugin.studio_catalog import (
    MuJoCoSO101BimanualPayload,
    MuJoCoSO101Payload,
    MuJoCoSO101Probe,
    _definitions,
    _SharedSO101Robot,
    register_physicalai_studio_plugin,
)


class TestMuJoCoSO101Payload:
    def test_default_payload(self) -> None:
        payload = MuJoCoSO101Payload()
        assert payload.name == DEFAULT_MUJOCO_OWNER_NAME
        assert payload.allow_remote is False
        assert payload.connect_timeout == 10.0

    def test_custom_payload(self) -> None:
        payload = MuJoCoSO101Payload(name="my-sim", allow_remote=True, connect_timeout=5.0)
        assert payload.name == "my-sim"
        assert payload.allow_remote is True
        assert payload.connect_timeout == 5.0

    def test_payload_model_rebuild(self) -> None:
        MuJoCoSO101Payload.model_rebuild(raise_errors=True)

    def test_name_str_validated(self) -> None:
        payload = MuJoCoSO101Payload(name="test-robot-1")
        assert payload.name == "test-robot-1"


class TestMuJoCoSO101BimanualPayload:
    def test_default_owner_name_differs_from_single_arm(self) -> None:
        payload = MuJoCoSO101BimanualPayload()
        assert payload.name == DEFAULT_BIMANUAL_MUJOCO_OWNER_NAME
        assert payload.name != MuJoCoSO101Payload().name

    def test_inherits_connection_settings(self) -> None:
        payload = MuJoCoSO101BimanualPayload(allow_remote=True, connect_timeout=3.0)
        assert payload.allow_remote is True
        assert payload.connect_timeout == 3.0

    def test_payload_model_rebuild(self) -> None:
        MuJoCoSO101BimanualPayload.model_rebuild(raise_errors=True)


class TestDefinitions:
    def test_definitions_return_list(self) -> None:
        defs = _definitions()
        assert len(defs) == 2

    def test_definition_contents(self) -> None:
        defs = _definitions()
        definition = defs[0]
        assert definition.type == "MuJoCo_SO101_Follower"
        assert definition.display_name == "MuJoCo SO-101 Follower"
        assert definition.role == "follower"
        assert definition.asset is not None
        assert definition.robot_payload is MuJoCoSO101Payload
        assert definition.probe is not None

    def test_bimanual_definition_contents(self) -> None:
        defs = _definitions()
        definition = defs[1]
        assert definition.type == "MuJoCo_SO101_Bimanual_Follower"
        assert definition.display_name == "MuJoCo SO-101 Bimanual Follower"
        assert definition.role == "follower"
        assert definition.asset is not None
        assert definition.robot_payload is MuJoCoSO101BimanualPayload
        assert definition.probe is not None

    def test_adapter_options(self) -> None:
        defs = _definitions()
        definition = defs[0]
        assert definition.adapter_options.include_velocities is False
        assert definition.adapter_options.external_effort_gain is None
        assert definition.adapter_options.goal_time_scale == 1.0

    def test_builder_callable(self) -> None:
        defs = _definitions()
        definition = defs[0]
        assert callable(definition.robot_builder)

    def test_payload_model_class(self) -> None:
        defs = _definitions()
        definition = defs[0]
        assert definition.robot_payload is MuJoCoSO101Payload

    def test_urdf_asset(self) -> None:
        defs = _definitions()
        definition = defs[0]
        assert definition.asset is not None
        assert str(definition.asset.urdf_relative_path) == "so101/so101_new_calib.urdf"
        assert "so101" in definition.asset.packages
        assert "shoulder_pan.pos" in definition.asset.joint_map
        assert definition.asset.root_resolver is not None
        assert (definition.asset.root_resolver() / definition.asset.urdf_relative_path).is_file()

    def test_bimanual_urdf_asset(self) -> None:
        defs = _definitions()
        definition = defs[1]
        assert definition.asset is not None
        assert str(definition.asset.urdf_relative_path) == "so101/so101_dual.urdf"
        assert "so101" in definition.asset.packages
        assert "left_shoulder_pan.pos" in definition.asset.joint_map
        assert "right_shoulder_pan.pos" in definition.asset.joint_map
        assert definition.asset.root_resolver is not None
        assert (definition.asset.root_resolver() / definition.asset.urdf_relative_path).is_file()


class TestSharedRobotAdapter:
    def test_has_no_owned_devices(self) -> None:
        robot = _SharedSO101Robot(
            DEFAULT_MUJOCO_OWNER_NAME,
            False,
            10.0,
            SO101_JOINT_ORDER,
        )
        assert robot.device_ids == ()
        assert robot.joint_names == list(SO101_JOINT_ORDER)

    def test_bimanual_joint_names(self) -> None:
        robot = _SharedSO101Robot(
            DEFAULT_MUJOCO_OWNER_NAME,
            False,
            10.0,
            BIMANUAL_SO101_JOINT_ORDER,
        )
        assert robot.joint_names == list(BIMANUAL_SO101_JOINT_ORDER)

    def test_exports_owner_name_recipe(self) -> None:
        robot = _SharedSO101Robot(
            DEFAULT_MUJOCO_OWNER_NAME,
            False,
            5.0,
            SO101_JOINT_ORDER,
        )

        assert to_config(robot) == {
            "class_path": "physicalai_mujoco_so101_plugin.studio_catalog._SharedSO101Robot",
            "init_args": {
                "owner_name": DEFAULT_MUJOCO_OWNER_NAME,
                "allow_remote": False,
                "connect_timeout": 5.0,
                "joint_names": list(SO101_JOINT_ORDER),
            },
        }


class TestSharedRobotLifecycle:
    @staticmethod
    def _robot() -> _SharedSO101Robot:
        return _SharedSO101Robot(
            owner_name=DEFAULT_MUJOCO_OWNER_NAME,
            allow_remote=False,
            connect_timeout=10.0,
            joint_names=SO101_JOINT_ORDER,
        )

    def test_disconnect_releases_the_handle(self) -> None:
        robot = self._robot()
        shared = MagicMock()
        with patch("physicalai.robot.transport.SharedRobot.attach", return_value=shared):
            robot.connect()
        robot.disconnect()

        shared.disconnect.assert_called_once()
        assert robot.is_connected() is False

    def test_disconnect_releases_the_handle_when_teardown_fails(self) -> None:
        robot = self._robot()
        shared = MagicMock()
        shared.disconnect.side_effect = RuntimeError("zenoh gone")
        with patch("physicalai.robot.transport.SharedRobot.attach", return_value=shared):
            robot.connect()

        with pytest.raises(RuntimeError, match="zenoh gone"):
            robot.disconnect()
        assert robot.is_connected() is False

    def test_reconnect_attaches_a_new_owner(self) -> None:
        robot = self._robot()
        first, second = MagicMock(), MagicMock()
        with patch("physicalai.robot.transport.SharedRobot.attach", side_effect=[first, second]) as attach:
            robot.connect()
            robot.disconnect()
            robot.connect()

        assert attach.call_count == 2
        assert robot._shared_robot is second  # noqa: SLF001

    def test_connect_is_idempotent(self) -> None:
        robot = self._robot()
        with patch("physicalai.robot.transport.SharedRobot.attach", return_value=MagicMock()) as attach:
            robot.connect()
            robot.connect()

        attach.assert_called_once()


class TestProbe:
    @pytest.mark.anyio
    async def test_discover(self) -> None:
        probe = MuJoCoSO101Probe()
        manager = AsyncMock()
        manager.robots = []
        result = await probe.discover(manager)
        assert result == []
        manager.find_robots.assert_awaited_once()

    @pytest.mark.anyio
    async def test_identify(self) -> None:
        probe = MuJoCoSO101Probe()
        payload = MuJoCoSO101Payload()
        await probe.identify(payload)

    @pytest.mark.anyio
    async def test_is_online_no_owner(self) -> None:
        from physicalai_mujoco_so101_plugin import studio_catalog as sc

        probe = MuJoCoSO101Probe()
        payload = MuJoCoSO101Payload(name="nonexistent")

        with patch.object(sc, "_check_zenoh_robot_online", return_value=False):
            result = await probe.is_online(payload)
            assert result is False


class TestRegistration:
    def test_register_called(self) -> None:
        registry = MagicMock()
        register_physicalai_studio_plugin(registry)
        assert registry.register_robot.call_count == 2
