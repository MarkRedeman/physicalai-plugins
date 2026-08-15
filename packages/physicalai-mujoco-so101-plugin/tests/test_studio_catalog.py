from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from physicalai.config import to_config
from physicalai.robot.transport import SharedRobot

from physicalai_mujoco_so101_plugin.studio_catalog import (
    MuJoCoSO101Payload,
    MuJoCoSO101Probe,
    _definitions,
    _SharedSO101Robot,
    register_physicalai_studio_plugin,
)


class TestMuJoCoSO101Payload:
    def test_default_payload(self) -> None:
        payload = MuJoCoSO101Payload()
        assert payload.name == "mujoco-so101"
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


class TestDefinitions:
    def test_definitions_return_list(self) -> None:
        defs = _definitions()
        assert len(defs) == 1

    def test_definition_contents(self) -> None:
        defs = _definitions()
        definition = defs[0]
        assert definition.type == "MuJoCo_SO101_Follower"
        assert definition.display_name == "MuJoCo SO-101 Follower"
        assert definition.category == "MuJoCo"
        assert definition.source == "first_party"
        assert definition.role == "follower"
        assert definition.asset is not None
        assert definition.robot_payload is MuJoCoSO101Payload
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


class TestSharedRobotAdapter:
    def test_has_no_owned_devices(self) -> None:
        robot = _SharedSO101Robot(SharedRobot.attach("mujoco-so101"))
        assert robot.device_ids == ()

    def test_exports_attach_only_shared_robot_recipe(self) -> None:
        robot = _SharedSO101Robot(SharedRobot.attach("mujoco-so101", connect_timeout=5.0))

        assert to_config(robot) == {
            "class_path": "physicalai_mujoco_so101_plugin.studio_catalog._SharedSO101Robot",
            "init_args": {
                "shared_robot": {
                    "class_path": "physicalai.robot.SharedRobot",
                    "init_args": {
                        "name": "mujoco-so101",
                        "allow_remote": False,
                        "connect_timeout": 5.0,
                    },
                },
            },
        }


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
        registry.register_robot.assert_called_once()
