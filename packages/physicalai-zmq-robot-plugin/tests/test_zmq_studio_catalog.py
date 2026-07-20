from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError

from physicalai_zmq_robot_plugin.studio_catalog import (
    ZMQRobotPayload,
    ZMQRobotProbe,
    _definitions,
    register_physicalai_studio_plugin,
)


class TestZMQRobotPayload:
    def test_valid_payload(self) -> None:
        payload = ZMQRobotPayload(zmq_endpoint="tcp://localhost:5555")
        assert payload.zmq_endpoint == "tcp://localhost:5555"
        assert payload.command_timeout == 5.0

    def test_invalid_payload_missing_endpoint(self) -> None:
        with pytest.raises(ValidationError):
            ZMQRobotPayload()


class TestDefinitions:
    def test_definitions_return_list(self) -> None:
        defs = _definitions()
        assert len(defs) == 1

    def test_definition_contents(self) -> None:
        defs = _definitions()
        definition = defs[0]
        assert definition.type == "ZMQ_Robot"
        assert definition.asset is None
        assert definition.robot_payload is ZMQRobotPayload
        assert definition.probe is not None


class TestProbe:
    @pytest.mark.anyio
    async def test_is_online_for_valid_endpoint(self) -> None:
        probe = ZMQRobotProbe()
        payload = ZMQRobotPayload(zmq_endpoint="tcp://localhost:5555")
        assert await probe.is_online(payload) is True

    @pytest.mark.anyio
    async def test_is_online_for_invalid_endpoint(self) -> None:
        probe = ZMQRobotProbe()
        payload = ZMQRobotPayload(zmq_endpoint="http://localhost:5555")
        assert await probe.is_online(payload) is False


class TestRegistration:
    def test_register_called(self) -> None:
        registry = MagicMock()
        register_physicalai_studio_plugin(registry)
        registry.register.assert_called_once()
