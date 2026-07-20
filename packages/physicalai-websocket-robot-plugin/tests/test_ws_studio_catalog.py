from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError

from physicalai_websocket_robot_plugin.studio_catalog import (
    WebSocketRobotPayload,
    WebSocketRobotProbe,
    _definitions,
    register_physicalai_studio_plugin,
)


class TestWebSocketRobotPayload:
    def test_valid_payload(self) -> None:
        payload = WebSocketRobotPayload(websocket_url="ws://localhost:8765")
        assert payload.websocket_url == "ws://localhost:8765"
        assert payload.connect_timeout == 10.0

    def test_invalid_payload_missing_url(self) -> None:
        with pytest.raises(ValidationError):
            WebSocketRobotPayload()


class TestDefinitions:
    def test_definitions_return_list(self) -> None:
        defs = _definitions()
        assert len(defs) == 1

    def test_definition_contents(self) -> None:
        defs = _definitions()
        definition = defs[0]
        assert definition.type == "WebSocket_Robot"
        assert definition.asset is None
        assert definition.robot_payload is WebSocketRobotPayload
        assert definition.probe is not None


class TestProbe:
    @pytest.mark.anyio
    async def test_is_online_for_valid_websocket_url(self) -> None:
        probe = WebSocketRobotProbe()
        payload = WebSocketRobotPayload(websocket_url="ws://localhost:8765")
        assert await probe.is_online(payload) is True

    @pytest.mark.anyio
    async def test_is_online_for_invalid_url(self) -> None:
        probe = WebSocketRobotProbe()
        payload = WebSocketRobotPayload(websocket_url="http://localhost:8765")
        assert await probe.is_online(payload) is False


class TestRegistration:
    def test_register_called(self) -> None:
        registry = MagicMock()
        register_physicalai_studio_plugin(registry)
        registry.register.assert_called_once()
