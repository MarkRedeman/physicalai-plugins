from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError

from physicalai_websocket_robot_plugin.studio_catalog import (
    WebSocketRobotPayload,
    _discover_devices,
    _definitions,
    register_physicalai_studio_plugin,
)


class TestWebSocketRobotPayload:
    def test_valid_payload(self):
        payload = WebSocketRobotPayload(websocket_url="ws://localhost:8765")
        assert payload.websocket_url == "ws://localhost:8765"
        assert payload.connect_timeout == 10.0

    def test_invalid_payload_missing_url(self):
        with pytest.raises(ValidationError):
            WebSocketRobotPayload()


class TestDefinitions:
    def test_definitions_return_list(self):
        defs = _definitions()
        assert len(defs) == 1

    def test_definition_contents(self):
        defs = _definitions()
        entry = defs[0]
        assert entry.entry.type == "WebSocket_Robot"
        assert entry.entry.urdf_path is None
        assert entry.entry.joint_map == {}
        assert entry.entry.package_map == {}
        assert entry.payload_model is WebSocketRobotPayload
        assert entry.urdf_relative_path is None
        assert entry.asset_root_resolver is None

    def test_discover_devices(self):
        import asyncio

        result = asyncio.run(_discover_devices([]))
        assert result == []


class TestRegistration:
    def test_register_called(self):
        registry = MagicMock()
        register_physicalai_studio_plugin(registry)
        registry.register_many.assert_called_once()
