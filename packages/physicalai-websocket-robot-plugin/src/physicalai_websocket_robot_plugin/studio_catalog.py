"""Studio catalog plugin for WebSocket-backed robots."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING
from urllib.parse import urlparse

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PayloadContainer,
    PortScanner,
    RobotAdapterOptions,
    RobotCatalogDefinition,
    RobotProbe,
    SerialPortInfo,
)
from pydantic import BaseModel, Field

from physicalai_websocket_robot_plugin.websocket_robot import WebSocketRobot

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: RobotCatalogDefinition) -> None: ...


class WebSocketRobotPayload(BaseModel):
    """Connection payload for a WebSocket-backed robot."""

    websocket_url: str = Field(..., description="WebSocket URL of the remote robot")
    connect_timeout: float = 10.0
    command_timeout: float = 5.0


class WebSocketRobotProbe(RobotProbe[WebSocketRobotPayload]):
    """Lightweight probe implementation for WebSocket robots."""

    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        """Discover available serial devices.

        Returns:
            list[SerialPortInfo]: Detected serial devices.
        """
        _ = self
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: WebSocketRobotPayload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Request a visual identify action, if supported."""
        _ = self, payload, manager, joint

    async def is_online(self, payload: WebSocketRobotPayload, manager: PortScanner | None = None) -> bool:
        """Check whether the WebSocket endpoint appears reachable.

        Returns:
            bool: ``True`` when payload URL parses as ``ws`` or ``wss``.
        """
        _ = self, manager
        parsed = urlparse(payload.websocket_url)
        return parsed.scheme in {"ws", "wss"} and bool(parsed.netloc)


_WEBSOCKET_PROBE = WebSocketRobotProbe()


async def _build_websocket_robot(
    robot: PayloadContainer[object],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    _ = factory
    await asyncio.sleep(0)
    raw = robot.payload
    validated = raw if isinstance(raw, WebSocketRobotPayload) else WebSocketRobotPayload.model_validate(raw)

    return WebSocketRobot(
        websocket_url=validated.websocket_url,
        connect_timeout=validated.connect_timeout,
        command_timeout=validated.command_timeout,
    )


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="WebSocket_Robot",
            display_name="WebSocket Robot",
            role="follower",
            robot_builder=_build_websocket_robot,
            robot_payload=WebSocketRobotPayload,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_WEBSOCKET_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register WebSocket robot catalog entries with the Studio registry."""
    for definition in _definitions():
        registry.register(definition)
