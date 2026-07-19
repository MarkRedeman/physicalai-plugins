from __future__ import annotations

from typing import TYPE_CHECKING, Any

from physicalai.robot.interface import Robot as PhysicalAIRobot
from physicalai_studio_plugin import (
    CatalogRobotFactory,
    RobotAdapterOptions,
    RobotCatalogDefinition,
)
from pydantic import BaseModel, Field

from physicalai_websocket_robot_plugin.websocket_robot import WebSocketRobot

if TYPE_CHECKING:
    from typing import Protocol

    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: RobotCatalogDefinition) -> None: ...


class WebSocketRobotPayload(BaseModel):
    websocket_url: str = Field(..., description="WebSocket URL of the remote robot")
    connect_timeout: float = 10.0
    command_timeout: float = 5.0


async def _build_websocket_robot(
    robot: Any,
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, WebSocketRobotPayload):
        validated = raw
    elif isinstance(raw, dict):
        validated = WebSocketRobotPayload.model_validate(raw)
    else:
        validated = WebSocketRobotPayload.model_validate(raw.model_dump(mode="json"))

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
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
