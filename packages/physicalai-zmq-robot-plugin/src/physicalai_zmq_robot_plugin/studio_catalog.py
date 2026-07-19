from __future__ import annotations

from typing import TYPE_CHECKING, Any

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    RobotAdapterOptions,
    RobotCatalogDefinition,
)
from pydantic import BaseModel, Field

from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobot

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: RobotCatalogDefinition) -> None: ...


class ZMQRobotPayload(BaseModel):
    zmq_endpoint: str = Field(..., description="ZMQ endpoint of the remote robot (e.g., tcp://host:port)")
    command_timeout: float = 5.0


async def _build_zmq_robot(
    robot: Any,
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, ZMQRobotPayload):
        validated = raw
    elif isinstance(raw, dict):
        validated = ZMQRobotPayload.model_validate(raw)
    else:
        validated = ZMQRobotPayload.model_validate(raw.model_dump(mode="json"))

    return ZMQRobot(
        zmq_endpoint=validated.zmq_endpoint,
        command_timeout=validated.command_timeout,
    )


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="ZMQ_Robot",
            display_name="ZMQ Robot",
            role="follower",
            robot_builder=_build_zmq_robot,
            robot_payload=ZMQRobotPayload,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
