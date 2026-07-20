"""Studio catalog plugin for ZMQ-backed robots."""

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

from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobot

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: RobotCatalogDefinition) -> None: ...


class ZMQRobotPayload(BaseModel):
    """Connection payload for a ZMQ-backed robot."""

    zmq_endpoint: str = Field(..., description="ZMQ endpoint of the remote robot (e.g., tcp://host:port)")
    command_timeout: float = 5.0


class ZMQRobotProbe(RobotProbe[ZMQRobotPayload]):
    """Lightweight probe implementation for ZMQ robots."""

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
        payload: ZMQRobotPayload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Request a visual identify action, if supported."""
        _ = self, payload, manager, joint

    async def is_online(self, payload: ZMQRobotPayload, manager: PortScanner | None = None) -> bool:
        """Check whether the ZMQ endpoint appears valid.

        Returns:
            bool: ``True`` when endpoint uses ``tcp`` with a host.
        """
        _ = self, manager
        parsed = urlparse(payload.zmq_endpoint)
        return parsed.scheme == "tcp" and bool(parsed.netloc)


_ZMQ_PROBE = ZMQRobotProbe()


async def _build_zmq_robot(
    robot: PayloadContainer[object],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    _ = factory
    await asyncio.sleep(0)
    raw = robot.payload
    validated = raw if isinstance(raw, ZMQRobotPayload) else ZMQRobotPayload.model_validate(raw)

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
            probe=_ZMQ_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register ZMQ robot catalog entries with the Studio registry."""
    for definition in _definitions():
        registry.register(definition)
