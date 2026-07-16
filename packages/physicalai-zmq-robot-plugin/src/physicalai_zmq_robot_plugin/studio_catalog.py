from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol

from physicalai.robot.interface import Robot as PhysicalAIRobot
from pydantic import BaseModel, Field

from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobot

if TYPE_CHECKING:
    from pathlib import Path


class _PortFinder(Protocol):
    async def find_port_by_serial(self, serial_number: str) -> str | None: ...


_BuildRobotCallable = Callable[..., Awaitable[PhysicalAIRobot]]


@dataclass(frozen=True)
class _RobotAdapterOptions:
    include_velocities: bool = False
    goal_time_scale: float = 1.0
    external_effort_gain: float | None = 0.1


@dataclass(frozen=True)
class _RobotAsset:
    urdf_relative_path: Path
    packages: dict[str, Path]
    joint_map: dict[str, list[str]]
    root_resolver: Callable[[], Path] | None = None


@dataclass(frozen=True)
class _CatalogDefinition:
    type: str
    display_name: str
    role: str
    robot_builder: _BuildRobotCallable | None = None
    robot_payload: type[BaseModel] | None = None
    asset: _RobotAsset | None = None
    adapter_options: _RobotAdapterOptions = field(default_factory=_RobotAdapterOptions)
    probe: Any = None

    @property
    def robot_type(self) -> str:
        return self.type


if TYPE_CHECKING:

    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: _CatalogDefinition) -> None: ...


class ZMQRobotPayload(BaseModel):
    zmq_endpoint: str = Field(..., description="ZMQ endpoint of the remote robot (e.g., tcp://host:port)")
    command_timeout: float = 5.0


async def _build_zmq_robot(
    robot: Any,
    factory: _PortFinder,
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


def _definitions() -> list[_CatalogDefinition]:
    return [
        _CatalogDefinition(
            type="ZMQ_Robot",
            display_name="ZMQ Robot",
            role="follower",
            robot_builder=_build_zmq_robot,
            robot_payload=ZMQRobotPayload,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
