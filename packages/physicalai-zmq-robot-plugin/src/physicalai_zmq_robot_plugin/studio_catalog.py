from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Protocol, TypeVar

from physicalai.robot.interface import Robot as PhysicalAIRobot
from pydantic import BaseModel, Field

from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobot

_PayloadT_co = TypeVar("_PayloadT_co", covariant=True)


class _PayloadContainer(Protocol[_PayloadT_co]):
    payload: _PayloadT_co


class _PortFinder(Protocol):
    async def find_port_by_serial(self, serial_number: str) -> str | None: ...


class _SerialPortInfo(Protocol):
    connection_string: str
    serial_number: str
    robot_type: str


@dataclass(frozen=True)
class _CatalogEntry:
    type: str
    display_name: str
    role: str
    urdf_path: str | None
    package_map: dict[str, str]
    joint_map: dict[str, list[str]]


_AssetSource = Literal["builtin", "plugin"]
_DiscoverDevicesCallable = Callable[[list[_SerialPortInfo]], Awaitable[list[_SerialPortInfo]]]
_AssetRootResolver = Callable[[], Path]
_BuildRobotCallable = Callable[..., Awaitable[PhysicalAIRobot]]
_PayloadModelType = type[BaseModel]


@dataclass(frozen=True)
class _RobotAdapterOptions:
    include_velocities: bool = False
    goal_time_scale: float = 1.0
    external_effort_gain: float | None = 0.1


@dataclass(frozen=True)
class _CatalogDefinition:
    entry: _CatalogEntry
    urdf_relative_path: Path | None
    package_root: Path | None
    asset_source: _AssetSource
    asset_root_resolver: _AssetRootResolver | None
    discover_devices: _DiscoverDevicesCallable
    robot_builder: _BuildRobotCallable | None = None
    payload_model: _PayloadModelType | None = None
    adapter_options: _RobotAdapterOptions = _RobotAdapterOptions()

    @property
    def robot_type(self) -> str:
        return self.entry.type


if TYPE_CHECKING:

    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: _CatalogDefinition) -> None: ...
        def register_many(self, definitions: list[_CatalogDefinition]) -> None: ...


class ZMQRobotPayload(BaseModel):
    zmq_endpoint: str = Field(..., description="ZMQ endpoint of the remote robot (e.g., tcp://host:port)")
    command_timeout: float = 5.0


async def _discover_devices(devices: list[_SerialPortInfo]) -> list[_SerialPortInfo]:
    await asyncio.sleep(0)
    return devices


async def _build_zmq_robot(
    robot: _PayloadContainer[ZMQRobotPayload],
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
            entry=_CatalogEntry(
                type="ZMQ_Robot",
                display_name="ZMQ Robot",
                role="follower",
                urdf_path=None,
                package_map={},
                joint_map={},
            ),
            urdf_relative_path=None,
            package_root=None,
            asset_source="plugin",
            asset_root_resolver=None,
            discover_devices=_discover_devices,
            robot_builder=_build_zmq_robot,
            payload_model=ZMQRobotPayload,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    registry.register_many(_definitions())
