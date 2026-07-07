from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Protocol, TypeVar

from loguru import logger
from physicalai.robot.interface import Robot as PhysicalAIRobot
from pydantic import BaseModel, Field

import physicalai_lekiwi_plugin
from physicalai_lekiwi_plugin import LeKiwi, get_urdf_path

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


_LEKIWI_TO_URDF: dict[str, list[str]] = {
    "arm_shoulder_pan.pos": ["shoulder_pan"],
    "arm_shoulder_lift.pos": ["shoulder_lift"],
    "arm_elbow_flex.pos": ["elbow_flex"],
    "arm_wrist_flex.pos": ["wrist_flex"],
    "arm_wrist_roll.pos": ["wrist_roll"],
    "arm_gripper.pos": ["gripper"],
}


def _get_lekiwi_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root

    plugin_package_root = Path(physicalai_lekiwi_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        logger.warning(
            "LeKiwi plugin get_urdf_path() returned missing path={}; falling back to {}",
            configured_root,
            site_packages_urdf_root,
        )
        return site_packages_urdf_root

    return configured_root


async def _discover_lekiwi_devices(devices: list[_SerialPortInfo]) -> list[_SerialPortInfo]:
    await asyncio.sleep(0)
    return devices


class LeKiwiPayload(BaseModel):
    connection_string: str = ""
    serial_number: str = Field(...)
    baudrate: int = 1_000_000
    disable_torque_on_disconnect: bool = True


async def _build_lekiwi_driver(
    robot: _PayloadContainer[LeKiwiPayload],
    factory: _PortFinder,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, LeKiwiPayload):
        validated = raw
    elif isinstance(raw, dict):
        validated = LeKiwiPayload.model_validate(raw)
    else:
        validated = LeKiwiPayload.model_validate(raw.model_dump(mode="json"))

    serial_number = validated.serial_number
    port = await factory.find_port_by_serial(serial_number)
    if port is None:
        msg = f"Robot not found: {serial_number}"
        raise RuntimeError(msg)

    return LeKiwi(
        port=port,
        baudrate=validated.baudrate,
        role="follower",
        unit="normalized",
    )


def _definitions() -> list[_CatalogDefinition]:
    return [
        _CatalogDefinition(
            entry=_CatalogEntry(
                type="LeKiwi_Follower",
                display_name="LeKiwi Follower",
                role="follower",
                urdf_path="/api/robots/catalog/LeKiwi_Follower/urdf",
                package_map={
                    "lekiwi": "/api/robots/catalog/LeKiwi_Follower",
                },
                joint_map=_LEKIWI_TO_URDF,
            ),
            urdf_relative_path=Path("lekiwi/urdf/LeKiwi.urdf"),
            package_root=Path("lekiwi"),
            asset_source="plugin",
            asset_root_resolver=_get_lekiwi_urdf_root,
            discover_devices=_discover_lekiwi_devices,
            robot_builder=_build_lekiwi_driver,
            payload_model=LeKiwiPayload,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    registry.register_many(_definitions())
