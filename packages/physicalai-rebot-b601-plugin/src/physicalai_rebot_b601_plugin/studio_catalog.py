"""Studio catalog plugin for Physical AI Studio.

Exposes :func:`register_physicalai_studio_plugin` as the entry-point callable
for the ``physicalai.studio.catalog_plugins`` group.
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Protocol, TypeVar

from loguru import logger
from physicalai.robot.interface import Robot as PhysicalAIRobot
from pydantic import BaseModel, Field

import physicalai_rebot_b601_plugin
from physicalai_rebot_b601_plugin import ReBotArm102Leader, ReBotB601DM, get_urdf_path

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


_REBOT_B601_DM_TO_URDF: dict[str, list[str]] = {
    "shoulder_pan.pos": ["joint1"],
    "shoulder_lift.pos": ["joint2"],
    "elbow_flex.pos": ["joint3"],
    "wrist_flex.pos": ["joint4"],
    "wrist_yaw.pos": ["joint5"],
    "wrist_roll.pos": ["joint6"],
    "gripper.pos": [],
}

_REBOT_ARM102_TO_URDF: dict[str, list[str]] = {
    "shoulder_pan.pos": ["joint1"],
    "shoulder_lift.pos": ["joint2"],
    "elbow_flex.pos": ["joint3"],
    "wrist_flex.pos": ["joint4"],
    "wrist_yaw.pos": ["joint5"],
    "wrist_roll.pos": ["joint6"],
    "gripper.pos": ["joint7_left", "joint7_right"],
}


def _get_rebot_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root

    plugin_package_root = Path(physicalai_rebot_b601_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        logger.warning(
            "ReBot plugin get_urdf_path() returned missing path={}; falling back to {}",
            configured_root,
            site_packages_urdf_root,
        )
        return site_packages_urdf_root

    return configured_root


async def _discover_rebot_devices(devices: list[_SerialPortInfo]) -> list[_SerialPortInfo]:
    await asyncio.sleep(0)
    return devices


class ReBotB601DMPayload(BaseModel):
    """Connection payload for a ReBot B601 DM follower arm."""

    connection_string: str = ""
    serial_number: str = Field(...)
    can_adapter: Literal["damiao", "socketcan"] = "damiao"
    dm_serial_baud: int = 921600
    disable_torque_on_disconnect: bool = True
    force_pos_torque_ratio: float = 0.1


class ReBotArm102Payload(BaseModel):
    """Connection payload for a ReBot Arm102 leader arm."""

    connection_string: str = ""
    serial_number: str = Field(...)
    baudrate: int = 1_000_000
    unlock_on_connect: bool = True
    reset_multi_turn_on_connect: bool = True
    zero_on_connect: bool = False


async def _build_rebot_b601_dm_driver(
    robot: _PayloadContainer[ReBotB601DMPayload],
    factory: _PortFinder,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, ReBotB601DMPayload):
        validated = raw
    elif isinstance(raw, dict):
        validated = ReBotB601DMPayload.model_validate(raw)
    else:
        validated = ReBotB601DMPayload.model_validate(raw.model_dump(mode="json"))

    serial_number = validated.serial_number
    port = await factory.find_port_by_serial(serial_number)
    if port is None:
        msg = f"Robot not found: {serial_number}"
        raise RuntimeError(msg)

    return ReBotB601DM(
        port=port,
        can_adapter=validated.can_adapter,
        dm_serial_baud=validated.dm_serial_baud,
        role="follower",
        disable_torque_on_disconnect=validated.disable_torque_on_disconnect,
        force_pos_torque_ratio=validated.force_pos_torque_ratio,
    )


async def _build_rebot_arm102_driver(
    robot: _PayloadContainer[ReBotArm102Payload],
    factory: _PortFinder,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, ReBotArm102Payload):
        validated = raw
    elif isinstance(raw, dict):
        validated = ReBotArm102Payload.model_validate(raw)
    else:
        validated = ReBotArm102Payload.model_validate(raw.model_dump(mode="json"))

    serial_number = validated.serial_number
    port = await factory.find_port_by_serial(serial_number)
    if port is None:
        msg = f"Robot not found: {serial_number}"
        raise RuntimeError(msg)

    return ReBotArm102Leader(
        port=port,
        baudrate=validated.baudrate,
        unlock_on_connect=validated.unlock_on_connect,
        reset_multi_turn_on_connect=validated.reset_multi_turn_on_connect,
        zero_on_connect=validated.zero_on_connect,
    )


def _definitions() -> list[_CatalogDefinition]:
    return [
        _CatalogDefinition(
            entry=_CatalogEntry(
                type="ReBot_B601_DM_Follower",
                display_name="ReBot B601 DM Follower",
                role="follower",
                urdf_path="/api/robots/catalog/ReBot_B601_DM_Follower/urdf",
                package_map={
                    "rebot-b601-dm": "/api/robots/catalog/ReBot_B601_DM_Follower",
                },
                joint_map=_REBOT_B601_DM_TO_URDF,
            ),
            urdf_relative_path=Path("rebot-b601-dm/urdf/reBot-DevArm_fixend.urdf"),
            package_root=Path("rebot-b601-dm"),
            asset_source="plugin",
            asset_root_resolver=_get_rebot_urdf_root,
            discover_devices=_discover_rebot_devices,
            robot_builder=_build_rebot_b601_dm_driver,
            payload_model=ReBotB601DMPayload,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
        ),
        _CatalogDefinition(
            entry=_CatalogEntry(
                type="ReBot_Arm102_Leader",
                display_name="ReBot Arm102 Leader",
                role="leader",
                urdf_path="/api/robots/catalog/ReBot_Arm102_Leader/urdf",
                package_map={
                    "stararm102": "/api/robots/catalog/ReBot_Arm102_Leader",
                },
                joint_map=_REBOT_ARM102_TO_URDF,
            ),
            urdf_relative_path=Path("stararm102/urdf/stararm102_description.urdf"),
            package_root=Path("stararm102"),
            asset_source="plugin",
            asset_root_resolver=_get_rebot_urdf_root,
            discover_devices=_discover_rebot_devices,
            robot_builder=_build_rebot_arm102_driver,
            payload_model=ReBotArm102Payload,
            adapter_options=_RobotAdapterOptions(include_velocities=False, external_effort_gain=None),
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register ReBot robot catalog entries with the Physical AI Studio registry.

    Args:
        registry: The Studio robot catalog registry instance.
    """
    registry.register_many(_definitions())
