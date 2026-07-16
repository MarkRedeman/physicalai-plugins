"""Studio catalog plugin for Physical AI Studio.

Exposes :func:`register_physicalai_studio_plugin` as the entry-point callable
for the ``physicalai.studio.catalog_plugins`` group.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol

from loguru import logger
from pydantic import BaseModel, Field

import physicalai_rebot_b601_plugin
from physicalai_rebot_b601_plugin import ReBotArm102Leader, ReBotB601DM, get_urdf_path

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from physicalai.robot.interface import Robot as PhysicalAIRobot


class _PortFinder(Protocol):
    async def find_so101_port(self, robot: Any) -> str: ...
    async def find_port_by_serial(self, serial_number: str) -> str | None: ...
    async def get_calibration_by_id(self, calibration_id: Any) -> Any: ...


class _SerialPortInfo(Protocol):
    connection_string: str | None
    serial_number: str | None


class _PortScanner(Protocol):
    async def find_robots(self) -> None: ...
    @property
    def robots(self) -> list[_SerialPortInfo]: ...


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


@dataclass
class _CatalogDefinition:
    type: str
    display_name: str
    role: str
    robot_builder: Callable[..., Awaitable[PhysicalAIRobot]] | None = None
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


_REBOT_B601_DM_ASSET = _RobotAsset(
    urdf_relative_path=Path("rebot-b601-dm/urdf/reBot-DevArm_fixend.urdf"),
    packages={"rebot-b601-dm": Path("rebot-b601-dm")},
    joint_map=_REBOT_B601_DM_TO_URDF,
    root_resolver=_get_rebot_urdf_root,
)

_REBOT_ARM102_ASSET = _RobotAsset(
    urdf_relative_path=Path("stararm102/urdf/stararm102_description.urdf"),
    packages={"stararm102": Path("stararm102")},
    joint_map=_REBOT_ARM102_TO_URDF,
    root_resolver=_get_rebot_urdf_root,
)


class ReBotB601DMPayload(BaseModel):
    connection_string: str = ""
    serial_number: str = Field(...)
    can_adapter: Literal["damiao", "socketcan"] = "damiao"
    dm_serial_baud: int = 921600
    disable_torque_on_disconnect: bool = True
    force_pos_torque_ratio: float = 0.1


class ReBotArm102Payload(BaseModel):
    connection_string: str = ""
    serial_number: str = Field(...)
    baudrate: int = 1_000_000
    unlock_on_connect: bool = True
    reset_multi_turn_on_connect: bool = True
    zero_on_connect: bool = False


class ReBotProbe:
    async def discover(self, manager: _PortScanner) -> list[_SerialPortInfo]:
        await manager.find_robots()
        return manager.robots

    async def identify(self, payload: dict[str, Any], manager: Any = None, joint: str | None = None) -> None:
        pass

    async def is_online(self, payload: dict[str, Any], manager: Any = None) -> bool:
        return True


_REBOT_PROBE = ReBotProbe()


async def _build_rebot_b601_dm_driver(robot: Any, factory: _PortFinder) -> PhysicalAIRobot:
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


async def _build_rebot_arm102_driver(robot: Any, factory: _PortFinder) -> PhysicalAIRobot:
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
            type="ReBot_B601_DM_Follower",
            display_name="ReBot B601 DM Follower",
            role="follower",
            robot_builder=_build_rebot_b601_dm_driver,
            robot_payload=ReBotB601DMPayload,
            asset=_REBOT_B601_DM_ASSET,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_REBOT_PROBE,
        ),
        _CatalogDefinition(
            type="ReBot_Arm102_Leader",
            display_name="ReBot Arm102 Leader",
            role="leader",
            robot_builder=_build_rebot_arm102_driver,
            robot_payload=ReBotArm102Payload,
            asset=_REBOT_ARM102_ASSET,
            adapter_options=_RobotAdapterOptions(include_velocities=False, external_effort_gain=None),
            probe=_REBOT_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
