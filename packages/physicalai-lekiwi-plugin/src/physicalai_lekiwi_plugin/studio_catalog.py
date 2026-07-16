"""Studio catalog plugin for Physical AI Studio.

Exposes :func:`register_physicalai_studio_plugin` as the entry-point callable
for the ``physicalai.studio.catalog_plugins`` group.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Protocol

from pydantic import BaseModel, Field

import physicalai_lekiwi_plugin
from physicalai_lekiwi_plugin import LeKiwi, get_urdf_path

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


_LEKIWI_TO_URDF: dict[str, list[str]] = {
    "arm_shoulder_pan.pos": ["STS3215_03a-v1_Revolute-45"],
    "arm_shoulder_lift.pos": ["STS3215_03a-v1-1_Revolute-49"],
    "arm_elbow_flex.pos": ["STS3215_03a-v1-2_Revolute-51"],
    "arm_wrist_flex.pos": ["STS3215_03a-v1-3_Revolute-53"],
    "arm_wrist_roll.pos": ["STS3215_03a_Wrist_Roll-v1_Revolute-55"],
    "arm_gripper.pos": ["STS3215_03a-v1-4_Revolute-57"],
}


def _get_lekiwi_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root
    plugin_package_root = Path(physicalai_lekiwi_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        return site_packages_urdf_root
    return configured_root


_LEKIWI_ASSET = _RobotAsset(
    urdf_relative_path=Path("lekiwi/urdf/LeKiwi.urdf"),
    packages={"lekiwi": Path("lekiwi")},
    joint_map=_LEKIWI_TO_URDF,
    root_resolver=_get_lekiwi_urdf_root,
)


class LeKiwiPayload(BaseModel):
    connection_string: str = ""
    serial_number: str = Field(...)
    baudrate: int = 1_000_000
    disable_torque_on_disconnect: bool = True


class LeKiwiProbe:
    async def discover(self, manager: _PortScanner) -> list[_SerialPortInfo]:
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: dict[str, Any],
        manager: Any = None,
        joint: str | None = None,
    ) -> None:
        pass

    async def is_online(self, payload: dict[str, Any], manager: Any = None) -> bool:
        validated = LeKiwiPayload(**payload)

        if manager is not None:
            ports_list = manager.robots
            if validated.serial_number:
                return any(p.serial_number == validated.serial_number for p in ports_list)
            return validated.connection_string in {p.connection_string for p in ports_list}

        from serial.tools import list_ports

        all_ports = list_ports.comports()
        if validated.serial_number:
            return any(p.serial_number == validated.serial_number for p in all_ports)
        return validated.connection_string in {p.device for p in all_ports}


_LEKIWI_PROBE = LeKiwiProbe()


async def _build_lekiwi_driver(robot: Any, factory: _PortFinder) -> PhysicalAIRobot:
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


async def _build_lekiwi_leader(robot: Any, factory: _PortFinder) -> PhysicalAIRobot:
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

    return LeKiwi.uncalibrated(
        port=port,
        baudrate=validated.baudrate,
        role="leader",
    )


def _definitions() -> list[_CatalogDefinition]:
    return [
        _CatalogDefinition(
            type="LeKiwi_Follower",
            display_name="LeKiwi Follower",
            role="follower",
            robot_builder=_build_lekiwi_driver,
            robot_payload=LeKiwiPayload,
            asset=_LEKIWI_ASSET,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEKIWI_PROBE,
        ),
        _CatalogDefinition(
            type="LeKiwi_Leader",
            display_name="LeKiwi Leader",
            role="leader",
            robot_builder=_build_lekiwi_leader,
            robot_payload=LeKiwiPayload,
            asset=_LEKIWI_ASSET,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEKIWI_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
