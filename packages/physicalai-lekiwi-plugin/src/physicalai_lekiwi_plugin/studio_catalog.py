from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PortScanner,
    RobotAdapterOptions,
    RobotAsset,
    RobotCatalogDefinition,
    SerialPortInfo,
)
from pydantic import BaseModel, Field

import physicalai_lekiwi_plugin
from physicalai_lekiwi_plugin import LeKiwi, get_urdf_path

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: RobotCatalogDefinition) -> None: ...


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


_LEKIWI_ASSET = RobotAsset(
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
    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: dict[str, Any],
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        pass

    async def is_online(self, payload: dict[str, Any], manager: PortScanner | None = None) -> bool:
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


async def _build_lekiwi_driver(robot: Any, factory: CatalogRobotFactory) -> PhysicalAIRobot:
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


async def _build_lekiwi_leader(robot: Any, factory: CatalogRobotFactory) -> PhysicalAIRobot:
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


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="LeKiwi_Follower",
            display_name="LeKiwi Follower",
            role="follower",
            robot_builder=_build_lekiwi_driver,
            robot_payload=LeKiwiPayload,
            asset=_LEKIWI_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEKIWI_PROBE,
        ),
        RobotCatalogDefinition(
            type="LeKiwi_Leader",
            display_name="LeKiwi Leader",
            role="leader",
            robot_builder=_build_lekiwi_leader,
            robot_payload=LeKiwiPayload,
            asset=_LEKIWI_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEKIWI_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
