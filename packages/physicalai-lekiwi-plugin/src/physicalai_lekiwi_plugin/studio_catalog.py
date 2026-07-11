"""Studio catalog plugin for Physical AI Studio.

Exposes :func:`register_physicalai_studio_plugin` as the entry-point callable
for the ``physicalai.studio.catalog_plugins`` group.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal
from uuid import UUID

from physicalai.robot.interface import Robot as PhysicalAIRobot
from pydantic import BaseModel, Field

import physicalai_lekiwi_plugin
from physicalai_lekiwi_plugin import LeKiwi, get_urdf_path

from schemas import SerialPortInfo
from schemas.robot_type import BaseRobot

from .types import RobotAdapterOptions, RobotCatalogDefinition

if TYPE_CHECKING:
    from .registry import RobotCatalogRegistry
    from .types import CatalogRobot, CatalogRobotFactory, PortScanner

LeKiwiTypes = Literal["LeKiwi_Follower", "LeKiwi_Leader"]


class LeKiwiPayload(BaseModel):
    """Connection payload for a LeKiwi robot."""

    connection_string: str = ""
    serial_number: str = Field(...)
    baudrate: int = 1_000_000
    disable_torque_on_disconnect: bool = True


class LeKiwiRobot(BaseRobot):
    """LeKiwi follower or leader robot using a serial connection."""

    type: LeKiwiTypes = Field(..., description="Type of robot configuration")
    payload: LeKiwiPayload = Field(..., description="LeKiwi connection configuration")


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


async def _build_lekiwi_driver(
    robot: CatalogRobot[LeKiwiPayload],
    factory: CatalogRobotFactory,
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


async def _build_lekiwi_leader(
    robot: CatalogRobot[LeKiwiPayload],
    factory: CatalogRobotFactory,
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

    return LeKiwi.uncalibrated(
        port=port,
        baudrate=validated.baudrate,
        role="leader",
    )


class LeKiwiProbe:
    """Probe for LeKiwi robots — serial port discovery + online check."""

    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: dict[str, Any],
        manager: PortScanner | None,
        joint: str | None = None,
    ) -> None:
        pass

    async def is_online(self, payload: dict[str, Any], manager: PortScanner | None = None) -> bool:
        robot_payload = LeKiwiPayload(**payload)

        if manager is not None:
            ports_list = manager.robots
            if robot_payload.serial_number:
                return any(p.serial_number == robot_payload.serial_number for p in ports_list)
            return robot_payload.connection_string in {p.connection_string for p in ports_list}

        from serial.tools import list_ports

        all_ports = list_ports.comports()
        if robot_payload.serial_number:
            return any(p.serial_number == robot_payload.serial_number for p in all_ports)
        return robot_payload.connection_string in {p.device for p in all_ports}


_LEKIWI_PROBE = LeKiwiProbe()


def get_definitions() -> list[RobotCatalogDefinition]:
    """Return LeKiwi robot catalog definitions."""
    return [
        RobotCatalogDefinition(
            type="LeKiwi_Follower",
            display_name="LeKiwi Follower",
            role="follower",
            urdf_path="/api/robots/catalog/LeKiwi_Follower/urdf",
            package_map={"lekiwi": "/api/robots/catalog/LeKiwi_Follower"},
            joint_map=_LEKIWI_TO_URDF,
            urdf_relative_path="lekiwi/urdf/LeKiwi.urdf",
            robot_builder=_build_lekiwi_driver,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEKIWI_PROBE,
            robot_model=LeKiwiRobot,
        ),
        RobotCatalogDefinition(
            type="LeKiwi_Leader",
            display_name="LeKiwi Leader",
            role="leader",
            urdf_path="/api/robots/catalog/LeKiwi_Leader/urdf",
            package_map={"lekiwi": "/api/robots/catalog/LeKiwi_Leader"},
            joint_map=_LEKIWI_TO_URDF,
            urdf_relative_path="lekiwi/urdf/LeKiwi.urdf",
            robot_builder=_build_lekiwi_leader,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEKIWI_PROBE,
            robot_model=LeKiwiRobot,
        ),
    ]


def register_physicalai_studio_plugin(registry: RobotCatalogRegistry) -> None:
    """Register LeKiwi robot catalog entries with the Physical AI Studio registry."""
    for definition in get_definitions():
        registry.register(definition)
