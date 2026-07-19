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

import physicalai_lerobot_plugin
from physicalai_lerobot_plugin import get_urdf_path

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: RobotCatalogDefinition) -> None: ...


def _get_lerobot_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root
    plugin_package_root = Path(physicalai_lerobot_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        return site_packages_urdf_root
    return configured_root


_LEROBOT_ASSET = RobotAsset(
    urdf_relative_path=Path("lerobot/urdf/lerobot.urdf"),
    packages={"lerobot": Path("lerobot")},
    joint_map={
        "shoulder_pan.pos": ["joint1"],
        "shoulder_lift.pos": ["joint2"],
        "elbow_flex.pos": ["joint3"],
        "wrist_flex.pos": ["joint4"],
        "wrist_roll.pos": ["joint5"],
        "gripper.pos": ["joint6"],
    },
    root_resolver=_get_lerobot_urdf_root,
)


class LeRobotPayload(BaseModel):
    robot_type: str = Field(...)
    port: str = Field(...)
    joint_order: list[str] = Field(...)
    obs_position_keys: list[str] | None = None
    act_position_keys: list[str] | None = None
    disable_torque_on_disconnect: bool = True
    serial_number: str = ""


class LeRobotProbe:
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
        validated = LeRobotPayload(**payload)
        if manager is not None:
            ports_list = manager.robots
            if validated.serial_number:
                return any(p.serial_number == validated.serial_number for p in ports_list)
            return validated.port in {p.connection_string for p in ports_list}
        from serial.tools import list_ports

        all_ports = list_ports.comports()
        if validated.serial_number:
            return any(p.serial_number == validated.serial_number for p in all_ports)
        return validated.port in {p.device for p in all_ports}


_LEROBOT_PROBE = LeRobotProbe()


def _make_lerobot_config(validated: LeRobotPayload) -> object:
    if validated.robot_type == "so100_follower":
        from lerobot.robots.so_follower.config_so_follower import SO100FollowerConfig

        return SO100FollowerConfig(
            port=validated.port,
            disable_torque_on_disconnect=validated.disable_torque_on_disconnect,
        )
    if validated.robot_type == "so101_follower":
        from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig

        return SO101FollowerConfig(
            port=validated.port,
            disable_torque_on_disconnect=validated.disable_torque_on_disconnect,
        )

    msg = f"Unsupported LeRobot type: {validated.robot_type}"
    raise ValueError(msg)


def _build_lerobot_driver(robot: object, role: str) -> PhysicalAIRobot:
    from lerobot.robots import make_robot_from_config

    from physicalai_lerobot_plugin.lerobot_adapter import LeRobotAdapter

    raw = robot.payload
    if isinstance(raw, LeRobotPayload):
        validated = raw
    elif isinstance(raw, dict):
        validated = LeRobotPayload.model_validate(raw)
    else:
        validated = LeRobotPayload.model_validate(raw.model_dump(mode="json"))

    lerobot_config = _make_lerobot_config(validated)
    lerobot_robot = make_robot_from_config(lerobot_config)

    return LeRobotAdapter(
        robot=lerobot_robot,
        joint_order=validated.joint_order,
        obs_position_keys=validated.obs_position_keys,
        act_position_keys=validated.act_position_keys,
        role=role,
    )


async def _build_lerobot_follower(robot: object, factory: CatalogRobotFactory) -> PhysicalAIRobot:
    return _build_lerobot_driver(robot, "follower")


async def _build_lerobot_leader(robot: object, factory: CatalogRobotFactory) -> PhysicalAIRobot:
    return _build_lerobot_driver(robot, "leader")


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="LeRobot_Follower",
            display_name="LeRobot Follower",
            role="follower",
            robot_builder=_build_lerobot_follower,
            robot_payload=LeRobotPayload,
            asset=_LEROBOT_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEROBOT_PROBE,
        ),
        RobotCatalogDefinition(
            type="LeRobot_Leader",
            display_name="LeRobot Leader",
            role="leader",
            robot_builder=_build_lerobot_leader,
            robot_payload=LeRobotPayload,
            asset=_LEROBOT_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEROBOT_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
