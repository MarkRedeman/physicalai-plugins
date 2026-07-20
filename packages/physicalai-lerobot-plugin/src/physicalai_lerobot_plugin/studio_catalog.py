# ruff: noqa: PLC0415, RUF029

"""Studio catalog plugin for LeRobot adapters."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, cast

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PayloadContainer,
    PortScanner,
    RobotAdapterOptions,
    RobotAsset,
    RobotCatalogDefinition,
    RobotProbe,
    SerialPortInfo,
)
from pydantic import BaseModel, Field
from serial.tools import list_ports

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
    """Connection payload for LeRobot-adapted robots."""

    robot_type: str = Field(...)
    port: str = Field(...)
    joint_order: list[str] = Field(...)
    obs_position_keys: list[str] | None = None
    act_position_keys: list[str] | None = None
    disable_torque_on_disconnect: bool = True
    serial_number: str = ""


class LeRobotProbe(RobotProbe[LeRobotPayload]):
    """Probe implementation for LeRobot-adapted robots."""

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
        payload: LeRobotPayload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Request a visual identify action, if supported."""
        _ = self, payload, manager, joint

    async def is_online(self, payload: LeRobotPayload, manager: PortScanner | None = None) -> bool:
        """Check whether the configured LeRobot endpoint is online.

        Returns:
            bool: ``True`` when the configured serial or port is present.
        """
        _ = self
        if manager is not None:
            ports_list = manager.robots
            if payload.serial_number:
                return any(p.serial_number == payload.serial_number for p in ports_list)
            return payload.port in {p.connection_string for p in ports_list}

        all_ports = list_ports.comports()
        if payload.serial_number:
            return any(p.serial_number == payload.serial_number for p in all_ports)
        return payload.port in {p.device for p in all_ports}


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


def _build_lerobot_driver(robot: PayloadContainer[object], role: Literal["follower", "leader"]) -> PhysicalAIRobot:
    from lerobot.robots import make_robot_from_config

    from physicalai_lerobot_plugin.lerobot_adapter import LeRobotAdapter

    raw = robot.payload
    validated = raw if isinstance(raw, LeRobotPayload) else LeRobotPayload.model_validate(raw)

    lerobot_config = _make_lerobot_config(validated)
    lerobot_robot = make_robot_from_config(cast("Any", lerobot_config))

    return LeRobotAdapter(
        robot=lerobot_robot,
        joint_order=validated.joint_order,
        obs_position_keys=validated.obs_position_keys,
        act_position_keys=validated.act_position_keys,
        role=role,
    )


async def _build_lerobot_follower(robot: PayloadContainer[object], factory: CatalogRobotFactory) -> PhysicalAIRobot:
    _ = factory
    return _build_lerobot_driver(robot, "follower")


async def _build_lerobot_leader(robot: PayloadContainer[object], factory: CatalogRobotFactory) -> PhysicalAIRobot:
    _ = factory
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
    """Register LeRobot catalog entries with the Physical AI Studio registry."""
    for definition in _definitions():
        registry.register(definition)
