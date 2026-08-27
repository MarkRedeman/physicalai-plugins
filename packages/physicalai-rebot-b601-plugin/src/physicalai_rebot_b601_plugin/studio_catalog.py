"""Studio catalog plugin for Physical AI Studio.

Exposes :func:`register_physicalai_studio_plugin` as the entry-point callable
for the ``physicalai.studio.catalog_plugins`` group.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Literal, Self

from loguru import logger
from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PayloadContainer,
    PortScanner,
    RobotAdapterOptions,
    RobotAsset,
    RobotCatalogDefinition,
    RobotProbe,
    SerialPortInfo,
    robot_payload_ui,
)
from pydantic import BaseModel, ConfigDict, Field, model_validator

import physicalai_rebot_b601_plugin
from physicalai_rebot_b601_plugin import ReBotArm102Leader, ReBotB601DM, get_urdf_path

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register_robot(self, definition: RobotCatalogDefinition) -> None: ...


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


_REBOT_B601_DM_ASSET = RobotAsset(
    urdf_relative_path=Path("rebot-b601-dm/urdf/reBot-DevArm_fixend.urdf"),
    packages={"rebot-b601-dm": Path("rebot-b601-dm")},
    joint_map=_REBOT_B601_DM_TO_URDF,
    root_resolver=_get_rebot_urdf_root,
)

_REBOT_ARM102_ASSET = RobotAsset(
    urdf_relative_path=Path("stararm102/urdf/stararm102_description.urdf"),
    packages={"stararm102": Path("stararm102")},
    joint_map=_REBOT_ARM102_TO_URDF,
    root_resolver=_get_rebot_urdf_root,
)


class ReBotB601DMPayload(BaseModel):
    """Connection payload for a ReBot B601 DM follower arm."""

    connection_string: str = ""
    serial_number: str = ""
    can_adapter: Literal["damiao", "socketcan"] = "damiao"
    dm_serial_baud: int = 921600
    disable_torque_on_disconnect: bool = True
    force_pos_torque_ratio: float = 0.1
    # Scales REBOT_B601_DM_POS_VEL_DEG_S down only: 1.0 is the arm's rated
    # ceiling, so a configuration must not be able to ask for more.
    max_velocity: float = Field(default=1.0, gt=0.0, le=1.0)

    model_config = ConfigDict(
        json_schema_extra=robot_payload_ui(  # pyrefly: ignore[bad-argument-type]
            [
                {
                    "kind": "connection",
                    "label": "Select robot",
                    "device_discovery": True,
                    "bind": {"connection": "connection_string", "serial_number": "serial_number"},
                },
            ],
        ),
    )

    @model_validator(mode="after")
    def _require_connection_identifier(self) -> Self:
        if not self.connection_string and not self.serial_number:
            msg = "At least one of connection_string or serial_number must be provided"
            raise ValueError(msg)
        return self


class ReBotArm102Payload(BaseModel):
    """Connection payload for a ReBot Arm102 leader arm."""

    connection_string: str = ""
    serial_number: str = ""
    baudrate: int = 1_000_000
    unlock_on_connect: bool = True
    reset_multi_turn_on_connect: bool = True
    zero_on_connect: bool = False

    model_config = ConfigDict(
        json_schema_extra=robot_payload_ui(  # pyrefly: ignore[bad-argument-type]
            [
                {
                    "kind": "connection",
                    "label": "Select robot",
                    "device_discovery": True,
                    "bind": {"connection": "connection_string", "serial_number": "serial_number"},
                },
            ],
        ),
    )

    @model_validator(mode="after")
    def _require_connection_identifier(self) -> Self:
        if not self.connection_string and not self.serial_number:
            msg = "At least one of connection_string or serial_number must be provided"
            raise ValueError(msg)
        return self


type ReBotPayload = ReBotB601DMPayload | ReBotArm102Payload


class ReBotProbe(RobotProbe[ReBotPayload]):
    """Probe implementation for ReBot serial devices."""

    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        """Discover available ReBot devices via the active port scanner.

        Returns:
            list[SerialPortInfo]: Detected serial devices.
        """
        _ = self
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: ReBotPayload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Request a visual identify action on a specific joint, if supported."""
        _ = self, payload, manager, joint

    async def is_online(self, payload: ReBotPayload, manager: PortScanner | None = None) -> bool:
        """Report whether a ReBot device is currently online.

        Returns:
            bool: ``True`` if the device is present in discovered serial ports.
        """
        _ = self
        if manager is None:
            return False

        ports = manager.robots
        if payload.connection_string and any(port.connection_string == payload.connection_string for port in ports):
            return True

        return bool(payload.serial_number) and any(port.serial_number == payload.serial_number for port in ports)


_REBOT_PROBE = ReBotProbe()


async def _build_rebot_b601_dm_driver(
    robot: PayloadContainer[ReBotB601DMPayload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, BaseModel) and type(raw) is not ReBotB601DMPayload:
        raw = raw.model_dump()
    validated = raw if isinstance(raw, ReBotB601DMPayload) else ReBotB601DMPayload.model_validate(raw)
    connection_string = validated.connection_string or None
    serial_number = validated.serial_number or None
    port = await factory.find_port(
        SerialPortInfo(
            connection_string=connection_string,
            serial_number=serial_number,
        ),
    )

    if port is None:
        msg = f"Robot not found: {serial_number or connection_string}"
        raise RuntimeError(msg)
    return ReBotB601DM(
        port=port,
        can_adapter=validated.can_adapter,
        dm_serial_baud=validated.dm_serial_baud,
        role="follower",
        disable_torque_on_disconnect=validated.disable_torque_on_disconnect,
        force_pos_torque_ratio=validated.force_pos_torque_ratio,
        max_velocity=validated.max_velocity,
    )


async def _build_rebot_arm102_driver(
    robot: PayloadContainer[ReBotArm102Payload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, BaseModel) and type(raw) is not ReBotArm102Payload:
        raw = raw.model_dump()
    validated = raw if isinstance(raw, ReBotArm102Payload) else ReBotArm102Payload.model_validate(raw)
    connection_string = validated.connection_string or None
    serial_number = validated.serial_number or None
    serial = SerialPortInfo(
        connection_string=connection_string,
        serial_number=serial_number,
    )
    port = await factory.find_port(serial)

    if port is None:
        msg = f"Robot not found: {serial_number or connection_string}"
        raise RuntimeError(msg)
    return ReBotArm102Leader(
        port=port,
        baudrate=validated.baudrate,
        unlock_on_connect=validated.unlock_on_connect,
        reset_multi_turn_on_connect=validated.reset_multi_turn_on_connect,
        zero_on_connect=validated.zero_on_connect,
    )


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="ReBot_B601_DM_Follower",
            display_name="ReBot B601 DM Follower",
            role="follower",
            robot_builder=_build_rebot_b601_dm_driver,
            robot_payload=ReBotB601DMPayload,
            asset=_REBOT_B601_DM_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_REBOT_PROBE,
        ),
        RobotCatalogDefinition(
            type="ReBot_Arm102_Leader",
            display_name="ReBot Arm102 Leader",
            role="leader",
            robot_builder=_build_rebot_arm102_driver,
            robot_payload=ReBotArm102Payload,
            asset=_REBOT_ARM102_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=False, external_effort_gain=None),
            probe=_REBOT_PROBE,
        ),
    ]


def _assert_payload_model_resolvable(model: type[BaseModel]) -> None:
    model.model_rebuild(_types_namespace=globals(), raise_errors=True)


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register ReBot catalog entries with the Physical AI Studio registry."""
    for definition in _definitions():
        payload_model = definition.robot_payload
        if isinstance(payload_model, type) and issubclass(payload_model, BaseModel):
            _assert_payload_model_resolvable(payload_model)
        registry.register_robot(definition)
