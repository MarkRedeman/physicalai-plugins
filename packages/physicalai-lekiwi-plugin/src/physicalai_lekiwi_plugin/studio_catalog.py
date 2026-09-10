"""Studio catalog plugin for LeKiwi robots."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PayloadContainer,
    PortScanner,
    RobotAdapterOptions,
    RobotAsset,
    RobotCatalogDefinition,
    RobotProbe,
    SerialPortInfo,
    robot_field_ui,
    robot_payload_ui,
)
from pydantic import BaseModel, ConfigDict, Field
from serial.tools import list_ports

import physicalai_lekiwi_plugin
from physicalai_lekiwi_plugin import LeKiwi, LeKiwiLeader, get_urdf_path
from physicalai_lekiwi_plugin.calibration import LeKiwiCalibration

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register_robot(self, definition: RobotCatalogDefinition) -> None: ...


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

_LEKIWI_LEADER_TO_URDF: dict[str, list[str]] = {
    "arm_shoulder_pan.pos": ["shoulder_pan"],
    "arm_shoulder_lift.pos": ["shoulder_lift"],
    "arm_elbow_flex.pos": ["elbow_flex"],
    "arm_wrist_flex.pos": ["wrist_flex"],
    "arm_wrist_roll.pos": ["wrist_roll"],
    "arm_gripper.pos": ["gripper"],
}

_LEKIWI_LEADER_ASSET = RobotAsset(
    urdf_relative_path=Path("so101-leader/urdf/so101_leader.urdf"),
    packages={"so101-leader": Path("so101-leader")},
    joint_map=_LEKIWI_LEADER_TO_URDF,
    root_resolver=_get_lekiwi_urdf_root,
)


class LeKiwiFollowerPayload(BaseModel):
    """Connection payload for a LeKiwi follower robot."""

    connection_string: str = ""
    serial_number: str = ""
    calibration: dict[str, LeKiwiJointCalibrationPayload] | None = Field(
        default=None,
        json_schema_extra=robot_field_ui(
            {
                "advanced_configuration": True,
                "info": {
                    "description": "Upload a LeKiwi calibration JSON exported from the robot.",
                },
            },
        ),
    )
    baudrate: int = Field(
        default=1_000_000,
        json_schema_extra=robot_field_ui({"advanced_configuration": True}),
    )
    disable_torque_on_disconnect: bool = Field(
        default=True,
        json_schema_extra=robot_field_ui({"advanced_configuration": True}),
    )
    model_config = ConfigDict(  # pyrefly: ignore [bad-argument-type]
        json_schema_extra=robot_payload_ui(
            [
                {
                    "kind": "connection",
                    "label": "Select robot",
                    "device_discovery": True,
                    "bind": {"connection": "connection_string", "serial_number": "serial_number"},
                },
                {
                    "kind": "calibration",
                    "name": "calibration",
                    "info": {
                        "description": (
                            "Upload calibration JSON in the SO101-style joint-name keyed format. "
                            "This field also accepts manual JSON editing."
                        ),
                    },
                },
            ],
        ),
    )


class LeKiwiLeaderPayload(BaseModel):
    """Connection payload for a LeKiwi SO101-based leader robot."""

    connection_string: str = ""
    serial_number: str = ""
    calibration: dict[str, LeKiwiJointCalibrationPayload] | None = Field(
        default=None,
        json_schema_extra=robot_field_ui(
            {
                "advanced_configuration": True,
                "info": {
                    "description": (
                        "Optional calibration for the SO101 leader arm joints. "
                        "When omitted, the leader arm uses raw ticks."
                    ),
                },
            },
        ),
    )
    baudrate: int = Field(
        default=1_000_000,
        json_schema_extra=robot_field_ui({"advanced_configuration": True}),
    )
    disable_torque_on_disconnect: bool = Field(
        default=True,
        json_schema_extra=robot_field_ui({"advanced_configuration": True}),
    )
    use_gamepad: bool = Field(
        default=True,
        json_schema_extra=robot_field_ui(
            {
                "advanced_configuration": True,
                "info": {
                    "description": "Enable Xbox-style gamepad control for base commands (pygame).",
                },
            },
        ),
    )

    model_config = ConfigDict(  # pyrefly: ignore [bad-argument-type]
        json_schema_extra=robot_payload_ui(
            [
                {
                    "kind": "info",
                    "title": "Leader input model",
                    "text": (
                        "LeKiwi leader uses an SO101 arm for arm joints and either keyboard "
                        "or Xbox-style gamepad input for base commands."
                    ),
                    "variant": "info",
                },
                {
                    "kind": "connection",
                    "label": "Select robot",
                    "device_discovery": True,
                    "bind": {"connection": "connection_string", "serial_number": "serial_number"},
                },
                {
                    "kind": "calibration",
                    "name": "calibration",
                    "info": {
                        "description": (
                            "Upload calibration JSON in the SO101-style joint-name keyed format. "
                            "This field also accepts manual JSON editing."
                        ),
                    },
                },
            ],
        ),
    )


class LeKiwiJointCalibrationPayload(BaseModel):
    """Typed calibration payload for one LeKiwi joint."""

    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


type LeKiwiCatalogPayload = LeKiwiFollowerPayload | LeKiwiLeaderPayload


class LeKiwiProbe(RobotProbe[LeKiwiCatalogPayload]):
    """Probe implementation for LeKiwi devices."""

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
        payload: LeKiwiCatalogPayload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Request a visual identify action, if supported."""
        _ = self, payload, manager, joint

    async def is_online(self, payload: LeKiwiCatalogPayload, manager: PortScanner | None = None) -> bool:
        """Check whether the configured LeKiwi is online.

        Returns:
            bool: ``True`` when a matching serial port is present.
        """
        _ = self

        if manager is not None:
            ports_list = manager.robots
            if payload.serial_number:
                return any(p.serial_number == payload.serial_number for p in ports_list)
            return payload.connection_string in {p.connection_string for p in ports_list}

        all_ports = list_ports.comports()
        if payload.serial_number:
            return any(p.serial_number == payload.serial_number for p in all_ports)
        return payload.connection_string in {p.device for p in all_ports}


_LEKIWI_PROBE = LeKiwiProbe()


def _payload_calibration_to_lekiwi(
    calibration: dict[str, LeKiwiJointCalibrationPayload],
) -> LeKiwiCalibration:
    return LeKiwiCalibration.from_dict({name: value.model_dump() for name, value in calibration.items()})


async def _resolve_lekiwi_port(
    validated: LeKiwiCatalogPayload,
    factory: CatalogRobotFactory,
) -> str:
    connection_string = validated.connection_string or None
    serial_number = validated.serial_number or None
    if connection_string is None and serial_number is None:
        msg = "At least one of connection_string or serial_number must be provided"
        raise RuntimeError(msg)
    port = await factory.find_port(
        SerialPortInfo(connection_string=connection_string, serial_number=serial_number),
    )
    if port is None:
        msg = f"Robot not found: {serial_number or connection_string}"
        raise RuntimeError(msg)
    return port


async def _build_lekiwi_driver(
    robot: PayloadContainer[LeKiwiFollowerPayload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, BaseModel) and type(raw) is not LeKiwiFollowerPayload:
        raw = raw.model_dump()
    validated = raw if isinstance(raw, LeKiwiFollowerPayload) else LeKiwiFollowerPayload.model_validate(raw)

    port = await _resolve_lekiwi_port(validated, factory)

    calibration: LeKiwiCalibration | None = None
    if validated.calibration is not None:
        calibration = _payload_calibration_to_lekiwi(validated.calibration)

    if calibration is not None:
        driver = LeKiwi(
            port=port,
            baudrate=validated.baudrate,
            role="follower",
            calibration=calibration,
            unit="normalized",
            disable_torque_on_disconnect=validated.disable_torque_on_disconnect,
        )
    else:
        driver = LeKiwi.uncalibrated(
            port=port,
            baudrate=validated.baudrate,
            role="follower",
            unit="ticks",
            disable_torque_on_disconnect=validated.disable_torque_on_disconnect,
        )
    return driver


async def _build_lekiwi_leader(
    robot: PayloadContainer[LeKiwiLeaderPayload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, BaseModel) and type(raw) is not LeKiwiLeaderPayload:
        raw = raw.model_dump()
    validated = raw if isinstance(raw, LeKiwiLeaderPayload) else LeKiwiLeaderPayload.model_validate(raw)

    port = await _resolve_lekiwi_port(validated, factory)

    calibration_dict: dict[str, dict[str, int]] | None = None
    if validated.calibration is not None:
        calibration_dict = {name: calibration.model_dump() for name, calibration in validated.calibration.items()}

    return LeKiwiLeader(
        port=port,
        baudrate=validated.baudrate,
        calibration=calibration_dict,
        use_gamepad=validated.use_gamepad,
        disable_torque_on_disconnect=validated.disable_torque_on_disconnect,
    )


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="LeKiwi_Follower",
            display_name="LeKiwi Follower",
            role="follower",
            robot_builder=_build_lekiwi_driver,
            robot_payload=LeKiwiFollowerPayload,
            asset=_LEKIWI_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEKIWI_PROBE,
        ),
        RobotCatalogDefinition(
            type="LeKiwi_Leader",
            display_name="LeKiwi Leader",
            role="leader",
            robot_builder=_build_lekiwi_leader,
            robot_payload=LeKiwiLeaderPayload,
            asset=_LEKIWI_LEADER_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEKIWI_PROBE,
        ),
    ]


def _assert_payload_model_resolvable(model: type[BaseModel]) -> None:
    model.model_rebuild(_types_namespace=globals(), raise_errors=True)


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register LeKiwi catalog entries with the Physical AI Studio registry."""
    for definition in _definitions():
        payload_model = definition.robot_payload
        if isinstance(payload_model, type) and issubclass(payload_model, BaseModel):
            _assert_payload_model_resolvable(payload_model)
        registry.register_robot(definition)
