"""Studio catalog plugin for Physical AI Studio.

Exposes :func:`register_physicalai_studio_plugin` as the entry-point callable
for the ``physicalai.studio.catalog_plugins`` group.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol

from physicalai.robot.interface import Robot as PhysicalAIRobot
from physicalai.robot.so101 import SO101, SO101Calibration
from pydantic import BaseModel, Field

import physicalai_bimanual_so101_plugin
from physicalai_bimanual_so101_plugin import BimanualSO101, get_urdf_path


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
    robot_model: type | None = None
    asset: _RobotAsset | None = None
    adapter_options: _RobotAdapterOptions = field(default_factory=_RobotAdapterOptions)
    probe: Any = None

    @property
    def robot_type(self) -> str:
        return self.type


if TYPE_CHECKING:
    class _RobotCatalogRegistry(Protocol):
        def register(self, definition: _CatalogDefinition) -> None: ...


_BIMANUAL_SO101_TO_URDF: dict[str, list[str]] = {
    "left_shoulder_pan.pos": ["left_shoulder_pan"],
    "left_shoulder_lift.pos": ["left_shoulder_lift"],
    "left_elbow_flex.pos": ["left_elbow_flex"],
    "left_wrist_flex.pos": ["left_wrist_flex"],
    "left_wrist_roll.pos": ["left_wrist_roll"],
    "left_gripper.pos": ["left_gripper"],
    "right_shoulder_pan.pos": ["right_shoulder_pan"],
    "right_shoulder_lift.pos": ["right_shoulder_lift"],
    "right_elbow_flex.pos": ["right_elbow_flex"],
    "right_wrist_flex.pos": ["right_wrist_flex"],
    "right_wrist_roll.pos": ["right_wrist_roll"],
    "right_gripper.pos": ["right_gripper"],
}


def _get_bimanual_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root
    plugin_package_root = Path(physicalai_bimanual_so101_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        return site_packages_urdf_root
    return configured_root


_BIMANUAL_SO101_ASSET = _RobotAsset(
    urdf_relative_path=Path("so101_dual/so101_dual.urdf"),
    packages={"so101_dual": Path("so101_dual")},
    joint_map=_BIMANUAL_SO101_TO_URDF,
    root_resolver=_get_bimanual_urdf_root,
)


class BimanualSO101Payload(BaseModel):
    left_serial_number: str = Field(...)
    right_serial_number: str = Field(...)
    left_calibration_id: str | None = None
    right_calibration_id: str | None = None
    baudrate: int = 1_000_000
    role: Literal["follower", "leader"] = "follower"
    disable_torque_on_disconnect: bool = True


class BimanualSO101Robot(BaseModel):
    type: Literal["BimanualSO101_Follower", "BimanualSO101_Leader"] = Field(...)
    payload: BimanualSO101Payload


class BimanualSO101Probe:
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
        validated = BimanualSO101Payload(**payload)
        if manager is not None:
            ports_list = manager.robots
            left_match = any(
                p.serial_number == validated.left_serial_number for p in ports_list
            )
            right_match = any(
                p.serial_number == validated.right_serial_number for p in ports_list
            )
            return left_match and right_match

        from serial.tools import list_ports

        all_ports = list_ports.comports()
        left_match = any(
            p.serial_number == validated.left_serial_number for p in all_ports
        )
        right_match = any(
            p.serial_number == validated.right_serial_number for p in all_ports
        )
        return left_match and right_match


_BIMANUAL_PROBE = BimanualSO101Probe()


def _calibration_to_so101(calibration: Any) -> SO101Calibration:
    return SO101Calibration.from_dict(
        {
            name: {
                "id": val.id,
                "drive_mode": val.drive_mode,
                "homing_offset": val.homing_offset,
                "range_min": val.range_min,
                "range_max": val.range_max,
            }
            for name, val in calibration.values.items()
        }
    )


async def _build_bimanual_driver(
    robot: Any, factory: _PortFinder
) -> PhysicalAIRobot:
    raw = robot.payload
    if isinstance(raw, BimanualSO101Payload):
        validated = raw
    elif isinstance(raw, dict):
        validated = BimanualSO101Payload.model_validate(raw)
    else:
        validated = BimanualSO101Payload.model_validate(raw.model_dump(mode="json"))

    role = validated.role
    baudrate = validated.baudrate

    left_port = await factory.find_port_by_serial(validated.left_serial_number)
    if left_port is None:
        msg = f"Left arm not found: {validated.left_serial_number}"
        raise RuntimeError(msg)

    right_port = await factory.find_port_by_serial(validated.right_serial_number)
    if right_port is None:
        msg = f"Right arm not found: {validated.right_serial_number}"
        raise RuntimeError(msg)

    has_left_cal = validated.left_calibration_id is not None
    has_right_cal = validated.right_calibration_id is not None

    if has_left_cal != has_right_cal:
        msg = (
            "Both arms must have calibration IDs or both must be uncalibrated. "
            f"Got left_calibration_id={validated.left_calibration_id!r}, "
            f"right_calibration_id={validated.right_calibration_id!r}."
        )
        raise ValueError(msg)

    if has_left_cal and has_right_cal:
        left_cal_data = await factory.get_calibration_by_id(
            validated.left_calibration_id
        )
        if left_cal_data is None:
            msg = f"Calibration not found for left arm: {validated.left_calibration_id}"
            raise RuntimeError(msg)
        right_cal_data = await factory.get_calibration_by_id(
            validated.right_calibration_id
        )
        if right_cal_data is None:
            msg = (
                f"Calibration not found for right arm: {validated.right_calibration_id}"
            )
            raise RuntimeError(msg)

        left_arm = SO101(
            port=left_port,
            baudrate=baudrate,
            role=role,
            calibration=_calibration_to_so101(left_cal_data),
            unit="normalized",
        )
        right_arm = SO101(
            port=right_port,
            baudrate=baudrate,
            role=role,
            calibration=_calibration_to_so101(right_cal_data),
            unit="normalized",
        )
    else:
        left_arm = SO101.uncalibrated(
            port=left_port, baudrate=baudrate, role=role, unit="ticks"
        )
        right_arm = SO101.uncalibrated(
            port=right_port, baudrate=baudrate, role=role, unit="ticks"
        )

    return BimanualSO101(left=left_arm, right=right_arm)


def _definitions() -> list[_CatalogDefinition]:
    return [
        _CatalogDefinition(
            type="BimanualSO101_Follower",
            display_name="Bimanual SO-101 Follower",
            role="follower",
            robot_builder=_build_bimanual_driver,
            robot_model=BimanualSO101Robot,
            asset=_BIMANUAL_SO101_ASSET,
            adapter_options=_RobotAdapterOptions(
                include_velocities=False, external_effort_gain=None
            ),
            probe=_BIMANUAL_PROBE,
        ),
        _CatalogDefinition(
            type="BimanualSO101_Leader",
            display_name="Bimanual SO-101 Leader",
            role="leader",
            robot_builder=_build_bimanual_driver,
            robot_model=BimanualSO101Robot,
            asset=_BIMANUAL_SO101_ASSET,
            adapter_options=_RobotAdapterOptions(
                include_velocities=False, external_effort_gain=None
            ),
            probe=_BIMANUAL_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    for definition in _definitions():
        registry.register(definition)
