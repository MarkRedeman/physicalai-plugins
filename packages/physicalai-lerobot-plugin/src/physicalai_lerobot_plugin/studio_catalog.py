"""Studio catalog plugin for Physical AI Studio.

Exposes :func:`register_physicalai_studio_plugin` as the entry-point callable
for the ``physicalai.studio.catalog_plugins`` group.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol

from pydantic import BaseModel, Field

import physicalai_lerobot_plugin
from physicalai_lerobot_plugin import get_urdf_path

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from physicalai.robot.interface import Robot as PhysicalAIRobot


class _PortFinder(Protocol):
    """Factory provided by Studio to resolve serial ports."""

    async def find_so101_port(self, robot: object) -> str: ...
    async def find_port_by_serial(self, serial_number: str) -> str | None: ...
    async def get_calibration_by_id(self, calibration_id: object) -> object: ...


class _SerialPortInfo(Protocol):
    """Serial port info protocol."""

    connection_string: str | None
    serial_number: str | None


class _PortScanner(Protocol):
    """Port scanner protocol."""

    async def find_robots(self) -> None: ...

    @property
    def robots(self) -> list[_SerialPortInfo]: ...


@dataclass(frozen=True)
class _RobotAdapterOptions:
    """Controls how Studio wraps the driver."""

    include_velocities: bool = False
    goal_time_scale: float = 1.0
    external_effort_gain: float | None = 0.1


@dataclass(frozen=True)
class _RobotAsset:
    """Filesystem configuration for a robot's visual model."""

    urdf_relative_path: Path
    packages: dict[str, Path]
    joint_map: dict[str, list[str]]
    root_resolver: Callable[[], Path] | None = None


@dataclass
class _CatalogDefinition:
    """Flat catalog definition schema."""

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
        """Registry protocol for catalog definitions."""

        def register(self, definition: _CatalogDefinition) -> None: ...


def _get_lerobot_urdf_root() -> Path:
    """Resolve the URDF root directory for the lerobot plugin.

    Returns:
        The absolute path to the URDF root directory.
    """
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root
    plugin_package_root = Path(physicalai_lerobot_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        return site_packages_urdf_root
    return configured_root


_LEROBOT_ASSET = _RobotAsset(
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
    """Payload model for LeRobot robot configuration.

    Attributes:
        robot_type: LeRobot robot type name (e.g. ``"so100_follower"``).
        port: Serial port path (e.g. ``"/dev/ttyACM0"``).
        joint_order: Joint names in PhysicalAI order.
        obs_position_keys: Keys in the lerobot observation dict.
        act_position_keys: Keys in the lerobot action dict.
        disable_torque_on_disconnect: Whether to disable torque on disconnect.
        serial_number: Serial number for port discovery.
    """

    robot_type: str = Field(...)
    port: str = Field(...)
    joint_order: list[str] = Field(...)
    obs_position_keys: list[str] | None = None
    act_position_keys: list[str] | None = None
    disable_torque_on_disconnect: bool = True
    serial_number: str = ""


class LeRobotRobotModel(BaseModel):
    """Pydantic model for Studio's dynamic discriminated union."""

    type: Literal["LeRobot_Follower", "LeRobot_Leader"] = Field(...)
    payload: LeRobotPayload


class LeRobotProbe:
    """Probe for discovering LeRobot devices."""

    async def discover(self, manager: _PortScanner) -> list[_SerialPortInfo]:
        """Discover available LeRobot devices.

        Args:
            manager: Port scanner provided by Studio.

        Returns:
            List of discovered serial port info.
        """
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: dict[str, Any],
        manager: object = None,
        joint: str | None = None,
    ) -> None:
        """Identify a device (no-op for LeRobot)."""

    async def is_online(
        self, payload: dict[str, Any], manager: object = None
    ) -> bool:
        """Check if the device is currently reachable.

        Args:
            payload: Device configuration.
            manager: Optional port scanner.

        Returns:
            True if the device appears online.
        """
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
    """Create a LeRobot RobotConfig from the validated payload.

    Args:
        validated: The validated payload.

    Returns:
        A LeRobot RobotConfig instance for the requested robot type.

    Raises:
        ValueError: If the robot type is not supported.
    """
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
    """Construct a LeRobotAdapter wrapping the appropriate LeRobot robot.

    Args:
        robot: The stored robot config (Pydantic model, dict, or unknown).
        role: ``"follower"`` or ``"leader"``.

    Returns:
        A PhysicalAI-compatible LeRobotAdapter instance.
    """
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


async def _build_lerobot_follower(robot: object, factory: _PortFinder) -> PhysicalAIRobot:
    """Build a follower LeRobot driver.

    Args:
        robot: The stored robot config.
        factory: Port finder provided by Studio.

    Returns:
        A PhysicalAI-compatible robot instance.
    """
    return _build_lerobot_driver(robot, "follower")


async def _build_lerobot_leader(robot: object, factory: _PortFinder) -> PhysicalAIRobot:
    """Build a leader LeRobot driver.

    Args:
        robot: The stored robot config.
        factory: Port finder provided by Studio.

    Returns:
        A PhysicalAI-compatible robot instance.
    """
    return _build_lerobot_driver(robot, "leader")


def _definitions() -> list[_CatalogDefinition]:
    """Build the list of catalog definitions for this plugin.

    Returns:
        A list of ``_CatalogDefinition`` entries.
    """
    return [
        _CatalogDefinition(
            type="LeRobot_Follower",
            display_name="LeRobot Follower",
            role="follower",
            robot_builder=_build_lerobot_follower,
            robot_model=LeRobotRobotModel,
            asset=_LEROBOT_ASSET,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEROBOT_PROBE,
        ),
        _CatalogDefinition(
            type="LeRobot_Leader",
            display_name="LeRobot Leader",
            role="leader",
            robot_builder=_build_lerobot_leader,
            robot_model=LeRobotRobotModel,
            asset=_LEROBOT_ASSET,
            adapter_options=_RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_LEROBOT_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register all catalog definitions with the given registry.

    Args:
        registry: The Studio catalog registry.
    """
    for definition in _definitions():
        registry.register(definition)
