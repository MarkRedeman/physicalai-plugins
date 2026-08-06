"""PhysicalAI Studio catalog registration for the MuJoCo SO-101 robot."""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import TYPE_CHECKING

from loguru import logger
from physicalai.config import export_config
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

import physicalai_mujoco_so101_plugin
from physicalai_mujoco_so101_plugin._urdf import get_urdf_path
from physicalai_mujoco_so101_plugin.constants import BIMANUAL_SO101_JOINT_ORDER, SO101_JOINT_ORDER

if TYPE_CHECKING:
    from typing import Protocol

    import numpy as np
    from physicalai.robot.interface import Robot as PhysicalAIRobot
    from physicalai.robot.interface import RobotObservation

    class _RobotCatalogRegistry(Protocol):
        def register_robot(self, definition: RobotCatalogDefinition) -> None: ...


_MUJOCO_SO101_TO_URDF: dict[str, list[str]] = {
    "shoulder_pan.pos": ["shoulder_pan"],
    "shoulder_lift.pos": ["shoulder_lift"],
    "elbow_flex.pos": ["elbow_flex"],
    "wrist_flex.pos": ["wrist_flex"],
    "wrist_roll.pos": ["wrist_roll"],
    "gripper.pos": ["gripper"],
}

_MUJOCO_SO101_BIMANUAL_TO_URDF: dict[str, list[str]] = {
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


def _get_mujoco_urdf_root() -> Path:
    configured_root = get_urdf_path()
    if configured_root.exists():
        return configured_root
    plugin_package_root = Path(physicalai_mujoco_so101_plugin.__file__).resolve().parent
    site_packages_urdf_root = plugin_package_root.parent / "urdf"
    if site_packages_urdf_root.exists():
        logger.warning(
            "get_urdf_path() returned missing path={}; falling back to {}",
            configured_root,
            site_packages_urdf_root,
        )
        return site_packages_urdf_root
    return configured_root


_MUJOCO_SO101_ASSET = RobotAsset(
    urdf_relative_path=Path("so101/so101_new_calib.urdf"),
    packages={"so101": Path("so101")},
    joint_map=_MUJOCO_SO101_TO_URDF,
    root_resolver=_get_mujoco_urdf_root,
)

_MUJOCO_SO101_BIMANUAL_ASSET = RobotAsset(
    urdf_relative_path=Path("so101/so101_dual.urdf"),
    packages={"so101": Path("so101")},
    joint_map=_MUJOCO_SO101_BIMANUAL_TO_URDF,
    root_resolver=_get_mujoco_urdf_root,
)


class MuJoCoSO101Payload(BaseModel):
    """Connection settings for a MuJoCo SO-101 simulation owner."""

    name: str = Field(
        default="mujoco-so101",
        description="Zenoh logical robot name of the running MuJoCo simulation",
    )
    allow_remote: bool = Field(
        default=False,
        description="Allow connecting to a zenoh owner beyond localhost",
    )
    connect_timeout: float = Field(
        default=10.0,
        description="Timeout in seconds for connecting to the zenoh owner",
    )


class MuJoCoSO101Probe(RobotProbe[MuJoCoSO101Payload]):
    """Discover and query MuJoCo SO-101 simulation owners."""

    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        """Return robots found by the port scanner."""
        _ = self
        await manager.find_robots()
        return manager.robots

    async def identify(
        self,
        payload: MuJoCoSO101Payload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Perform no visual identification for the simulated robot."""
        _ = self, payload, manager, joint

    async def is_online(
        self,
        payload: MuJoCoSO101Payload,
        manager: PortScanner | None = None,
    ) -> bool:
        """Return whether the configured simulation owner is reachable."""
        _ = self, manager
        return await asyncio.to_thread(_check_zenoh_robot_online, payload.name)


def _check_zenoh_robot_online(name: str) -> bool:
    from physicalai.robot.transport import SharedRobot  # noqa: PLC0415

    try:
        robot = SharedRobot.attach(name=name, connect_timeout=2.0)
        robot.connect()
        robot.disconnect()
    except (ConnectionError, TimeoutError, RuntimeError):
        return False
    else:
        return True


_MUJOCO_PROBE = MuJoCoSO101Probe()


@export_config(class_path="physicalai_mujoco_so101_plugin.studio_catalog._SharedSO101Robot")
class _SharedSO101Robot:
    def __init__(self, shared_robot: object, joint_names: list[str] | tuple[str, ...]) -> None:
        self._shared_robot = shared_robot
        self.joint_names = list(joint_names)

    @property
    def device_ids(self) -> tuple[str, ...]:
        """The attached MuJoCo owner, not this wrapper, owns the simulation."""
        return ()

    def connect(self) -> None:
        self._shared_robot.connect()

    def disconnect(self) -> None:
        self._shared_robot.disconnect()

    def get_observation(self) -> RobotObservation:
        return self._shared_robot.get_observation()

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        self._shared_robot.send_action(action, goal_time=goal_time)

    def is_connected(self) -> bool:
        return self._shared_robot.is_connected()


async def _build_mujoco_robot(
    robot: PayloadContainer[MuJoCoSO101Payload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    _ = factory
    await asyncio.sleep(0)
    raw = robot.payload
    validated = raw if isinstance(raw, MuJoCoSO101Payload) else MuJoCoSO101Payload.model_validate(raw)

    from physicalai.robot.transport import SharedRobot  # noqa: PLC0415

    shared = SharedRobot.attach(
        name=validated.name,
        allow_remote=validated.allow_remote,
        connect_timeout=validated.connect_timeout,
    )
    return _SharedSO101Robot(shared, SO101_JOINT_ORDER)


async def _build_bimanual_mujoco_robot(
    robot: PayloadContainer[MuJoCoSO101Payload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    _ = factory
    await asyncio.sleep(0)
    raw = robot.payload
    validated = raw if isinstance(raw, MuJoCoSO101Payload) else MuJoCoSO101Payload.model_validate(raw)

    from physicalai.robot.transport import SharedRobot  # noqa: PLC0415

    shared = SharedRobot.attach(
        name=validated.name,
        allow_remote=validated.allow_remote,
        connect_timeout=validated.connect_timeout,
    )
    return _SharedSO101Robot(shared, BIMANUAL_SO101_JOINT_ORDER)


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="MuJoCo_SO101_Follower",
            display_name="MuJoCo SO-101 Follower",
            category="MuJoCo",
            source="first_party",
            role="follower",
            robot_builder=_build_mujoco_robot,
            robot_payload=MuJoCoSO101Payload,
            asset=_MUJOCO_SO101_ASSET,
            adapter_options=RobotAdapterOptions(
                include_velocities=False,
                external_effort_gain=None,
            ),
            probe=_MUJOCO_PROBE,
        ),
        RobotCatalogDefinition(
            type="MuJoCo_SO101_Bimanual_Follower",
            display_name="MuJoCo SO-101 Bimanual Follower",
            category="MuJoCo",
            source="first_party",
            role="follower",
            robot_builder=_build_bimanual_mujoco_robot,
            robot_payload=MuJoCoSO101Payload,
            asset=_MUJOCO_SO101_BIMANUAL_ASSET,
            adapter_options=RobotAdapterOptions(
                include_velocities=False,
                external_effort_gain=None,
            ),
            probe=_MUJOCO_PROBE,
        ),
    ]


def _assert_payload_model_resolvable(model: type[BaseModel]) -> None:
    model.model_rebuild(_types_namespace=globals(), raise_errors=True)


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register the MuJoCo SO-101 catalog definition."""
    for definition in _definitions():
        payload_model = definition.robot_payload
        if isinstance(payload_model, type) and issubclass(payload_model, BaseModel):
            _assert_payload_model_resolvable(payload_model)
        registry.register_robot(definition)
