"""PhysicalAI Studio catalog entry for generic ROS 2 robots."""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PayloadContainer,
    PortScanner,
    RobotAdapterOptions,
    RobotCatalogDefinition,
    RobotProbe,
    SerialPortInfo,
)
from pydantic import BaseModel, Field

from physicalai_ros2_plugin.robot import ROS2Robot

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register_robot(self, definition: RobotCatalogDefinition) -> None: ...


class ROS2RobotPayload(BaseModel):
    """Topic contract for a generic ROS 2 follower robot."""

    joint_names: list[str] = Field(..., min_length=1)
    state_topic: str = "/joint_states"
    command_topic: str = "/joint_trajectory_controller/joint_trajectory"
    node_name: str = "physicalai_ros2_robot"
    namespace: str = ""
    command_timeout: float = Field(default=1.0, gt=0)
    connect_timeout: float = Field(default=10.0, gt=0)
    goal_time: float = Field(default=0.1, gt=0)
    angle_unit: Literal["radians", "degrees"] = "radians"
    sensor_topics: dict[str, str] = Field(default_factory=dict)
    camera_topics: dict[str, str] = Field(default_factory=dict)


class ROS2Probe(RobotProbe[ROS2RobotPayload]):
    """ROS graph discovery is intentionally delegated to the configured topics."""

    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        _ = self, manager
        return []

    async def identify(
        self,
        payload: ROS2RobotPayload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        _ = self, payload, manager, joint

    async def is_online(self, payload: ROS2RobotPayload, manager: PortScanner | None = None) -> bool:
        _ = self, payload, manager
        return False


async def _build_ros2_robot(
    robot: PayloadContainer[ROS2RobotPayload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    _ = factory
    raw = robot.payload
    payload = raw if isinstance(raw, ROS2RobotPayload) else ROS2RobotPayload.model_validate(raw)
    return ROS2Robot(**payload.model_dump())


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="ROS2_Generic_Follower",
            display_name="Generic ROS 2 Follower",
            category="ROS 2",
            source="first_party",
            role="follower",
            robot_builder=_build_ros2_robot,
            robot_payload=ROS2RobotPayload,
            asset=None,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=ROS2Probe(),
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register the generic ROS 2 follower catalog definition."""
    ROS2RobotPayload.model_rebuild(raise_errors=True)
    for definition in _definitions():
        registry.register_robot(definition)
