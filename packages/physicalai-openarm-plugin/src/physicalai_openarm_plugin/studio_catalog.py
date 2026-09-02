# ruff: noqa: DOC201, RUF029

"""Physical AI Studio catalog definitions for direct OpenArm control."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Literal, Self

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PayloadContainer,
    RobotAdapterOptions,
    RobotAsset,
    RobotCatalogDefinition,
    RobotProbe,
    robot_field_ui,
)
from pydantic import BaseModel, Field, model_validator

from physicalai_openarm_plugin import get_urdf_path
from physicalai_openarm_plugin.bimanual import BimanualOpenArmFollower, BimanualOpenArmLeader
from physicalai_openarm_plugin.openarm import OpenArmFollower, OpenArmLeader

if TYPE_CHECKING:
    from typing import Protocol

    from physicalai.robot.interface import Robot as PhysicalAIRobot

    class _RobotCatalogRegistry(Protocol):
        def register_robot(self, definition: RobotCatalogDefinition) -> None: ...


_SINGLE_JOINT_MAP = {
    **{f"joint_{index}.pos": [f"openarm_joint{index}"] for index in range(1, 8)},
    "gripper.pos": ["openarm_finger_joint1", "openarm_finger_joint2"],
}
_BIMANUAL_JOINT_MAP = {
    f"{side}_joint_{index}.pos": [f"openarm_{side}_joint{index}"] for side in ("left", "right") for index in range(1, 8)
}
_BIMANUAL_JOINT_MAP.update({
    f"{side}_gripper.pos": [f"openarm_{side}_finger_joint1", f"openarm_{side}_finger_joint2"]
    for side in ("left", "right")
})

_SINGLE_ASSET = RobotAsset(
    urdf_relative_path=Path("openarm/openarm_parallel_gripper.urdf"),
    packages={"openarm": Path("openarm")},
    joint_map=_SINGLE_JOINT_MAP,
    root_resolver=get_urdf_path,
)
_BIMANUAL_ASSET = RobotAsset(
    urdf_relative_path=Path("openarm/openarm_bimanual_parallel_gripper.urdf"),
    packages={"openarm": Path("openarm")},
    joint_map=_BIMANUAL_JOINT_MAP,
    root_resolver=get_urdf_path,
)


class OpenArmPayload(BaseModel):
    """Direct OpenArm SocketCAN or experimental Damiao USB-CAN settings."""

    port: str
    side: Literal["left", "right"] | None = None
    can_adapter: Literal["socketcan", "damiao"] = Field(
        default="socketcan",
        json_schema_extra=robot_field_ui({"advanced_configuration": True}),
    )
    dm_serial_baud: int = Field(
        default=921_600,
        json_schema_extra=robot_field_ui({"advanced_configuration": True}),
    )
    use_can_fd: bool = Field(default=True, json_schema_extra=robot_field_ui({"advanced_configuration": True}))
    can_bitrate: int = Field(default=1_000_000, json_schema_extra=robot_field_ui({"advanced_configuration": True}))
    can_data_bitrate: int = Field(default=5_000_000, json_schema_extra=robot_field_ui({"advanced_configuration": True}))
    disable_torque_on_disconnect: bool = Field(
        default=True,
        json_schema_extra=robot_field_ui({"advanced_configuration": True}),
    )
    max_relative_target: float | None = Field(
        default=None,
        json_schema_extra=robot_field_ui({"advanced_configuration": True}),
    )

    @model_validator(mode="after")
    def _validate_follower_configuration(self) -> Self:
        if not self.port:
            msg = "port must be a non-empty SocketCAN interface name"
            raise ValueError(msg)
        return self


class BimanualOpenArmPayload(BaseModel):
    """Direct bimanual OpenArm SocketCAN or experimental Damiao USB-CAN settings."""

    left_port: str
    right_port: str
    can_adapter: Literal["socketcan", "damiao"] = "socketcan"
    dm_serial_baud: int = 921_600
    use_can_fd: bool = True
    can_bitrate: int = 1_000_000
    can_data_bitrate: int = 5_000_000
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | None = None

    @model_validator(mode="after")
    def _validate_ports(self) -> Self:
        if not self.left_port or not self.right_port:
            msg = "left_port and right_port must be non-empty SocketCAN interface names"
            raise ValueError(msg)
        if self.left_port == self.right_port:
            msg = "left_port and right_port must be distinct"
            raise ValueError(msg)
        return self


class OpenArmProbe(RobotProbe[OpenArmPayload | BimanualOpenArmPayload]):
    """SocketCAN cannot be safely auto-discovered through Studio's serial scanner."""

    async def discover(self, manager: object) -> list[object]:
        """Return no serial devices for direct SocketCAN hardware."""
        _ = self, manager
        return []

    async def identify(
        self,
        payload: OpenArmPayload | BimanualOpenArmPayload,
        manager: object | None = None,
        joint: str | None = None,
    ) -> None:
        """Do not send motion commands as a catalog identify action."""
        _ = self, payload, manager, joint

    async def is_online(self, payload: OpenArmPayload | BimanualOpenArmPayload, manager: object | None = None) -> bool:
        """Require explicit connection validation rather than probing live motors."""
        _ = self, payload, manager
        return False


_PROBE = OpenArmProbe()


async def _build_single(
    robot: PayloadContainer[OpenArmPayload],
    factory: CatalogRobotFactory,
    *,
    role: Literal["follower", "leader"],
) -> PhysicalAIRobot:
    _ = factory
    payload = OpenArmPayload.model_validate(robot.payload)
    shared = {
        "can_adapter": payload.can_adapter,
        "dm_serial_baud": payload.dm_serial_baud,
        "use_can_fd": payload.use_can_fd,
        "can_bitrate": payload.can_bitrate,
        "can_data_bitrate": payload.can_data_bitrate,
    }
    if role == "leader":
        return OpenArmLeader(payload.port, **shared)
    if payload.side is None:
        msg = "OpenArm follower requires side='left' or side='right' for safety limits"
        raise ValueError(msg)
    return OpenArmFollower(
        payload.port,
        side=payload.side,
        disable_torque_on_disconnect=payload.disable_torque_on_disconnect,
        max_relative_target=payload.max_relative_target,
        **shared,
    )


async def _build_follower(robot: PayloadContainer[OpenArmPayload], factory: CatalogRobotFactory) -> PhysicalAIRobot:
    return await _build_single(robot, factory, role="follower")


async def _build_leader(robot: PayloadContainer[OpenArmPayload], factory: CatalogRobotFactory) -> PhysicalAIRobot:
    return await _build_single(robot, factory, role="leader")


async def _build_bimanual(
    robot: PayloadContainer[BimanualOpenArmPayload],
    factory: CatalogRobotFactory,
    *,
    role: Literal["follower", "leader"],
) -> PhysicalAIRobot:
    _ = factory
    payload = BimanualOpenArmPayload.model_validate(robot.payload)
    shared = {
        "can_adapter": payload.can_adapter,
        "dm_serial_baud": payload.dm_serial_baud,
        "use_can_fd": payload.use_can_fd,
        "can_bitrate": payload.can_bitrate,
        "can_data_bitrate": payload.can_data_bitrate,
    }
    if role == "leader":
        return BimanualOpenArmLeader(
            OpenArmLeader(payload.left_port, **shared),
            OpenArmLeader(payload.right_port, **shared),
        )
    return BimanualOpenArmFollower(
        OpenArmFollower(
            payload.left_port,
            side="left",
            disable_torque_on_disconnect=payload.disable_torque_on_disconnect,
            max_relative_target=payload.max_relative_target,
            **shared,
        ),
        OpenArmFollower(
            payload.right_port,
            side="right",
            disable_torque_on_disconnect=payload.disable_torque_on_disconnect,
            max_relative_target=payload.max_relative_target,
            **shared,
        ),
    )


async def _build_bimanual_follower(
    robot: PayloadContainer[BimanualOpenArmPayload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    return await _build_bimanual(robot, factory, role="follower")


async def _build_bimanual_leader(
    robot: PayloadContainer[BimanualOpenArmPayload],
    factory: CatalogRobotFactory,
) -> PhysicalAIRobot:
    return await _build_bimanual(robot, factory, role="leader")


def _definitions() -> list[RobotCatalogDefinition]:
    return [
        RobotCatalogDefinition(
            type="OpenArm_Follower",
            display_name="OpenArm Follower",
            role="follower",
            robot_builder=_build_follower,
            robot_payload=OpenArmPayload,
            asset=_SINGLE_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_PROBE,
        ),
        RobotCatalogDefinition(
            type="OpenArm_Leader",
            display_name="OpenArm Leader",
            role="leader",
            robot_builder=_build_leader,
            robot_payload=OpenArmPayload,
            asset=_SINGLE_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_PROBE,
        ),
        RobotCatalogDefinition(
            type="BimanualOpenArm_Follower",
            display_name="Bimanual OpenArm Follower",
            role="follower",
            robot_builder=_build_bimanual_follower,
            robot_payload=BimanualOpenArmPayload,
            asset=_BIMANUAL_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_PROBE,
        ),
        RobotCatalogDefinition(
            type="BimanualOpenArm_Leader",
            display_name="Bimanual OpenArm Leader",
            role="leader",
            robot_builder=_build_bimanual_leader,
            robot_payload=BimanualOpenArmPayload,
            asset=_BIMANUAL_ASSET,
            adapter_options=RobotAdapterOptions(include_velocities=True, external_effort_gain=None),
            probe=_PROBE,
        ),
    ]


def register_physicalai_studio_plugin(registry: _RobotCatalogRegistry) -> None:
    """Register direct OpenArm follower and leader definitions with Studio."""
    for definition in _definitions():
        registry.register_robot(definition)
