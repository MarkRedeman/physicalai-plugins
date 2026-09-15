# ruff: noqa: DOC201, DOC501, PLR6301, S101, UP046

"""Bimanual composition for direct OpenArm drivers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Generic, TypeVar

import numpy as np
from physicalai.config import export_config

from physicalai_openarm_plugin.constants import NUM_BIMANUAL_OPENARM_JOINTS, NUM_OPENARM_JOINTS
from physicalai_openarm_plugin.openarm import OpenArmFollower, OpenArmLeader, OpenArmObservation

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation

ArmT = TypeVar("ArmT", OpenArmFollower, OpenArmLeader)


@dataclass
class BimanualOpenArmObservation:
    """Combined left-then-right OpenArm observation."""

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        """Combined primary state vector."""
        return self.joint_positions


class _BimanualOpenArm(Generic[ArmT]):
    NUM_JOINTS = NUM_BIMANUAL_OPENARM_JOINTS

    def __init__(self, left: ArmT, right: ArmT) -> None:
        if type(left) is not type(right):
            msg = "Both OpenArm instances must have the same driver type"
            raise ValueError(msg)
        if left.port == right.port:
            msg = "Left and right OpenArms must use distinct SocketCAN interfaces"
            raise ValueError(msg)
        self.left = left
        self.right = right

    @property
    def joint_names(self) -> list[str]:
        """Left then right prefixed canonical joint names."""
        return [f"left_{name}" for name in self.left.joint_names] + [f"right_{name}" for name in self.right.joint_names]

    @property
    def device_ids(self) -> tuple[str, ...]:
        """Stable identities for both independent CAN interfaces."""
        return tuple(sorted(self.left.device_ids + self.right.device_ids))

    def connect(self) -> None:
        """Connect both arms and roll back left if right fails."""
        self.left.connect()
        try:
            self.right.connect()
        except Exception:
            self.left.disconnect()
            raise

    def disconnect(self) -> None:
        """Disconnect both arms even when one teardown fails."""
        first_error: Exception | None = None
        for arm in (self.left, self.right):
            try:
                arm.disconnect()
            except Exception as error:  # noqa: BLE001  # pragma: no cover - hardware defensive path
                first_error = first_error or error
        if first_error is not None:
            raise first_error

    def is_connected(self) -> bool:
        """Return whether both arms are connected."""
        return self.left.is_connected() and self.right.is_connected()

    def get_observation(self) -> RobotObservation:
        """Read and concatenate both arm states."""
        left = self.left.get_observation()
        right = self.right.get_observation()
        assert isinstance(left, OpenArmObservation)
        assert isinstance(right, OpenArmObservation)
        sensor_data = None
        if left.sensor_data is not None and right.sensor_data is not None:
            sensor_data = {
                name: np.concatenate((left.sensor_data[name], right.sensor_data[name]))
                for name in left.sensor_data.keys() & right.sensor_data.keys()
            }
        return BimanualOpenArmObservation(
            np.concatenate((left.joint_positions, right.joint_positions)),
            left.timestamp,
            sensor_data,
        )


@export_config(class_path="physicalai_openarm_plugin.BimanualOpenArmFollower")
class BimanualOpenArmFollower(_BimanualOpenArm[OpenArmFollower]):
    """Two directly controlled OpenArm followers."""

    def __init__(self, left: OpenArmFollower, right: OpenArmFollower) -> None:
        """Initialize the paired follower drivers."""
        super().__init__(left, right)

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Split a 16-element target vector and send it to both followers."""
        if action.shape != (self.NUM_JOINTS,):
            msg = f"Expected action shape ({self.NUM_JOINTS},), got {action.shape}"
            raise ValueError(msg)
        self.left.send_action(action[:NUM_OPENARM_JOINTS], goal_time=goal_time)
        self.right.send_action(action[NUM_OPENARM_JOINTS:], goal_time=goal_time)


@export_config(class_path="physicalai_openarm_plugin.BimanualOpenArmLeader")
class BimanualOpenArmLeader(_BimanualOpenArm[OpenArmLeader]):
    """Two read-only hand-guided OpenArm leaders."""

    def __init__(self, left: OpenArmLeader, right: OpenArmLeader) -> None:
        """Initialize the paired leader drivers."""
        super().__init__(left, right)

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Reject feedback writes because OpenArm leaders are read-only."""
        _ = action, goal_time
        msg = "Cannot send actions to OpenArm leaders. Bilateral feedback is not implemented."
        raise RuntimeError(msg)
