"""Adapter that wraps a lerobot.robots.robot.Robot into PhysicalAI's Robot protocol."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, cast

import numpy as np
from loguru import logger

from physicalai_lerobot_plugin.constants import VALID_ROLES

if TYPE_CHECKING:
    from lerobot.robots.robot import Robot as LeRobotRobot
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation


@dataclass
class LeRobotAdapterObservation:
    """Observation from a LeRobot-wrapped robot.

    Attributes:
        joint_positions: Array of shape ``(N,)`` matching ``joint_names`` order.
        timestamp: ``time.monotonic()`` at capture.
        sensor_data: Optional auxiliary sensor readings.
        images: Optional built-in camera frames.
    """

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        """Joint positions as the primary state vector."""
        return self.joint_positions


_DIM_THRESHOLD_IMAGE: int = 2


class LeRobotAdapter:
    """Wraps a lerobot.robots.robot.Robot into PhysicalAI's Robot protocol.

    Converts between LeRobot's dict-based observation/action format and
    PhysicalAI's numpy-array-based format.

    Attributes:
        NUM_JOINTS: Number of joints, set from `joint_order` length.
    """

    def __init__(
        self,
        robot: LeRobotRobot,
        joint_order: list[str],
        *,
        obs_position_keys: list[str] | None = None,
        act_position_keys: list[str] | None = None,
        role: Literal["leader", "follower"] = "follower",
    ) -> None:
        """Initialize the adapter.

        Args:
            robot: A lerobot Robot instance to wrap.
            joint_order: Joint names in the order expected by PhysicalAI.
            obs_position_keys: Keys in the lerobot observation dict to extract
                as the joint position vector. Defaults to ``["{name}.pos", ...]``.
            act_position_keys: Keys in the lerobot action dict to populate from
                the action array. Defaults to ``obs_position_keys``.
            role: ``"follower"`` (full control) or ``"leader"`` (read-only).

        Raises:
            ValueError: If role is invalid, or key list lengths don't match.
        """
        if role not in VALID_ROLES:
            msg = f"Invalid role {role!r}. Must be one of {sorted(VALID_ROLES)}."
            raise ValueError(msg)

        self._robot = robot
        self._joint_order = list(joint_order)
        self._num_joints = len(joint_order)
        self._role = role

        self._obs_position_keys = (
            obs_position_keys if obs_position_keys is not None else [f"{j}.pos" for j in joint_order]
        )
        self._act_position_keys = act_position_keys if act_position_keys is not None else self._obs_position_keys

        if len(self._obs_position_keys) != self._num_joints:
            msg = (
                f"obs_position_keys length ({len(self._obs_position_keys)}) "
                f"must match joint_order length ({self._num_joints})"
            )
            raise ValueError(msg)
        if len(self._act_position_keys) != self._num_joints:
            msg = (
                f"act_position_keys length ({len(self._act_position_keys)}) "
                f"must match joint_order length ({self._num_joints})"
            )
            raise ValueError(msg)

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names matching the position/action array."""
        return self._joint_order

    @property
    def robot(self) -> LeRobotRobot:
        """The wrapped lerobot Robot instance."""
        return self._robot

    def connect(self) -> None:
        """Open the connection. Idempotent — no-op if already connected.

        Raises:
            ConnectionError: If the connection fails.
        """
        if self.is_connected():
            return
        try:
            self._robot.connect(calibrate=True)
        except Exception as e:
            msg = f"Failed to connect LeRobot {self._robot}: {e}"
            raise ConnectionError(msg) from e

    def disconnect(self) -> None:
        """Close the connection. Safe to call multiple times."""
        if not self.is_connected():
            return
        try:
            self._robot.disconnect()
        except Exception:  # noqa: BLE001
            logger.exception("Error during LeRobot disconnect")

    def is_connected(self) -> bool:
        """Check if the robot is currently connected.

        Returns:
            True if connected.
        """
        return self._robot.is_connected

    def get_observation(self) -> RobotObservation:
        """Read current joint positions and return an observation.

        Returns:
            LeRobotAdapterObservation with joint positions and sensor data.
        """
        lerobot_obs = self._robot.get_observation()

        positions = np.array(
            [lerobot_obs[key] for key in self._obs_position_keys],
            dtype=np.float32,
        )

        sensor_data: dict[str, np.ndarray] = {}
        images: dict[str, Frame] = {}
        for key, value in lerobot_obs.items():
            if key in self._obs_position_keys or key in self._act_position_keys:
                continue
            if isinstance(value, np.ndarray) and value.ndim >= _DIM_THRESHOLD_IMAGE:
                images[key] = cast("Frame", value)
            elif isinstance(value, np.ndarray):
                sensor_data[key] = value
            elif isinstance(value, (int, float)):
                sensor_data[key] = np.array([value], dtype=np.float32)

        return LeRobotAdapterObservation(
            joint_positions=positions,
            timestamp=time.monotonic(),
            sensor_data=sensor_data or None,
            images=images or None,
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Send a joint command to the robot.

        Args:
            action: Array of shape ``(N,)`` matching ``joint_names`` order.
            goal_time: Time to reach the goal in seconds (forwarded to lerobot).

        Raises:
            RuntimeError: If called in leader role.
            ValueError: If action has incorrect shape.
        """
        _ = goal_time
        if self._role == "leader":
            msg = "Cannot send actions to a leader arm."
            raise RuntimeError(msg)

        if action.shape != (self._num_joints,):
            msg = f"Expected action shape ({self._num_joints},), got {action.shape}"
            raise ValueError(msg)

        action_dict: dict[str, Any] = {}
        for i, key in enumerate(self._act_position_keys):
            action_dict[key] = float(action[i])

        self._robot.send_action(action_dict)
