"""Adapter that wraps a lerobot.robots.robot.Robot into PhysicalAI's Robot protocol.

Joint order, observation keys, and action keys are auto-detected from the
lerobot robot's observation dict on the first ``get_observation()`` call
(typically made during ``connect()``).
"""

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

_POSITION_KEY_SUFFIX: str = ".pos"


class LeRobotAdapter:
    """Wraps a lerobot.robots.robot.Robot into PhysicalAI's Robot protocol.

    Joint names and observation/action key mappings are auto-detected from
    the lerobot observation dict on the first call to ``get_observation()``
    (invoked automatically by ``connect()``).  The adapter looks for keys
    ending with ``.pos``, sorts them alphabetically, and derives the joint
    order from those keys.

    Attributes:
        NUM_JOINTS: Number of joints, available after the first observation.
    """

    def __init__(
        self,
        robot: LeRobotRobot,
        *,
        role: Literal["leader", "follower"] = "follower",
    ) -> None:
        """Initialize the adapter.

        Args:
            robot: A lerobot Robot instance to wrap.
            role: ``"follower"`` (full control) or ``"leader"`` (read-only).

        Raises:
            ValueError: If role is invalid.
        """
        if role not in VALID_ROLES:
            msg = f"Invalid role {role!r}. Must be one of {sorted(VALID_ROLES)}."
            raise ValueError(msg)

        self._robot = robot
        self._role = role
        self._joint_order: list[str] | None = None
        self._obs_position_keys: list[str] | None = None
        self._act_position_keys: list[str] | None = None
        self._num_joints: int | None = None

    def _ensure_joint_order(self, obs: dict[str, Any]) -> None:
        """Discover joint order from a lerobot observation dict.

        Keys ending with ``.pos`` are treated as joint position keys, sorted
        alphabetically, and stripped of the suffix to produce the joint names.

        Args:
            obs: The lerobot observation dict.
        """
        if self._joint_order is not None:
            return
        pos_keys = sorted(k for k in obs if k.endswith(_POSITION_KEY_SUFFIX))
        if not pos_keys:
            pos_keys = sorted(
                k for k in obs if "position" in k.lower() or k.endswith("pos")
            )
        self._joint_order = [k[: -len(_POSITION_KEY_SUFFIX)] for k in pos_keys]
        self._num_joints = len(self._joint_order)
        self._obs_position_keys = list(pos_keys)
        self._act_position_keys = list(pos_keys)

    def _require_joint_order(self) -> None:
        """Raise if joint order has not been discovered yet.

        Raises:
            RuntimeError: If ``connect()`` has not been called.
        """
        if self._joint_order is None:
            msg = (
                "Joint order not yet discovered. Call connect() first "
                "or ensure the robot has been observed."
            )
            raise RuntimeError(msg)

    @property
    def NUM_JOINTS(self) -> int:  # noqa: N802
        """Number of joints (available after first observation)."""
        self._require_joint_order()
        return self._num_joints  # type: ignore[return-value]

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names matching the position/action array."""
        self._require_joint_order()
        return self._joint_order  # type: ignore[return-value]

    @property
    def robot(self) -> LeRobotRobot:
        """The wrapped lerobot Robot instance."""
        return self._robot

    def connect(self) -> None:
        """Open the connection and discover joint order.

        Idempotent — no-op if already connected.
        Calls ``get_observation()`` once to auto-detect joint names.

        Raises:
            ConnectionError: If the connection or initial observation fails.
        """
        if self.is_connected():
            return
        try:
            self._robot.connect(calibrate=True)
            obs = self._robot.get_observation()
            self._ensure_joint_order(obs)
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

        Joint order is auto-detected on the first call.

        Returns:
            LeRobotAdapterObservation with joint positions and sensor data.
        """
        lerobot_obs = self._robot.get_observation()
        self._ensure_joint_order(lerobot_obs)
        self._require_joint_order()

        positions = np.array(
            [lerobot_obs[key] for key in self._obs_position_keys],  # type: ignore[union-attr]
            dtype=np.float32,
        )

        sensor_data: dict[str, np.ndarray] = {}
        images: dict[str, Frame] = {}
        obs_key_set = set(self._obs_position_keys)  # type: ignore[union-attr]
        act_key_set = set(self._act_position_keys)  # type: ignore[union-attr]
        for key, value in lerobot_obs.items():
            if key in obs_key_set or key in act_key_set:
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
            goal_time: Time to reach the goal in seconds (ignored).

        Raises:
            RuntimeError: If called in leader role or before ``connect()``.
            ValueError: If action has incorrect shape.
        """
        _ = goal_time
        if self._role == "leader":
            msg = "Cannot send actions to a leader arm."
            raise RuntimeError(msg)

        self._require_joint_order()
        if action.shape != (self._num_joints,):
            msg = f"Expected action shape ({self._num_joints},), got {action.shape}"
            raise ValueError(msg)

        action_dict: dict[str, Any] = {}
        for i, key in enumerate(self._act_position_keys):  # type: ignore[union-attr]
            action_dict[key] = float(action[i])

        self._robot.send_action(action_dict)
