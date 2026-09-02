# ruff: noqa: DOC501, D107, PLR6301

"""PhysicalAI robot implementations for direct OpenArm control."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar, Literal

import numpy as np
from physicalai.config import export_config

from physicalai_openarm_plugin.constants import (
    DEFAULT_POSITION_KD,
    DEFAULT_POSITION_KP,
    LEFT_JOINT_LIMITS_DEG,
    NUM_OPENARM_JOINTS,
    OPENARM_JOINT_ORDER,
    OPENARM_MOTOR_CONFIG,
    RIGHT_JOINT_LIMITS_DEG,
)
from physicalai_openarm_plugin.damiao import DamiaoSerial, DamiaoSocketCAN

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation

OpenArmSide = Literal["left", "right"]
OpenArmCANAdapter = Literal["socketcan", "damiao"]


@dataclass
class OpenArmObservation:
    """OpenArm state in canonical joint order and degrees."""

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        """Primary position state vector."""
        return self.joint_positions


class _OpenArmBase:
    JOINT_ORDER: ClassVar[tuple[str, ...]] = OPENARM_JOINT_ORDER
    NUM_JOINTS: ClassVar[int] = NUM_OPENARM_JOINTS

    def __init__(
        self,
        port: str,
        *,
        can_adapter: OpenArmCANAdapter = "socketcan",
        dm_serial_baud: int = 921_600,
        use_can_fd: bool = True,
        can_bitrate: int = 1_000_000,
        can_data_bitrate: int = 5_000_000,
        response_timeout: float = 0.02,
        _transport: DamiaoSocketCAN | DamiaoSerial | None = None,
    ) -> None:
        if not port:
            msg = "port must be a non-empty CAN interface or Damiao USB serial device"
            raise ValueError(msg)
        if can_bitrate <= 0 or can_data_bitrate <= 0 or response_timeout <= 0:
            msg = "CAN bitrates and response_timeout must be positive"
            raise ValueError(msg)
        if can_adapter not in {"socketcan", "damiao"}:
            msg = "can_adapter must be 'socketcan' or 'damiao'"
            raise ValueError(msg)
        self._port = port
        self._can_adapter = can_adapter
        self._transport = _transport or self._make_transport(
            port=port,
            can_adapter=can_adapter,
            dm_serial_baud=dm_serial_baud,
            use_can_fd=use_can_fd,
            can_bitrate=can_bitrate,
            can_data_bitrate=can_data_bitrate,
            response_timeout=response_timeout,
        )

    @staticmethod
    def _make_transport(
        *,
        port: str,
        can_adapter: OpenArmCANAdapter,
        dm_serial_baud: int,
        use_can_fd: bool,
        can_bitrate: int,
        can_data_bitrate: int,
        response_timeout: float,
    ) -> DamiaoSocketCAN | DamiaoSerial:
        if can_adapter == "damiao":
            return DamiaoSerial(port, OPENARM_MOTOR_CONFIG, baud=dm_serial_baud)
        return DamiaoSocketCAN(
            port,
            OPENARM_MOTOR_CONFIG,
            use_can_fd=use_can_fd,
            bitrate=can_bitrate,
            data_bitrate=can_data_bitrate,
            response_timeout=response_timeout,
        )

    @property
    def port(self) -> str:
        """Configured SocketCAN channel or Damiao USB serial device."""
        return self._port

    @property
    def joint_names(self) -> list[str]:
        """Fixed arm-first joint order in degrees."""
        return list(self.JOINT_ORDER)

    @property
    def device_ids(self) -> tuple[str, ...]:
        """Stable identity of the exclusively owned CAN interface."""
        return (f"openarm:{self._can_adapter}:{self.port}",)

    def is_connected(self) -> bool:
        """Return whether the CAN transport is connected."""
        return self._transport.is_connected

    def _observation(self) -> OpenArmObservation:
        states = self._transport.read_states()
        positions = np.array([states[name].position for name in self.JOINT_ORDER], dtype=np.float32)
        velocities = np.array([states[name].velocity for name in self.JOINT_ORDER], dtype=np.float32)
        torques = np.array([states[name].torque for name in self.JOINT_ORDER], dtype=np.float32)
        return OpenArmObservation(positions, time.monotonic(), {"velocities": velocities, "torques": torques})


@export_config(class_path="physicalai_openarm_plugin.OpenArmFollower")
class OpenArmFollower(_OpenArmBase):
    """Direct OpenArm follower with side-specific position safety limits."""

    def __init__(
        self,
        port: str,
        *,
        side: OpenArmSide,
        can_adapter: OpenArmCANAdapter = "socketcan",
        dm_serial_baud: int = 921_600,
        disable_torque_on_disconnect: bool = True,
        use_can_fd: bool = True,
        can_bitrate: int = 1_000_000,
        can_data_bitrate: int = 5_000_000,
        response_timeout: float = 0.02,
        max_relative_target: float | None = None,
        position_kp: tuple[float, ...] = DEFAULT_POSITION_KP,
        position_kd: tuple[float, ...] = DEFAULT_POSITION_KD,
        _transport: DamiaoSocketCAN | DamiaoSerial | None = None,
    ) -> None:
        if side not in {"left", "right"}:
            msg = "side must be 'left' or 'right'; OpenArm followers require explicit safety limits"
            raise ValueError(msg)
        if max_relative_target is not None and (not math.isfinite(max_relative_target) or max_relative_target <= 0):
            msg = "max_relative_target must be a finite positive number when provided"
            raise ValueError(msg)
        if len(position_kp) != self.NUM_JOINTS or len(position_kd) != self.NUM_JOINTS:
            msg = f"position_kp and position_kd must each contain {self.NUM_JOINTS} values"
            raise ValueError(msg)
        super().__init__(
            port,
            can_adapter=can_adapter,
            dm_serial_baud=dm_serial_baud,
            use_can_fd=use_can_fd,
            can_bitrate=can_bitrate,
            can_data_bitrate=can_data_bitrate,
            response_timeout=response_timeout,
            _transport=_transport,
        )
        self.side = side
        self.disable_torque_on_disconnect = disable_torque_on_disconnect
        self.max_relative_target = max_relative_target
        self.position_kp = position_kp
        self.position_kd = position_kd
        self._limits = LEFT_JOINT_LIMITS_DEG if side == "left" else RIGHT_JOINT_LIMITS_DEG

    def connect(self) -> None:
        """Connect, validate all eight motors, and enable follower torque."""
        self._transport.connect()

    def disconnect(self) -> None:
        """Release the CAN transport, disabling torque by default."""
        self._transport.disconnect(disable_torque=self.disable_torque_on_disconnect)

    def get_observation(self) -> RobotObservation:
        """Return positions, velocities, and torques from all motors."""
        return self._observation()

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Clip and send a full 8-element degree position target vector."""
        _ = goal_time
        if action.shape != (self.NUM_JOINTS,):
            msg = f"Expected action shape ({self.NUM_JOINTS},), got {action.shape}"
            raise ValueError(msg)
        if not np.isfinite(action).all():
            msg = "OpenArm actions must contain only finite values"
            raise ValueError(msg)
        current = self._transport.read_states() if self.max_relative_target is not None else None
        commands: dict[str, tuple[float, float, float]] = {}
        for index, name in enumerate(self.JOINT_ORDER):
            lower, upper = self._limits[name]
            target = float(np.clip(action[index], lower, upper))
            if current is not None:
                position = current[name].position
                target = float(
                    np.clip(target, position - self.max_relative_target, position + self.max_relative_target),
                )
            commands[name] = (self.position_kp[index], self.position_kd[index], target)
        self._transport.send_positions(commands)


@export_config(class_path="physicalai_openarm_plugin.OpenArmLeader")
class OpenArmLeader(_OpenArmBase):
    """Direct, read-only OpenArm leader for hand-guided unilateral teleoperation."""

    def __init__(
        self,
        port: str,
        *,
        manual_control: bool = True,
        can_adapter: OpenArmCANAdapter = "socketcan",
        dm_serial_baud: int = 921_600,
        use_can_fd: bool = True,
        can_bitrate: int = 1_000_000,
        can_data_bitrate: int = 5_000_000,
        response_timeout: float = 0.02,
        _transport: DamiaoSocketCAN | DamiaoSerial | None = None,
    ) -> None:
        super().__init__(
            port,
            can_adapter=can_adapter,
            dm_serial_baud=dm_serial_baud,
            use_can_fd=use_can_fd,
            can_bitrate=can_bitrate,
            can_data_bitrate=can_data_bitrate,
            response_timeout=response_timeout,
            _transport=_transport,
        )
        self.manual_control = manual_control

    def connect(self) -> None:
        """Connect and leave torque disabled when configured for hand guidance."""
        self._transport.connect()
        if self.manual_control:
            self._transport.disable_torque()

    def disconnect(self) -> None:
        """Release the leader bus and retain its manual-control torque state."""
        self._transport.disconnect(disable_torque=self.manual_control)

    def get_observation(self) -> RobotObservation:
        """Return the leader's current degree positions and measured state."""
        return self._observation()

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Ignore runtime writes because OpenArm leader feedback is unsupported."""
        _ = action, goal_time
