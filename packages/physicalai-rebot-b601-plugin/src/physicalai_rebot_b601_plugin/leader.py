"""FashionStar leader arm driver for teleoperation.

Uses the ``motorbridge_smart_servo`` SDK to communicate with FashionStar
UART servos on the Star Arm 102 leader arm. This driver is read-only and
raises :class:`RuntimeError` on any attempt to send actions.
"""

from __future__ import annotations

import contextlib
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar, Protocol

import numpy as np
from loguru import logger

from physicalai_rebot_b601_plugin.constants import (
    REBOT_ARM_102_JOINT_IDS,
    REBOT_ARM_102_JOINT_ORDER,
    REBOT_ARM_102_JOINT_RANGES_DEG,
)

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation

from motorbridge_smart_servo import FashionStarServo


class _AngleSample(Protocol):
    """Protocol for a single angle-read sample from a FashionStar servo."""

    raw_deg: float
    filtered_deg: float
    reliable: bool


class _FashionStarBus(Protocol):
    """Protocol for the FashionStar UART servo bus."""

    def ping(self, servo_id: int) -> bool: ...
    def unlock(self, servo_id: int) -> None: ...
    def reset_multi_turn(self, servo_id: int) -> None: ...
    def set_origin_point(self, servo_id: int) -> None: ...
    def read_angle(self, servo_id: int, *, multi_turn: bool = True) -> _AngleSample: ...
    def close(self) -> None: ...


@dataclass
class ReBotArm102LeaderObservation:
    """Observation data for the reBot Arm 102 leader.

    Attributes:
        joint_positions: Measured joint positions in degrees.
        timestamp: Monotonic time of the observation.
        sensor_data: Optional dict of raw positions and reliability flags.
        images: Optional camera frames.
    """

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        """Alias for joint positions, matching the Robot protocol."""
        return self.joint_positions


class ReBotArm102Leader:
    """FashionStar UART leader arm driver (read-only teleoperation).

    Connects to the Star Arm 102 leader arm via a UART-to-USB adapter.
    This arm has no torque control; it is manually positioned by the
    operator and the driver only reads joint angles.
    """

    JOINT_ORDER: ClassVar[list[str]] = list(REBOT_ARM_102_JOINT_ORDER)
    NUM_JOINTS: ClassVar[int] = len(JOINT_ORDER)

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        *,
        baudrate: int = 1_000_000,
        unlock_on_connect: bool = True,
        reset_multi_turn_on_connect: bool = True,
        zero_on_connect: bool = False,
    ) -> None:
        """Initialize the FashionStar leader arm driver.

        Args:
            port: UART serial port (e.g. ``/dev/ttyUSB0``).
            baudrate: Serial baud rate for the FashionStar bus.
            unlock_on_connect: Whether to unlock servos on connect.
            reset_multi_turn_on_connect: Whether to reset multi-turn counters on connect.
            zero_on_connect: Whether to set the current position as origin point.

        Raises:
            ValueError: If any parameter has an invalid value.
        """
        if baudrate <= 0:
            msg = f"baudrate must be a positive integer, got {baudrate!r}"
            raise ValueError(msg)

        self._port = port
        self._baudrate = baudrate
        self._unlock_on_connect = unlock_on_connect
        self._reset_multi_turn_on_connect = reset_multi_turn_on_connect
        self._zero_on_connect = zero_on_connect
        self._bus: _FashionStarBus | None = None
        self._last_positions: np.ndarray | None = None
        self._last_raw_positions: np.ndarray | None = None
        self._last_reliable: np.ndarray | None = None

    @property
    def joint_names(self) -> list[str]:
        """Ordered list of joint names matching the expected observation layout."""
        return self.JOINT_ORDER

    @property
    def port(self) -> str:
        """UART serial port the driver is configured for."""
        return self._port

    @property
    def baudrate(self) -> int:
        """Serial baud rate for the FashionStar bus."""
        return self._baudrate

    @property
    def zero_on_connect(self) -> bool:
        """Whether the current position is set as origin point on connect."""
        return self._zero_on_connect

    def _require_bus(self) -> _FashionStarBus:
        bus = self._bus
        if bus is None:
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)
        return bus

    def connect(self) -> None:
        """Open the UART bus, ping all servos, and configure them."""
        if self.is_connected():
            return

        bus = FashionStarServo(self.port, baudrate=self.baudrate)
        try:
            self._ping_servos(bus)
            self._bus = bus
            self._configure_servos(bus)
        except Exception:
            with contextlib.suppress(Exception):
                bus.close()
            self._bus = None
            raise

        logger.info(f"ReBotArm102Leader connected on {self.port}")

    def disconnect(self) -> None:
        """Close the UART bus and release resources."""
        bus = self._bus
        if bus is None:
            return
        self._bus = None
        bus.close()
        logger.info(f"ReBotArm102Leader disconnected from {self.port}")

    def is_connected(self) -> bool:
        """Return whether the UART bus connection is active."""
        return self._bus is not None

    def _ping_servos(self, bus: _FashionStarBus) -> None:
        for name in self.JOINT_ORDER:
            servo_id = REBOT_ARM_102_JOINT_IDS[name]
            if not bus.ping(servo_id):
                msg = f"Servo '{name}' (ID {servo_id}) did not respond on {self.port}."
                raise ConnectionError(msg)

    def _configure_servos(self, bus: _FashionStarBus) -> None:
        for name in self.JOINT_ORDER:
            servo_id = REBOT_ARM_102_JOINT_IDS[name]
            if self._unlock_on_connect:
                bus.unlock(servo_id)
            if self._zero_on_connect:
                bus.set_origin_point(servo_id)
            if self._reset_multi_turn_on_connect:
                bus.reset_multi_turn(servo_id)

    def get_observation(self) -> RobotObservation:
        """Read joint positions from all servos.

        Falls back to the last valid sample if a read fails, as long as at
        least one successful read has occurred.

        Returns:
            A ``ReBotArm102LeaderObservation`` with joint positions in degrees
            and sensor data including raw positions and reliability flags.

        Raises:
            ConnectionError: If no prior sample exists and the read fails.
        """
        bus = self._require_bus()
        try:
            positions, raw_positions, reliable = self._read_positions(bus)
            self._last_positions = positions
            self._last_raw_positions = raw_positions
            self._last_reliable = reliable
        except Exception as e:
            if self._last_positions is None or self._last_raw_positions is None or self._last_reliable is None:
                msg = f"Failed to read reBot Arm 102 leader positions: {e}"
                raise ConnectionError(msg) from e
            logger.warning(f"Failed to read reBot Arm 102 leader positions; using last valid sample: {e}")
            positions = self._last_positions.copy()
            raw_positions = self._last_raw_positions.copy()
            reliable = np.zeros(self.NUM_JOINTS, dtype=np.float32)

        return ReBotArm102LeaderObservation(
            joint_positions=positions,
            timestamp=time.monotonic(),
            sensor_data={
                "raw_positions": raw_positions,
                "reliable": reliable,
            },
        )

    def _read_positions(self, bus: _FashionStarBus) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        positions = np.empty(self.NUM_JOINTS, dtype=np.float32)
        raw_positions = np.empty(self.NUM_JOINTS, dtype=np.float32)
        reliable = np.empty(self.NUM_JOINTS, dtype=np.float32)

        for i, name in enumerate(self.JOINT_ORDER):
            servo_id = REBOT_ARM_102_JOINT_IDS[name]
            sample = bus.read_angle(servo_id, multi_turn=True)
            range_min, range_max = REBOT_ARM_102_JOINT_RANGES_DEG[name]
            unwrapped, _ = self._round_to_valid_range(float(sample.filtered_deg), range_min, range_max)
            positions[i] = float(np.clip(unwrapped, range_min, range_max))
            raw_positions[i] = float(sample.raw_deg)
            reliable[i] = 1.0 if sample.reliable else 0.0

        return positions, raw_positions, reliable

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Raise an error — leader arms are read-only.

        Args:
            action: Ignored; present for protocol compatibility.
            goal_time: Ignored; present for protocol compatibility.

        Raises:
            RuntimeError: Always raised.
        """
        msg = "Cannot send actions to ReBotArm102Leader. Leader arms are read-only for teleoperation."
        raise RuntimeError(msg)

    @staticmethod
    def _round_to_valid_range(value: float, min_value: float, max_value: float) -> tuple[float, int]:
        center = (min_value + max_value) / 2.0
        low = center - 180.0
        high = center + 180.0
        for k in range(4096):
            candidate_plus = value + k * 360.0
            if low <= candidate_plus <= high:
                return candidate_plus, k
            candidate_minus = value - k * 360.0
            if low <= candidate_minus <= high:
                return candidate_minus, k
        return value - round((value - center) / 360.0) * 360.0, 4096
