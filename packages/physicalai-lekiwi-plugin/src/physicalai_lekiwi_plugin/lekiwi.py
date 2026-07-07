"""LeKiwi mobile manipulator driver (6-DOF SO-ARM100 arm + 3-wheel holonomic base).

Uses direct serial communication via ``scservo_sdk`` (Feetech STS3215 servos).
Arm motors run in POSITION mode; base wheel motors run in VELOCITY mode.
"""

from __future__ import annotations

import contextlib
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, ClassVar, Literal

import numpy as np
from loguru import logger
from scservo_sdk import GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler

from physicalai_lekiwi_plugin.calibration import LeKiwiCalibration
from physicalai_lekiwi_plugin.constants import (
    ARM_D_COEFFICIENT,
    ARM_I_COEFFICIENT,
    ARM_P_COEFFICIENT,
    BASE_RADIUS,
    LEKIWI_ARM_JOINTS,
    LEKIWI_BASE_JOINTS,
    LEKIWI_JOINT_ORDER,
    LEKIWI_MOTOR_IDS,
    MAX_RAW_SPEED_NEGATIVE,
    MAX_RAW_SPEED_POSITIVE,
    MAX_RAW_WHEEL,
    POSITION_MODE,
    PROTOCOL_VERSION,
    STEPS_PER_DEG,
    TICKS_PER_REVOLUTION,
    VALID_ROLES,
    VELOCITY_MODE,
    WHEEL_ANGLES_DEG,
    WHEEL_OFFSET_DEG,
    WHEEL_RADIUS,
    STS3215Addr,
    STS3215Len,
)

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation

LeKiwiUnit = Literal["ticks", "normalized"]


@dataclass(frozen=True)
class _LeKiwiConnection:
    port_handler: Any
    packet_handler: Any
    arm_group_sync_read: Any
    arm_group_sync_write: Any
    base_group_sync_read: Any
    base_group_sync_write: Any


@dataclass
class LeKiwiObservation:
    """Observation from the LeKiwi robot.

    Attributes:
        joint_positions: Array of shape ``(9,)`` with joint positions (6 arm + 3 base velocities).
        timestamp: ``time.monotonic()`` at the moment of capture.
        sensor_data: Optional dict of body-frame velocities and wheel feedback.
    """

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        """State vector: joint positions (9,)."""
        return self.joint_positions


class LeKiwi:
    """Driver for the LeKiwi robot (6-DOF arm + 3-wheel holonomic base).

    Args:
        port: Serial port path, e.g. ``"/dev/ttyACM0"``.
        baudrate: Serial baudrate. Defaults to 1 000 000 (STS3215 factory default).
        role: ``"follower"`` (torque enabled, full control) or ``"leader"``
            (torque disabled, read-only for teleoperation).
        calibration: Calibration object or JSON path. Required for normal operation.
        unit: Joint-space command/observation unit. Defaults to ``"normalized"``.
    """

    JOINT_ORDER: ClassVar[list[str]] = list(LEKIWI_JOINT_ORDER)
    NUM_JOINTS: ClassVar[int] = 9

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 1_000_000,
        role: Literal["leader", "follower"] = "follower",
        calibration: LeKiwiCalibration | str | Path | None = None,
        unit: LeKiwiUnit = "normalized",
        *,
        _allow_uncalibrated: bool = False,
    ) -> None:
        """Initialize the LeKiwi driver (does not open the connection).

        Args:
            port: Serial port path.
            baudrate: Serial baudrate.
            role: ``"follower"`` or ``"leader"``.
            calibration: Calibration object or path.
            unit: ``"normalized"`` or ``"ticks"``.
            _allow_uncalibrated: Skip calibration requirement (for testing).

        Raises:
            ValueError: If role is invalid.
        """
        if role not in VALID_ROLES:
            msg = f"Invalid role {role!r}. Must be one of {sorted(VALID_ROLES)}."
            raise ValueError(msg)

        self._port = port
        self._baudrate = baudrate
        self._role = role
        self._unit: LeKiwiUnit = unit

        if calibration is None and not _allow_uncalibrated:
            msg = (
                "calibration is required for LeKiwi. "
                "Pass a calibration object/path, or use LeKiwi.uncalibrated(...) "
                "for explicit raw-ticks bringup mode."
            )
            raise ValueError(msg)

        if isinstance(calibration, (str, Path)):
            calibration = LeKiwiCalibration.from_path(calibration)

        self._calibration: LeKiwiCalibration | None = calibration
        self._uncalibrated_mode = self._calibration is None
        self._validate_unit()
        self._warned_uncalibrated = False

        if self._calibration is not None:
            self.servo_ids = {name: self._calibration.joints[name].id for name in self.JOINT_ORDER}
        else:
            self.servo_ids = LEKIWI_MOTOR_IDS.copy()

        self._connection: _LeKiwiConnection | None = None
        self._torque_on_disconnect: bool = role == "follower"

    @classmethod
    def uncalibrated(
        cls,
        port: str = "/dev/ttyACM0",
        baudrate: int = 1_000_000,
        role: Literal["leader", "follower"] = "follower",
        unit: LeKiwiUnit = "ticks",
    ) -> LeKiwi:
        """Create an uncalibrated LeKiwi instance in raw-ticks mode.

        Intended for bringup/debug only. Observations and actions use raw servo ticks (0-4095).

        Returns:
            LeKiwi: An uncalibrated driver instance.
        """
        return cls(
            port=port,
            calibration=None,
            baudrate=baudrate,
            role=role,
            unit=unit,
            _allow_uncalibrated=True,
        )

    @property
    def joint_names(self) -> list[str]:
        """Ordered list of joint names (6 arm + 3 base)."""
        return self.JOINT_ORDER

    @property
    def port(self) -> str:
        """Serial port path."""
        return self._port

    @port.setter
    def port(self, value: str) -> None:
        self._port = value

    @property
    def baudrate(self) -> int:
        """Serial baudrate."""
        return self._baudrate

    @baudrate.setter
    def baudrate(self, value: int) -> None:
        if value <= 0:
            msg = f"baudrate must be a positive integer, got {value!r}"
            raise ValueError(msg)
        self._baudrate = value

    @property
    def role(self) -> Literal["leader", "follower"]:
        """Current role (``"leader"`` or ``"follower"``)."""
        return self._role

    @role.setter
    def role(self, value: Literal["leader", "follower"]) -> None:
        if value not in VALID_ROLES:
            msg = f"Invalid role {value!r}. Must be one of {sorted(VALID_ROLES)}."
            raise ValueError(msg)
        self._role = value

    @property
    def calibrated(self) -> bool:
        """Whether a calibration object has been loaded."""
        return self._calibration is not None

    @property
    def unit(self) -> LeKiwiUnit:
        """Current unit mode (``"normalized"`` or ``"ticks"``)."""
        return self._unit

    @unit.setter
    def unit(self, value: LeKiwiUnit) -> None:
        self._unit = value
        self._validate_unit()

    @property
    def torque_on_disconnect(self) -> bool:
        """Whether to disable torque on disconnect."""
        return self._torque_on_disconnect

    @torque_on_disconnect.setter
    def torque_on_disconnect(self, value: bool) -> None:
        if self.role != "follower" and value:
            msg = "Torque on disconnect can only be enabled for follower arms."
            raise ValueError(msg)
        if not value and self._torque_on_disconnect:
            logger.warning(
                "Disabling torque on disconnect will cause the arm to drop under gravity. Ensure this is intentional.",
            )
        self._torque_on_disconnect = value

    def _validate_unit(self) -> None:
        valid_units = {"ticks", "normalized"}
        if self._unit not in valid_units:
            msg = f"Invalid unit {self._unit!r}. Must be one of {sorted(valid_units)}."
            raise ValueError(msg)
        if self._calibration is None and self._unit != "ticks":
            msg = "Uncalibrated mode only supports unit='ticks'."
            raise ValueError(msg)
        if self._calibration is not None and self._unit == "ticks":
            msg = "Calibrated mode does not support unit='ticks'. Use 'normalized'."
            raise ValueError(msg)

    @staticmethod
    def _validate_conversion_unit(unit: LeKiwiUnit) -> None:
        if unit not in {"ticks", "normalized"}:
            msg = f"Invalid unit {unit!r}."
            raise ValueError(msg)

    def _resolve_unit(self, unit: LeKiwiUnit | None) -> LeKiwiUnit:
        resolved = self.unit if unit is None else unit
        self._validate_conversion_unit(resolved)
        if self._calibration is None and resolved != "ticks":
            msg = "Uncalibrated mode only supports unit='ticks'."
            raise ValueError(msg)
        if self._calibration is not None and resolved == "ticks":
            msg = "Calibrated mode does not support unit='ticks'. Use 'normalized'."
            raise ValueError(msg)
        return resolved

    def _require_connection(self) -> _LeKiwiConnection:
        """Return the active connection or raise.

        Returns:
            _LeKiwiConnection: The active serial connection.

        Raises:
            ConnectionError: If the robot is not connected.
        """
        conn = self._connection
        if conn is None:
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)
        return conn

    def _require_calibration(self) -> LeKiwiCalibration:
        if self._calibration is None:
            msg = (
                "Calibration is required for tick/unit conversion. "
                "Provide calibration or avoid conversion methods in uncalibrated mode."
            )
            raise RuntimeError(msg)
        return self._calibration

    def connect(self) -> None:
        """Open the serial port, ping all servos, and configure torque.

        Raises:
            ConnectionError: If the serial port cannot be opened or any servo fails to respond.
        """
        if self.is_connected():
            return

        port_handler = PortHandler(self.port)
        if not port_handler.openPort():
            msg = f"Failed to open serial port {self.port}"
            raise ConnectionError(msg)

        port_handler.setPacketTimeoutMillis(50.0)

        if not port_handler.setBaudRate(self.baudrate):
            port_handler.closePort()
            msg = f"Failed to set baudrate {self.baudrate} on {self.port}"
            raise ConnectionError(msg)

        packet_handler = PacketHandler(PROTOCOL_VERSION)

        try:
            arm_servo_ids = [self.servo_ids[name] for name in LEKIWI_ARM_JOINTS]
            base_servo_ids = [self.servo_ids[name] for name in LEKIWI_BASE_JOINTS]

            arm_group_sync_read = GroupSyncRead(
                port_handler,
                packet_handler,
                STS3215Addr.PRESENT_POSITION,
                STS3215Len.PRESENT_POSITION,
            )
            for servo_id in arm_servo_ids:
                if not arm_group_sync_read.addParam(servo_id):
                    msg = f"Failed to add arm servo {servo_id} to sync read group"
                    raise ConnectionError(msg)

            arm_group_sync_write = GroupSyncWrite(
                port_handler,
                packet_handler,
                STS3215Addr.GOAL_POSITION,
                STS3215Len.GOAL_POSITION,
            )

            base_group_sync_read = GroupSyncRead(
                port_handler,
                packet_handler,
                STS3215Addr.PRESENT_VELOCITY,
                STS3215Len.PRESENT_VELOCITY,
            )
            for servo_id in base_servo_ids:
                if not base_group_sync_read.addParam(servo_id):
                    msg = f"Failed to add base servo {servo_id} to sync read group"
                    raise ConnectionError(msg)

            base_group_sync_write = GroupSyncWrite(
                port_handler,
                packet_handler,
                STS3215Addr.GOAL_VELOCITY,
                STS3215Len.GOAL_VELOCITY,
            )

            self._connection = _LeKiwiConnection(
                port_handler=port_handler,
                packet_handler=packet_handler,
                arm_group_sync_read=arm_group_sync_read,
                arm_group_sync_write=arm_group_sync_write,
                base_group_sync_read=base_group_sync_read,
                base_group_sync_write=base_group_sync_write,
            )

            self._ping_servos()
            self._configure_servos()
            self._set_torque(enabled=self.role == "follower")
        except Exception:
            with contextlib.suppress(Exception):
                port_handler.closePort()
            self._connection = None
            raise

        logger.info(f"LeKiwi connected on {self.port} (role={self.role})")

    def disconnect(self) -> None:
        """Disconnect from the robot, leaving it in a safe state."""
        conn = self._connection
        if conn is None:
            return

        try:
            self._stop_base()
            if self._torque_on_disconnect:
                self._hold_position()
        except Exception:  # noqa: BLE001
            logger.exception(
                "Failed to stop base/hold position while disconnecting LeKiwi; proceeding to close port.",
            )
        finally:
            self._connection = None
            try:
                conn.port_handler.closePort()
            except Exception:  # noqa: BLE001
                logger.exception(
                    "Error while closing LeKiwi serial port; continuing cleanup.",
                )

        logger.info(f"LeKiwi disconnected from {self.port}")

    def get_observation(self) -> RobotObservation:
        """Read current joint positions and base velocities from all motors.

        Returns:
            LeKiwiObservation: Observation with joint positions (9,) and sensor data.
        """
        raw_arm_positions = self._read_arm_positions()

        sensor_data: dict[str, np.ndarray] = {}

        if self._calibration is not None:
            arm_state = self._ticks_to_unit(raw_arm_positions)
        else:
            if not self._warned_uncalibrated:
                logger.warning(
                    "LeKiwi running in explicit uncalibrated mode. Joint "
                    "positions/actions are raw servo ticks (0-4095). "
                    "Do not use uncalibrated mode for policy inference/deployment.",
                )
                self._warned_uncalibrated = True
            arm_state = raw_arm_positions.astype(np.float32)

        raw_wheel_velocities = self._read_base_velocities()
        wheel_degps = raw_wheel_velocities.astype(np.float32) / STEPS_PER_DEG

        sensor_data["wheel_velocities_degps"] = wheel_degps
        body_vel = self._wheel_raw_to_body(
            int(raw_wheel_velocities[0]),
            int(raw_wheel_velocities[1]),
            int(raw_wheel_velocities[2]),
        )
        sensor_data["x_vel"] = np.array([body_vel["x.vel"]], dtype=np.float32)
        sensor_data["y_vel"] = np.array([body_vel["y.vel"]], dtype=np.float32)
        sensor_data["theta_vel"] = np.array([body_vel["theta.vel"]], dtype=np.float32)

        joint_positions = np.empty(self.NUM_JOINTS, dtype=np.float32)
        joint_positions[:6] = arm_state
        joint_positions[6:] = wheel_degps

        return LeKiwiObservation(
            joint_positions=joint_positions,
            timestamp=time.monotonic(),
            sensor_data=sensor_data,
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Send joint position commands to arm and velocity commands to base.

        Args:
            action: Array of shape ``(9,)`` (6 arm positions + 3 base velocities).
            goal_time: Time to reach the goal in seconds (unused in current implementation).

        Raises:
            RuntimeError: If called in leader role.
            ValueError: If action has incorrect shape.
        """
        if self.role == "leader":
            msg = "Cannot send actions to a leader arm. Leader arms are read-only for teleoperation."
            raise RuntimeError(msg)

        expected_shape = (self.NUM_JOINTS,)
        if action.shape != expected_shape:
            msg = f"Expected action shape {expected_shape}, got {action.shape}"
            raise ValueError(msg)

        arm_action = action[:6]
        body_action = action[6:]

        if self._calibration is not None:
            arm_ticks = self._unit_to_ticks(arm_action)
        else:
            arm_ticks = np.round(arm_action).astype(np.int32)
        self._write_arm_positions(arm_ticks)

        wheel_raw = self._body_to_wheel_raw(
            float(body_action[0]),
            float(body_action[1]),
            float(body_action[2]),
        )
        self._write_base_velocities(
            wheel_raw["base_left_wheel"],
            wheel_raw["base_back_wheel"],
            wheel_raw["base_right_wheel"],
        )

    def is_connected(self) -> bool:
        """Check if the robot serial connection is active.

        Returns:
            True if connected, False otherwise.
        """
        return self._connection is not None

    def set_torque(self, *, enabled: bool) -> None:
        """Enable or disable torque on all servos."""
        self._set_torque(enabled=enabled)

    def _stop_base(self) -> None:
        conn = self._connection
        if conn is None:
            return
        self._write_base_velocities(0, 0, 0)

    def _ping_servos(self) -> None:
        conn = self._require_connection()
        for name, servo_id in self.servo_ids.items():
            _, comm_result, error = conn.packet_handler.ping(conn.port_handler, servo_id)
            if comm_result != 0:
                msg = f"Servo '{name}' (ID {servo_id}) did not respond on {self.port}. Comm result: {comm_result}"
                raise ConnectionError(msg)
            if error != 0:
                logger.warning(f"Servo '{name}' (ID {servo_id}) returned error: {error}")

    def _set_torque(self, *, enabled: bool) -> None:
        conn = self._require_connection()
        value = 1 if enabled else 0
        for name, servo_id in self.servo_ids.items():
            comm_result, error = conn.packet_handler.write1ByteTxRx(
                conn.port_handler,
                servo_id,
                STS3215Addr.TORQUE_ENABLE,
                value,
            )
            if comm_result != 0:
                logger.warning(f"Failed to set torque on servo '{name}' (ID {servo_id}): comm={comm_result}")
            if error != 0:
                logger.warning(f"Torque write error on servo '{name}' (ID {servo_id}): err={error}")

    def _hold_position(self) -> None:
        raw = self._read_arm_positions()
        self._write_arm_positions(raw.astype(np.int32))
        self._write_base_velocities(0, 0, 0)
        self._set_torque(enabled=True)

    def _configure_servos(self) -> None:
        conn = self._require_connection()
        self._set_torque(enabled=False)

        for servo_id in [self.servo_ids[name] for name in LEKIWI_ARM_JOINTS]:
            self._write_register(conn, servo_id, STS3215Addr.RETURN_DELAY_TIME, 0)
            self._write_register(conn, servo_id, STS3215Addr.MAXIMUM_ACCELERATION, 254)
            self._write_register(conn, servo_id, STS3215Addr.ACCELERATION, 254)
            self._write_register(conn, servo_id, STS3215Addr.OPERATING_MODE, POSITION_MODE)
            self._write_register(conn, servo_id, STS3215Addr.P_COEFFICIENT, ARM_P_COEFFICIENT)
            self._write_register(conn, servo_id, STS3215Addr.I_COEFFICIENT, ARM_I_COEFFICIENT)
            self._write_register(conn, servo_id, STS3215Addr.D_COEFFICIENT, ARM_D_COEFFICIENT)

            if self.servo_ids_for_name(servo_id) == "arm_gripper":
                self._write_register(conn, servo_id, STS3215Addr.MAX_TORQUE_LIMIT, 500)
                self._write_register(conn, servo_id, STS3215Addr.PROTECTION_CURRENT, 250)
                self._write_register(conn, servo_id, STS3215Addr.OVERLOAD_TORQUE, 25)

        for servo_id in [self.servo_ids[name] for name in LEKIWI_BASE_JOINTS]:
            self._write_register(conn, servo_id, STS3215Addr.RETURN_DELAY_TIME, 0)
            self._write_register(conn, servo_id, STS3215Addr.MAXIMUM_ACCELERATION, 254)
            self._write_register(conn, servo_id, STS3215Addr.OPERATING_MODE, VELOCITY_MODE)

    def servo_ids_for_name(self, servo_id: int) -> str | None:
        """Look up the joint name by servo ID.

        Args:
            servo_id: The servo/motor ID to look up.

        Returns:
            The joint name if found, or ``None``.
        """
        for name, sid in self.servo_ids.items():
            if sid == servo_id:
                return name
        return None

    @staticmethod
    def _write_register(
        conn: _LeKiwiConnection,
        servo_id: int,
        address: STS3215Addr,
        value: int,
    ) -> None:
        byte_width = STS3215Len[address.name]
        if byte_width == 1:
            comm_result, error = conn.packet_handler.write1ByteTxRx(
                conn.port_handler,
                servo_id,
                address,
                value,
            )
        elif byte_width == 2:
            comm_result, error = conn.packet_handler.write2ByteTxRx(
                conn.port_handler,
                servo_id,
                address,
                value,
            )
        else:
            msg = f"Unsupported byte_width={byte_width} for register write"
            raise ValueError(msg)

        if comm_result != 0:
            logger.warning(f"Register write failed: addr={address} servo={servo_id} comm={comm_result}")
        if error != 0:
            logger.warning(f"Register write error: addr={address} servo={servo_id} err={error}")

    def _read_arm_positions(self) -> np.ndarray:
        conn = self._require_connection()
        comm_result = conn.arm_group_sync_read.txRxPacket()
        if comm_result != 0:
            msg = f"Arm sync read failed with comm result {comm_result}"
            raise ConnectionError(msg)

        positions = np.empty(6, dtype=np.int32)
        for i, name in enumerate(LEKIWI_ARM_JOINTS):
            servo_id = self.servo_ids[name]
            if not conn.arm_group_sync_read.isAvailable(
                servo_id,
                STS3215Addr.PRESENT_POSITION,
                STS3215Len.PRESENT_POSITION,
            ):
                msg = f"Arm servo '{name}' (ID {servo_id}) data not available in sync read"
                raise ConnectionError(msg)
            positions[i] = conn.arm_group_sync_read.getData(
                servo_id,
                STS3215Addr.PRESENT_POSITION,
                STS3215Len.PRESENT_POSITION,
            )
        return positions

    def _write_arm_positions(self, ticks: np.ndarray) -> None:
        conn = self._require_connection()
        ticks = np.clip(ticks, 0, TICKS_PER_REVOLUTION - 1)
        conn.arm_group_sync_write.clearParam()

        for i, name in enumerate(LEKIWI_ARM_JOINTS):
            servo_id = self.servo_ids[name]
            position = int(ticks[i])
            param = [position & 0xFF, (position >> 8) & 0xFF]
            if not conn.arm_group_sync_write.addParam(servo_id, param):
                msg = f"Failed to add arm servo '{name}' (ID {servo_id}) to sync write"
                raise ConnectionError(msg)

        comm_result = conn.arm_group_sync_write.txPacket()
        if comm_result != 0:
            msg = f"Arm sync write failed with comm result {comm_result}"
            raise ConnectionError(msg)

    def _read_base_velocities(self) -> np.ndarray:
        conn = self._require_connection()
        comm_result = conn.base_group_sync_read.txRxPacket()
        if comm_result != 0:
            msg = f"Base sync read failed with comm result {comm_result}"
            raise ConnectionError(msg)

        velocities = np.empty(3, dtype=np.int32)
        for i, name in enumerate(LEKIWI_BASE_JOINTS):
            servo_id = self.servo_ids[name]
            if not conn.base_group_sync_read.isAvailable(
                servo_id,
                STS3215Addr.PRESENT_VELOCITY,
                STS3215Len.PRESENT_VELOCITY,
            ):
                msg = f"Base servo '{name}' (ID {servo_id}) data not available in sync read"
                raise ConnectionError(msg)
            velocities[i] = conn.base_group_sync_read.getData(
                servo_id,
                STS3215Addr.PRESENT_VELOCITY,
                STS3215Len.PRESENT_VELOCITY,
            )
        return velocities

    def _write_base_velocities(self, left_raw: int, back_raw: int, right_raw: int) -> None:
        conn = self._require_connection()
        conn.base_group_sync_write.clearParam()

        raw_values = {
            "base_left_wheel": left_raw,
            "base_back_wheel": back_raw,
            "base_right_wheel": right_raw,
        }
        for name in LEKIWI_BASE_JOINTS:
            servo_id = self.servo_ids[name]
            value = int(np.clip(raw_values[name], -32768, 32767))
            param = [value & 0xFF, (value >> 8) & 0xFF]
            if not conn.base_group_sync_write.addParam(servo_id, param):
                msg = f"Failed to add base servo '{name}' (ID {servo_id}) to sync write"
                raise ConnectionError(msg)

        comm_result = conn.base_group_sync_write.txPacket()
        if comm_result != 0:
            msg = f"Base sync write failed with comm result {comm_result}"
            raise ConnectionError(msg)

    def _ticks_to_normalized(self, ticks: np.ndarray) -> np.ndarray:
        calibration = self._require_calibration()
        result = np.empty(6, dtype=np.float32)
        for i, name in enumerate(LEKIWI_ARM_JOINTS):
            cal = calibration.joints[name]
            rng = cal.range_max - cal.range_min
            if rng <= 0:
                result[i] = 0.0
                continue
            tick_value = int(np.clip(ticks[i], cal.range_min, cal.range_max))
            if name == "arm_gripper":
                norm = ((tick_value - cal.range_min) / rng) * 100.0
            else:
                norm = ((tick_value - cal.range_min) / rng) * 200.0 - 100.0
            if name == "arm_gripper":
                result[i] = float(np.clip(norm, 0.0, 100.0))
            else:
                result[i] = float(np.clip(norm, -100.0, 100.0))
        return result

    def _ticks_to_unit(self, ticks: np.ndarray, *, unit: LeKiwiUnit | None = None) -> np.ndarray:
        resolved = self._resolve_unit(unit)
        if resolved == "normalized":
            return self._ticks_to_normalized(ticks)
        return ticks.astype(np.float32)

    def _normalized_to_ticks(self, values: np.ndarray) -> np.ndarray:
        calibration = self._require_calibration()
        result = np.empty(6, dtype=np.int32)
        for i, name in enumerate(LEKIWI_ARM_JOINTS):
            cal = calibration.joints[name]
            rng = cal.range_max - cal.range_min
            if rng <= 0:
                result[i] = int(np.clip(cal.homing_offset, cal.range_min, cal.range_max))
                continue
            value = float(values[i])
            if name == "arm_gripper":
                clamped = float(np.clip(value, 0.0, 100.0))
                ticks_val = round(cal.range_min + (clamped / 100.0) * rng)
            else:
                clamped = float(np.clip(value, -100.0, 100.0))
                ticks_val = round(cal.range_min + ((clamped + 100.0) / 200.0) * rng)
            result[i] = int(np.clip(ticks_val, cal.range_min, cal.range_max))
        return result

    def _unit_to_ticks(self, values: np.ndarray, *, unit: LeKiwiUnit | None = None) -> np.ndarray:
        resolved = self._resolve_unit(unit)
        if resolved == "normalized":
            return self._normalized_to_ticks(values)
        return np.round(values).astype(np.int32)

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        speed_in_steps = degps * STEPS_PER_DEG
        speed_int = round(speed_in_steps)
        if speed_int > MAX_RAW_SPEED_POSITIVE:
            speed_int = MAX_RAW_SPEED_POSITIVE
        elif speed_int < MAX_RAW_SPEED_NEGATIVE:
            speed_int = MAX_RAW_SPEED_NEGATIVE
        return speed_int

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        return raw_speed / STEPS_PER_DEG

    @staticmethod
    def _body_to_wheel_raw(
        x: float,
        y: float,
        theta: float,
        wheel_radius: float = WHEEL_RADIUS,
        base_radius: float = BASE_RADIUS,
        max_raw: int = MAX_RAW_WHEEL,
    ) -> dict[str, int]:
        theta_rad = theta * (np.pi / 180.0)
        velocity_vector = np.array([x, y, theta_rad])

        angles = np.radians(np.array(WHEEL_ANGLES_DEG) - WHEEL_OFFSET_DEG)
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

        wheel_linear_speeds = m.dot(velocity_vector)
        wheel_angular_speeds = wheel_linear_speeds / wheel_radius

        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

        raw_floats = [abs(degps) * STEPS_PER_DEG for degps in wheel_degps]
        max_raw_computed = max(raw_floats) if raw_floats else 0.0
        if max_raw_computed > max_raw:
            scale = max_raw / max_raw_computed
            wheel_degps *= scale

        wheel_raw = [LeKiwi._degps_to_raw(deg) for deg in wheel_degps]

        return {
            "base_left_wheel": wheel_raw[0],
            "base_back_wheel": wheel_raw[1],
            "base_right_wheel": wheel_raw[2],
        }

    @staticmethod
    def _wheel_raw_to_body(
        left_wheel_speed: int,
        back_wheel_speed: int,
        right_wheel_speed: int,
        wheel_radius: float = WHEEL_RADIUS,
        base_radius: float = BASE_RADIUS,
    ) -> dict[str, float]:
        wheel_degps = np.array([
            LeKiwi._raw_to_degps(left_wheel_speed),
            LeKiwi._raw_to_degps(back_wheel_speed),
            LeKiwi._raw_to_degps(right_wheel_speed),
        ])

        wheel_radps = wheel_degps * (np.pi / 180.0)
        wheel_linear_speeds = wheel_radps * wheel_radius

        angles = np.radians(np.array(WHEEL_ANGLES_DEG) - WHEEL_OFFSET_DEG)
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])

        m_inv = np.linalg.inv(m)
        velocity_vector = m_inv.dot(wheel_linear_speeds)
        x, y, theta_rad = velocity_vector
        theta = theta_rad * (180.0 / np.pi)
        return {
            "x.vel": float(x),
            "y.vel": float(y),
            "theta.vel": float(theta),
        }
