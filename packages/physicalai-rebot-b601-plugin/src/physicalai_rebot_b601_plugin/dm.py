"""Damiao motor driver for the reBot B601 robot arm.

Uses the ``motorbridge`` SDK to communicate with Damiao DM-series motors over
serial (Damiao CAN adapter) or native SocketCAN. The gripper runs in
FORCE_POS mode; all other joints run in POS_VEL mode.
"""

from __future__ import annotations

import contextlib
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar, Literal

import numpy as np
from loguru import logger

from physicalai_rebot_b601_plugin.constants import (
    REBOT_B601_DM_JOINT_DIRECTIONS,
    REBOT_B601_DM_JOINT_LIMITS_DEG,
    REBOT_B601_DM_JOINT_ORDER,
    REBOT_B601_DM_MOTOR_IDS,
    REBOT_B601_DM_MOTOR_MODELS,
    REBOT_B601_DM_POS_VEL_DEG_S,
    VALID_CAN_ADAPTERS,
    VALID_ROLES,
)

if TYPE_CHECKING:
    from motorbridge import Motor, MotorState
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation

from motorbridge import Controller, Mode

ReBotCANAdapter = Literal["damiao", "socketcan"]
ReBotRole = Literal["follower"]


@dataclass
class ReBotB601DMObservation:
    """Observation data for the Damiao reBot B601 arm.

    Attributes:
        joint_positions: Measured joint positions in degrees.
        timestamp: Monotonic time of the observation.
        sensor_data: Optional dict of velocity, torque, temperature, and status arrays.
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


class ReBotB601DM:
    """Damiao motor driver for the reBot B601 robot arm.

    Controls 7 DM-series motors (6-DOF + gripper) using POS_VEL mode for
    position joints and FORCE_POS mode for the gripper.
    """

    JOINT_ORDER: ClassVar[list[str]] = list(REBOT_B601_DM_JOINT_ORDER)
    NUM_JOINTS: ClassVar[int] = len(JOINT_ORDER)

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        *,
        can_adapter: ReBotCANAdapter = "damiao",
        dm_serial_baud: int = 921600,
        role: ReBotRole = "follower",
        disable_torque_on_disconnect: bool = True,
        force_pos_torque_ratio: float = 0.1,
    ) -> None:
        """Initialize the Damiao motor driver.

        Args:
            port: Serial port (``/dev/ttyACM0``) or SocketCAN channel (``can0``).
            can_adapter: ``"damiao"`` for Damiao USB-CAN, ``"socketcan"`` for native SocketCAN.
            dm_serial_baud: Baud rate for Damiao serial adapter.
            role: Currently only ``"follower"`` is supported.
            disable_torque_on_disconnect: Whether to disable all motors on disconnect.
            force_pos_torque_ratio: FORCE_POS torque ratio in ``[0, 1]`` for gripper.

        Raises:
            ValueError: If any parameter has an invalid value.
        """
        if role not in VALID_ROLES:
            msg = f"Invalid role {role!r}. ReBotB601DM currently supports only {sorted(VALID_ROLES)}."
            raise ValueError(msg)
        if can_adapter not in VALID_CAN_ADAPTERS:
            msg = f"Invalid can_adapter {can_adapter!r}. Must be one of {sorted(VALID_CAN_ADAPTERS)}."
            raise ValueError(msg)
        if dm_serial_baud <= 0:
            msg = f"dm_serial_baud must be a positive integer, got {dm_serial_baud!r}"
            raise ValueError(msg)
        if not (0.0 <= force_pos_torque_ratio <= 1.0):
            msg = f"force_pos_torque_ratio must be in [0, 1], got {force_pos_torque_ratio!r}"
            raise ValueError(msg)

        self._port = port
        self._can_adapter = can_adapter
        self._dm_serial_baud = dm_serial_baud
        self._role = role
        self._disable_torque_on_disconnect = disable_torque_on_disconnect
        self._force_pos_torque_ratio = force_pos_torque_ratio
        self._controller: Controller | None = None
        self._motors: dict[str, Motor] = {}

    @property
    def joint_names(self) -> list[str]:
        """Ordered list of joint names matching the expected action/observation layout."""
        return self.JOINT_ORDER

    @property
    def port(self) -> str:
        """Serial port or CAN channel the driver is configured for."""
        return self._port

    @property
    def can_adapter(self) -> ReBotCANAdapter:
        """CAN adapter type (``"damiao"`` or ``"socketcan"``)."""
        return self._can_adapter

    @property
    def role(self) -> ReBotRole:
        """Role of this driver instance (``"follower"``)."""
        return self._role

    @property
    def disable_torque_on_disconnect(self) -> bool:
        """Whether torque is automatically disabled on disconnect."""
        return self._disable_torque_on_disconnect

    @disable_torque_on_disconnect.setter
    def disable_torque_on_disconnect(self, value: bool) -> None:
        self._disable_torque_on_disconnect = value

    def _require_controller(self) -> Controller:
        controller = self._controller
        if controller is None:
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)
        return controller

    def connect(self) -> None:
        """Open the controller connection, register motors, and configure control modes."""
        if self.is_connected():
            return

        try:
            controller = self._open_controller()
            self._controller = controller
            self._motors = self._register_motors(controller)
            self._configure_motors()
        except Exception:
            with contextlib.suppress(Exception):
                self._cleanup_connection()
            raise

        logger.info(f"ReBotB601DM connected on {self.port} (adapter={self.can_adapter})")

    def _open_controller(self) -> Controller:
        if self.can_adapter == "damiao":
            return Controller.from_dm_serial(serial_port=self.port, baud=self._dm_serial_baud)
        return Controller(channel=self.port)

    def _register_motors(self, controller: Controller) -> dict[str, Motor]:
        motors: dict[str, Motor] = {}
        for name in self.JOINT_ORDER:
            motor_id, feedback_id = REBOT_B601_DM_MOTOR_IDS[name]
            motors[name] = controller.add_damiao_motor(motor_id, feedback_id, REBOT_B601_DM_MOTOR_MODELS[name])
        return motors

    def disconnect(self) -> None:
        """Disable torque (if configured), clear errors, close motors, and release the controller."""
        if self._controller is None:
            return

        try:
            self._disconnect_motors()
        finally:
            self._cleanup_connection()

        logger.info(f"ReBotB601DM disconnected from {self.port}")

    def _disconnect_motors(self) -> None:
        if self.disable_torque_on_disconnect and self._controller is not None:
            with contextlib.suppress(Exception):
                self._controller.disable_all()
        for name, motor in self._motors.items():
            with contextlib.suppress(Exception):
                motor.clear_error()
            with contextlib.suppress(Exception):
                motor.close()
                logger.debug(f"Closed reBot motor {name}")

    def _cleanup_connection(self) -> None:
        controller = self._controller
        self._controller = None
        self._motors = {}
        if controller is not None:
            with contextlib.suppress(Exception):
                controller.close()

    def is_connected(self) -> bool:
        """Return whether the controller connection is active."""
        return self._controller is not None

    def _configure_motors(self) -> None:
        controller = self._require_controller()
        controller.disable_all()

        for name, motor in self._motors.items():
            target_mode = Mode.FORCE_POS if name == "gripper" else Mode.POS_VEL
            motor.ensure_mode(target_mode)

        controller.enable_all()

    def disable_torque(self) -> None:
        """Disable torque (brake) on all motors."""
        self._require_controller().disable_all()

    def enable_torque(self) -> None:
        """Enable torque on all motors."""
        self._require_controller().enable_all()

    def configure_position_control(self) -> None:
        """Reconfigure all motors to their position-control modes."""
        self._configure_motors()

    def _read_motor_states(self) -> list[MotorState]:
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)
        controller = self._require_controller()
        for motor in self._motors.values():
            motor.request_feedback()
        controller.poll_feedback_once()

        states: list[MotorState] = []
        for name in self.JOINT_ORDER:
            state = self._motors[name].get_state()
            if state is None:
                msg = f"No feedback received for motor '{name}'"
                raise ConnectionError(msg)
            states.append(state)
        return states

    def get_observation(self) -> RobotObservation:
        """Read joint positions, velocities, torques, and temperatures from all motors.

        Returns:
            A ``ReBotB601DMObservation`` with joint positions in degrees and
            sensor data arrays for velocities, torques, temperatures, and status codes.

        Raises:
            ConnectionError: If the robot is not connected.
        """
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)
        states = self._read_motor_states()

        positions = np.empty(self.NUM_JOINTS, dtype=np.float32)
        velocities = np.empty(self.NUM_JOINTS, dtype=np.float32)
        torques = np.empty(self.NUM_JOINTS, dtype=np.float32)
        mos_temperatures = np.empty(self.NUM_JOINTS, dtype=np.float32)
        rotor_temperatures = np.empty(self.NUM_JOINTS, dtype=np.float32)
        status_codes = np.empty(self.NUM_JOINTS, dtype=np.int32)

        for i, state in enumerate(states):
            positions[i] = math.degrees(float(state.pos))
            velocities[i] = math.degrees(float(state.vel))
            torques[i] = float(state.torq)
            mos_temperatures[i] = float(state.t_mos)
            rotor_temperatures[i] = float(state.t_rotor)
            status_codes[i] = int(state.status_code)

        return ReBotB601DMObservation(
            joint_positions=positions,
            timestamp=time.monotonic(),
            sensor_data={
                "velocities": velocities,
                "torques": torques,
                "mos_temperatures": mos_temperatures,
                "rotor_temperatures": rotor_temperatures,
                "status_codes": status_codes,
            },
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Send a target position command to each joint.

        The gripper uses FORCE_POS mode; all other joints use POS_VEL mode
        with a fixed velocity limit.

        Args:
            action: Array of 7 joint position targets in degrees.
            goal_time: Ignored; present for protocol compatibility.

        Raises:
            ConnectionError: If the robot is not connected.
            ValueError: If the action shape does not match ``NUM_JOINTS``.
        """
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)
        self._require_controller()
        expected_shape = (self.NUM_JOINTS,)
        if action.shape != expected_shape:
            msg = f"Expected action shape {expected_shape}, got {action.shape}"
            raise ValueError(msg)

        for i, name in enumerate(self.JOINT_ORDER):
            target_deg = self._map_and_clip_action(name, float(action[i]))
            target_rad = math.radians(target_deg)
            velocity_rad_s = math.radians(REBOT_B601_DM_POS_VEL_DEG_S[i])
            motor = self._motors[name]

            if name == "gripper":
                motor.send_force_pos(target_rad, velocity_rad_s, self._force_pos_torque_ratio)
            else:
                motor.send_pos_vel(target_rad, velocity_rad_s)

    @staticmethod
    def _map_and_clip_action(name: str, action_deg: float) -> float:
        mapped = action_deg * REBOT_B601_DM_JOINT_DIRECTIONS[name]
        min_deg, max_deg = REBOT_B601_DM_JOINT_LIMITS_DEG[name]
        return float(np.clip(mapped, min_deg, max_deg))
