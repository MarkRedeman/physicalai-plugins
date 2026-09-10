"""LeKiwi leader wrapper: SO-101 arm + keyboard base command channels."""

from __future__ import annotations

import contextlib
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, ClassVar, Literal, Protocol

import numpy as np
from loguru import logger
from physicalai.config import export_config
from physicalai.robot import Robot
from physicalai.robot.so101 import SO101, SO101Calibration

from physicalai_lekiwi_plugin.constants import LEKIWI_ARM_JOINTS, LEKIWI_JOINT_ORDER
from physicalai_lekiwi_plugin.teleop.gamepad import GamepadTeleop
from physicalai_lekiwi_plugin.teleop.keyboard import KeyboardTeleop

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation


LeKiwiLeaderUnit = Literal["ticks", "normalized"]
_SO101_LEADER_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
_LEKIWI_TO_SO101_JOINT = {
    "arm_shoulder_pan": "shoulder_pan",
    "arm_shoulder_lift": "shoulder_lift",
    "arm_elbow_flex": "elbow_flex",
    "arm_wrist_flex": "wrist_flex",
    "arm_wrist_roll": "wrist_roll",
    "arm_gripper": "gripper",
}


@dataclass
class _BaseUpdateObservation:
    joint_positions: np.ndarray


class _BaseSource(Protocol):
    def connect(self, *, bus: object, session_id: str) -> None: ...

    def update(self, robot_state: object, camera_frames: object, step: int) -> np.ndarray: ...

    def disconnect(self) -> None: ...


@dataclass
class LeKiwiLeaderObservation:
    """Observation from a LeKiwiLeader robot.

    Attributes:
        joint_positions: Array of shape ``(9,)`` with arm joints followed by
            keyboard-driven base commands ``[vx, vy, vtheta]``.
        timestamp: ``time.monotonic()`` at the moment of capture.
        sensor_data: Optional dict containing ``base_command``.
    """

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        """State vector: joint positions (9,)."""
        return self.joint_positions


@export_config(class_path="physicalai_lekiwi_plugin.LeKiwiLeader")
class LeKiwiLeader(Robot):
    """SO-101 leader arm with keyboard-driven LeKiwi base channels.

    This wraps a leader-role :class:`physicalai.robot.SO101` and appends
    keyboard base commands to observations so it can drive a LeKiwi follower
    through :class:`physicalai.runtime.TeleopSource`.
    """

    JOINT_ORDER: ClassVar[list[str]] = list(LEKIWI_JOINT_ORDER)
    NUM_JOINTS: ClassVar[int] = 9

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 1_000_000,
        calibration: SO101Calibration | dict[str, dict[str, int]] | str | Path | None = None,
        unit: LeKiwiLeaderUnit | None = None,
        *,
        vx: float = 0.15,
        vy: float = 0.10,
        vtheta: float = 30.0,
        debug: bool = False,
        use_gamepad: bool = False,
        gamepad_deadzone: float = 0.10,
        gamepad_joystick_index: int = 0,
        disable_torque_on_disconnect: bool = True,
    ) -> None:
        """Initialize the LeKiwi leader wrapper.

        Args:
            port: Leader arm serial port.
            baudrate: Leader arm serial baudrate.
            calibration: SO101 calibration object/path/dict or LeKiwi-style
                calibration dict/path (only arm joints are used).
            unit: ``"normalized"`` in calibrated mode, ``"ticks"`` in
                explicit uncalibrated mode.
            vx: Forward/backward keyboard speed in ``m/s``.
            vy: Lateral keyboard speed in ``m/s``.
            vtheta: Rotational keyboard speed in ``deg/s``.
            debug: Print keyboard debug diagnostics to stderr.
            use_gamepad: If ``True``, read base commands from an Xbox-style
                gamepad via pygame. If ``False``, use keyboard control.
            gamepad_deadzone: Analog deadzone for gamepad axis filtering.
            gamepad_joystick_index: Pygame joystick index to open.
            disable_torque_on_disconnect: Kept for payload compatibility.
                Must be ``True`` for leader-role operation.

        Raises:
            ValueError: If unit/role constraints are invalid.
        """
        if not disable_torque_on_disconnect:
            msg = "LeKiwiLeader only supports disable_torque_on_disconnect=True in leader mode."
            raise ValueError(msg)

        self._port = port
        self._baudrate = baudrate
        self._unit: LeKiwiLeaderUnit
        self._disable_torque_on_disconnect = disable_torque_on_disconnect

        arm_calibration = self._resolve_so101_calibration(calibration)
        if arm_calibration is None:
            if unit is None:
                self._unit = "ticks"
            else:
                self._unit = unit
            if self._unit != "ticks":
                msg = "Uncalibrated LeKiwiLeader only supports unit='ticks'."
                raise ValueError(msg)
            self._leader = SO101.uncalibrated(
                port=port,
                baudrate=baudrate,
                role="leader",
                unit="ticks",
            )
        else:
            self._unit = "normalized" if unit is None else unit
            if self._unit != "normalized":
                msg = "Calibrated LeKiwiLeader only supports unit='normalized'."
                raise ValueError(msg)
            self._leader = SO101(
                port=port,
                baudrate=baudrate,
                role="leader",
                calibration=arm_calibration,
                unit=self._unit,
            )

        self._base_source: _BaseSource = (
            GamepadTeleop(
                vx=vx,
                vy=vy,
                vtheta=vtheta,
                deadzone=gamepad_deadzone,
                joystick_index=gamepad_joystick_index,
                num_arm_joints=0,
                num_base_joints=3,
            )
            if use_gamepad
            else KeyboardTeleop(
                vx=vx,
                vy=vy,
                vtheta=vtheta,
                debug=debug,
                num_arm_joints=0,
                num_base_joints=3,
            )
        )
        self._use_gamepad = use_gamepad
        self._base_observation = _BaseUpdateObservation(joint_positions=np.zeros(0, dtype=np.float32))
        self._keyboard_ready = False
        self._base_source_connect_attempted = False
        self._keyboard_unavailable_logged = False
        self._step = 0

    @classmethod
    def uncalibrated(
        cls,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 1_000_000,
        unit: LeKiwiLeaderUnit = "ticks",
        *,
        vx: float = 0.15,
        vy: float = 0.10,
        vtheta: float = 30.0,
        debug: bool = False,
        use_gamepad: bool = False,
        gamepad_deadzone: float = 0.10,
        gamepad_joystick_index: int = 0,
        disable_torque_on_disconnect: bool = True,
    ) -> LeKiwiLeader:
        """Create an explicit uncalibrated leader in raw-ticks arm mode.

        Returns:
            LeKiwiLeader: Uncalibrated SO101-based leader wrapper.
        """
        return cls(
            port=port,
            baudrate=baudrate,
            calibration=None,
            unit=unit,
            vx=vx,
            vy=vy,
            vtheta=vtheta,
            debug=debug,
            use_gamepad=use_gamepad,
            gamepad_deadzone=gamepad_deadzone,
            gamepad_joystick_index=gamepad_joystick_index,
            disable_torque_on_disconnect=disable_torque_on_disconnect,
        )

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names (SO-101 arm followed by 3 base command channels)."""
        return self.JOINT_ORDER

    @property
    def role(self) -> Literal["leader"]:
        """Role is always ``"leader"`` for this wrapper."""
        return "leader"

    @property
    def device_ids(self) -> tuple[str, ...]:
        """Device identity delegated from the wrapped SO101 leader."""
        return self._leader.device_ids

    @property
    def port(self) -> str:
        """Configured serial port for the wrapped SO101 leader."""
        return self._port

    @property
    def baudrate(self) -> int:
        """Configured serial baudrate for the wrapped SO101 leader."""
        return self._baudrate

    def connect(self) -> None:
        """Connect the wrapped SO101 leader.

        Base input setup is deferred until observation reads so robot transport
        can connect in non-interactive environments (for example Studio owner workers).
        """
        if self.is_connected():
            return

        self._leader.connect()
        self._keyboard_ready = False
        self._base_source_connect_attempted = False

    def disconnect(self) -> None:
        """Disconnect keyboard capture and the wrapped SO101 leader."""
        with contextlib.suppress(Exception):
            if self._keyboard_ready:
                self._base_source.disconnect()
        self._keyboard_ready = False
        self._leader.disconnect()

    def is_connected(self) -> bool:
        """Return ``True`` when the wrapped SO101 leader is connected."""
        return self._leader.is_connected()

    def get_observation(self) -> RobotObservation:
        """Read SO101 leader arm state and append current base command channels.

        Returns:
            LeKiwiLeaderObservation: Arm joint state plus ``[vx, vy, vtheta]``.
        """
        arm_obs = self._leader.get_observation()
        base_command = np.zeros(3, dtype=np.float32)

        self._ensure_base_source_connected()

        if self._keyboard_ready:
            try:
                base_action = self._base_source.update(self._base_observation, {}, self._step)
                base_command[:] = np.asarray(base_action[:3], dtype=np.float32)
            except RuntimeError as exc:
                self._keyboard_ready = False
                logger.warning(
                    f"LeKiwiLeader keyboard update failed; base commands reset to zero until reconnect. Reason: {exc}",
                )

        self._step += 1
        joint_positions = np.empty(self.NUM_JOINTS, dtype=np.float32)
        joint_positions[:6] = np.asarray(arm_obs.joint_positions[:6], dtype=np.float32)
        joint_positions[6:] = base_command

        return LeKiwiLeaderObservation(
            joint_positions=joint_positions,
            timestamp=time.monotonic(),
            sensor_data={"base_command": base_command.copy()},
        )

    def _ensure_base_source_connected(self) -> None:
        if self._keyboard_ready or self._base_source_connect_attempted:
            return

        self._base_source_connect_attempted = True
        try:
            self._base_source.connect(bus=object(), session_id="lekiwi-leader")
            self._keyboard_ready = True
        except (OSError, RuntimeError, ValueError) as exc:
            if self._use_gamepad:
                msg = f"LeKiwiLeader gamepad setup failed: {exc}"
                raise RuntimeError(msg) from exc
            if not self._keyboard_unavailable_logged:
                logger.warning(
                    "LeKiwiLeader keyboard input unavailable; base commands will remain zero until stdin is a TTY. "
                    f"Reason: {exc}",
                )
                self._keyboard_unavailable_logged = True

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Raise because leader devices are read-only in teleoperation flows.

        Raises:
            RuntimeError: Always, because leader devices are read-only.
        """
        _ = self, action, goal_time
        msg = "Cannot send actions to a leader arm. Leader arms are read-only for teleoperation."
        raise RuntimeError(msg)

    @staticmethod
    def _resolve_so101_calibration(
        calibration: SO101Calibration | dict[str, dict[str, int]] | str | Path | None,
    ) -> SO101Calibration | None:
        if calibration is None:
            return None
        if isinstance(calibration, SO101Calibration):
            return calibration
        if isinstance(calibration, dict):
            return LeKiwiLeader._calibration_from_dict(calibration)
        if isinstance(calibration, (str, Path)):
            return LeKiwiLeader._calibration_from_path(calibration)
        msg = f"Unsupported calibration type: {type(calibration).__name__}"
        raise TypeError(msg)

    @staticmethod
    def _calibration_from_path(path: str | Path) -> SO101Calibration:
        with Path(path).expanduser().open("r", encoding="utf-8") as f:
            payload = json.load(f)
        if not isinstance(payload, dict):
            msg = "Calibration JSON must be a mapping of joint names to calibration entries."
            raise TypeError(msg)
        return LeKiwiLeader._calibration_from_dict(payload)

    @staticmethod
    def _calibration_from_dict(calibration: dict[str, Any]) -> SO101Calibration:
        if set(_LEKIWI_TO_SO101_JOINT).issubset(calibration):
            so101_dict = {
                _LEKIWI_TO_SO101_JOINT[lekiwi_name]: calibration[lekiwi_name] for lekiwi_name in LEKIWI_ARM_JOINTS
            }
            return SO101Calibration.from_dict(so101_dict)
        if set(_SO101_LEADER_JOINTS).issubset(calibration):
            return SO101Calibration.from_dict(calibration)
        msg = (
            "Calibration dict must contain either SO101 arm keys "
            f"{sorted(_SO101_LEADER_JOINTS)} or LeKiwi arm keys {sorted(_LEKIWI_TO_SO101_JOINT)}."
        )
        raise ValueError(msg)
