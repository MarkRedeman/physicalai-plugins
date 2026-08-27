"""Keyboard teleoperation: WASD/QE base control with the arm held in place.

Reads single-key input from an interactive TTY and converts it into base
velocities ``[vx, vy, vtheta]`` (``m/s``, ``m/s``, ``deg/s``), while the arm
joints are held at their currently observed position. Intended to be driven
by :class:`physicalai.runtime.RobotRuntime` via a runtime config
(``physicalai run --config``); the returned action always has
``num_arm_joints + num_base_joints`` entries.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import tty
from typing import TYPE_CHECKING

import numpy as np
from physicalai.config import export_config

if TYPE_CHECKING:
    from collections.abc import Mapping

    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation
    from physicalai.runtime._callback_bus import _CallbackBus

_SELECT_TIMEOUT_S = 0.0
_MAX_BASE_AXES = 3


@export_config(class_path="physicalai_common_extras.KeyboardTeleop")
class KeyboardTeleop:
    """Map keyboard input to base velocities; hold the arm at its current pose.

    Args:
        vx: Forward/backward base speed in ``m/s``.
        vy: Strafe base speed in ``m/s``.
        vtheta: Rotational base speed in ``deg/s``.
        num_arm_joints: Number of leading joints echoed from the observation.
        num_base_joints: Number of trailing joints driven from the keyboard.

    Keys (single characters, case-insensitive):

    - ``w`` / ``s`` — forward / backward
    - ``a`` / ``d`` — rotate left / right
    - ``q`` / ``e`` — strafe left / right
    - ``space`` — stop (zero the base)

    Requires an interactive TTY on stdin; raises ``RuntimeError`` otherwise.
    The terminal is restored to its previous settings on disconnect.
    """

    _FORWARD = "w"
    _BACKWARD = "s"
    _ROTATE_LEFT = "a"
    _ROTATE_RIGHT = "d"
    _STRAFE_LEFT = "q"
    _STRAFE_RIGHT = "e"
    _STOP = " "

    def __init__(
        self,
        *,
        vx: float = 0.15,
        vy: float = 0.10,
        vtheta: float = 0.5,
        num_arm_joints: int = 6,
        num_base_joints: int = 3,
    ) -> None:
        """Initialize the keyboard teleop source.

        Raises:
            ValueError: If the joint counts or speeds are non-positive.
        """
        if vx <= 0 or vy <= 0 or vtheta <= 0:
            msg = "vx, vy and vtheta must be positive"
            raise ValueError(msg)
        if num_arm_joints < 0 or num_base_joints <= 0:
            msg = "num_arm_joints must be >= 0 and num_base_joints must be > 0"
            raise ValueError(msg)

        self._vx = vx
        self._vy = vy
        self._vtheta = vtheta
        self._num_arm_joints = num_arm_joints
        self._num_base_joints = num_base_joints
        self._num_joints = num_arm_joints + num_base_joints
        self._commands = np.zeros(_MAX_BASE_AXES, dtype=np.float32)
        self._fd: int | None = None
        self._old_settings: object | None = None

    def connect(
        self,
        *,
        bus: _CallbackBus,  # noqa: ARG002
        session_id: str,  # noqa: ARG002
    ) -> None:
        """Put stdin into cbreak mode so keys arrive without pressing Enter.

        Args:
            bus: Unused; part of the ``ActionSource`` protocol.
            session_id: Unused; part of the ``ActionSource`` protocol.

        Raises:
            RuntimeError: If stdin is not an interactive TTY.
        """
        if not sys.stdin.isatty():
            msg = "KeyboardTeleop requires an interactive TTY on stdin; run from a terminal."
            raise RuntimeError(msg)
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

    def update(
        self,
        robot_state: RobotObservation,
        camera_frames: Mapping[str, Frame],  # noqa: ARG002
        step: int,  # noqa: ARG002
    ) -> np.ndarray:
        """Return the next action: arm held, base from the pressed keys.

        Args:
            robot_state: The follower observation; its leading arm joints are
                echoed back unchanged so the arm stays put.
            camera_frames: Unused; part of the ``ActionSource`` protocol.
            step: Unused; part of the ``ActionSource`` protocol.

        Returns:
            An action of shape ``(num_arm_joints + num_base_joints,)``.
        """
        self._drain_keys()
        action = np.zeros(self._num_joints, dtype=np.float32)
        action[: self._num_arm_joints] = np.asarray(
            robot_state.joint_positions[: self._num_arm_joints],
            dtype=np.float32,
        )
        action[self._num_arm_joints : self._num_joints] = self._commands
        return action

    def disconnect(self) -> None:
        """Restore the terminal and release the file descriptor."""
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSANOW, self._old_settings)
        self._fd = None
        self._old_settings = None

    def _drain_keys(self) -> None:
        """Apply every pending keypress to the base velocity command."""
        if self._fd is None:
            return
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], _SELECT_TIMEOUT_S)
            if not readable:
                return
            try:
                data = os.read(self._fd, 1)
            except OSError:
                return
            if not data:
                return
            self._apply_key(chr(data[0]).lower())

    def _apply_key(self, key: str) -> None:
        """Map a single keypress to base velocities.

        Args:
            key: The lowercased character read from stdin.
        """
        if key == self._FORWARD:
            self._commands[0] = self._vx
        elif key == self._BACKWARD:
            self._commands[0] = -self._vx
        elif key == self._STRAFE_LEFT:
            self._commands[1] = self._vy
        elif key == self._STRAFE_RIGHT:
            self._commands[1] = -self._vy
        elif key == self._ROTATE_LEFT:
            self._commands[2] = self._vtheta
        elif key == self._ROTATE_RIGHT:
            self._commands[2] = -self._vtheta
        elif key == self._STOP:
            self._commands[:] = 0.0
