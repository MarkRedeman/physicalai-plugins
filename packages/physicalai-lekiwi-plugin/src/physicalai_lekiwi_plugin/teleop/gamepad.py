"""Gamepad teleoperation: Xbox-style analog base control with the arm held in place."""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Protocol

import numpy as np
from physicalai.config import export_config

if TYPE_CHECKING:
    from collections.abc import Mapping

    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation
    from physicalai.runtime._callback_bus import _CallbackBus

_MAX_BASE_AXES = 3


class _PygameEventModule(Protocol):
    def pump(self) -> None: ...

    def get(self) -> list[object]: ...


class _PygameJoystick(Protocol):
    def init(self) -> None: ...

    def quit(self) -> None: ...

    def get_axis(self, axis: int) -> float: ...


class _PygameJoystickModule(Protocol):
    def init(self) -> None: ...

    def quit(self) -> None: ...

    def get_count(self) -> int: ...

    def Joystick(self, joystick_index: int) -> _PygameJoystick: ...  # noqa: N802


class _PygameModule(Protocol):
    event: _PygameEventModule
    joystick: _PygameJoystickModule

    def init(self) -> None: ...


def _import_pygame() -> _PygameModule:
    return importlib.import_module("pygame")


@export_config(class_path="physicalai_lekiwi_plugin.teleop.GamepadTeleop")
class GamepadTeleop:
    """Map an Xbox-style gamepad to base velocities; hold arm at current pose.

    Axis mapping defaults:

    - left stick Y (axis 1) -> forward/backward (vx)
    - left stick X (axis 0) -> strafe (vy)
    - right stick X (axis 3) -> rotation (vtheta)
    """

    def __init__(
        self,
        *,
        vx: float = 0.15,
        vy: float = 0.10,
        vtheta: float = 30.0,
        deadzone: float = 0.10,
        joystick_index: int = 0,
        num_arm_joints: int = 6,
        num_base_joints: int = 3,
    ) -> None:
        """Initialize the gamepad teleop source.

        Raises:
            ValueError: If parameters are outside valid ranges.
        """
        if vx <= 0 or vy <= 0 or vtheta <= 0:
            msg = "vx, vy and vtheta must be positive"
            raise ValueError(msg)
        if not 0.0 <= deadzone < 1.0:
            msg = "deadzone must satisfy 0.0 <= deadzone < 1.0"
            raise ValueError(msg)
        if joystick_index < 0:
            msg = "joystick_index must be >= 0"
            raise ValueError(msg)
        if num_arm_joints < 0 or num_base_joints <= 0:
            msg = "num_arm_joints must be >= 0 and num_base_joints must be > 0"
            raise ValueError(msg)

        self._vx = vx
        self._vy = vy
        self._vtheta = vtheta
        self._deadzone = deadzone
        self._joystick_index = joystick_index
        self._num_arm_joints = num_arm_joints
        self._num_base_joints = num_base_joints
        self._num_joints = num_arm_joints + num_base_joints
        self._commands = np.zeros(_MAX_BASE_AXES, dtype=np.float32)

        self._pygame: _PygameModule | None = None
        self._joystick: _PygameJoystick | None = None

    def connect(
        self,
        *,
        bus: _CallbackBus,  # noqa: ARG002
        session_id: str,  # noqa: ARG002
    ) -> None:
        """Initialize pygame and connect to the configured joystick.

        Raises:
            RuntimeError: If pygame is unavailable or no controller is present.
        """
        if self._joystick is not None:
            return

        try:
            pygame = _import_pygame()
        except ModuleNotFoundError as exc:
            msg = 'GamepadTeleop requires pygame. Install it with: uv add "physicalai-lekiwi-plugin[gamepad]"'
            raise RuntimeError(msg) from exc

        pygame.init()
        pygame.joystick.init()

        count = int(pygame.joystick.get_count())
        if self._joystick_index >= count:
            msg = f"No gamepad at index {self._joystick_index}. Detected {count} connected controller(s)."
            pygame.joystick.quit()
            raise RuntimeError(msg)

        joystick = pygame.joystick.Joystick(self._joystick_index)
        joystick.init()

        self._pygame = pygame
        self._joystick = joystick

    def update(
        self,
        robot_state: RobotObservation,
        camera_frames: Mapping[str, Frame],  # noqa: ARG002
        step: int,  # noqa: ARG002
    ) -> np.ndarray:
        """Return the next action: arm held, base from current analog stick state.

        Raises:
            RuntimeError: If called before :meth:`connect`.
        """
        joystick = self._joystick
        pygame = self._pygame
        if joystick is None or pygame is None:
            msg = "GamepadTeleop is not connected. Call connect() first."
            raise RuntimeError(msg)

        # Pump and drain events so axis/button state stays fresh.
        pygame.event.pump()
        pygame.event.get()

        left_x = self._read_axis(joystick, axis=0)
        left_y = self._read_axis(joystick, axis=1)
        right_x = self._read_axis(joystick, axis=3)

        self._commands[0] = -left_y * self._vx
        self._commands[1] = -left_x * self._vy
        self._commands[2] = -right_x * self._vtheta

        action = np.zeros(self._num_joints, dtype=np.float32)
        action[: self._num_arm_joints] = np.asarray(
            robot_state.joint_positions[: self._num_arm_joints],
            dtype=np.float32,
        )
        action[self._num_arm_joints : self._num_joints] = self._commands
        return action

    def disconnect(self) -> None:
        """Release joystick and pygame joystick subsystem resources."""
        joystick = self._joystick
        pygame = self._pygame
        if joystick is not None and hasattr(joystick, "quit"):
            joystick.quit()
        if pygame is not None:
            with_quit = getattr(pygame.joystick, "quit", None)
            if callable(with_quit):
                with_quit()
        self._joystick = None
        self._pygame = None

    def _read_axis(self, joystick: _PygameJoystick, *, axis: int) -> float:
        try:
            value = float(joystick.get_axis(axis))
        except Exception:  # noqa: BLE001
            return 0.0
        return 0.0 if abs(value) < self._deadzone else value
