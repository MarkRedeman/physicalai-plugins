"""Runtime helpers for safe OpenArm state-reading examples."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from physicalai.config import export_config

if TYPE_CHECKING:
    from collections.abc import Mapping

    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation
    from physicalai.runtime._callback_bus import _CallbackBus
    from physicalai.runtime.events import TickEvent


@export_config(class_path="physicalai_openarm_plugin.motion.HoldPoseSource")
class HoldPoseSource:
    """Echo the current observation for runtimes that require an action source."""

    def __init__(self) -> None:
        """Initialize the stateless action source."""

    def connect(self, *, bus: _CallbackBus, session_id: str) -> None:
        """Perform no setup for the runtime action-source protocol."""
        _ = self, bus, session_id

    def update(
        self,
        robot_state: RobotObservation,
        camera_frames: Mapping[str, Frame],
        step: int,
    ) -> np.ndarray:
        """Return observed positions unchanged."""
        _ = self, camera_frames, step
        return np.asarray(robot_state.joint_positions, dtype=np.float32)

    def disconnect(self) -> None:
        """Perform no teardown for the runtime action-source protocol."""
        _ = self


@export_config(class_path="physicalai_openarm_plugin.motion.JointLogger")
class JointLogger:
    """Print observed joint positions periodically."""

    def __init__(self, throttle_steps: int = 1) -> None:  # noqa: D107
        if throttle_steps <= 0:
            msg = "throttle_steps must be positive"
            raise ValueError(msg)
        self._throttle_steps = throttle_steps

    def on_tick(self, event: TickEvent) -> None:
        """Print joint positions observed on this tick."""
        if event.step % self._throttle_steps == 0:
            values = "  ".join(f"{value:8.2f}" for value in event.robot_state.joint_positions)
            print(f"[{event.robot_state.timestamp:13.3f}]  {values}")  # noqa: T201
