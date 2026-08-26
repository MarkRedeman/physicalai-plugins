"""Composite teleoperation: a leader arm combined with a keyboard/gamepad base.

Lets you teleoperate a robot (such as a LeKiwi) using both a leader arm —
which positions the arm joints — and a separate base source, such as
:class:`~physicalai_lekiwi_plugin.teleop.KeyboardTeleop`, which drives the
base velocities.
"""

from __future__ import annotations

import contextlib
from typing import TYPE_CHECKING

import numpy as np
from physicalai.config import export_config

if TYPE_CHECKING:
    from collections.abc import Mapping

    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import Robot, RobotObservation
    from physicalai.runtime._callback_bus import _CallbackBus
    from physicalai.runtime.action_sources.base import ActionSource


@export_config(class_path="physicalai_lekiwi_plugin.teleop.CompositeTeleop")
class CompositeTeleop:
    """Teleoperate a robot with a leader arm plus a separate base source.

    Reads the leader's arm joints (its leading ``num_arm_joints`` observed
    values) and combines them with the base velocities produced by
    ``base_source``, yielding a full action of
    ``num_arm_joints + num_base_joints`` entries. This allows driving a
    LeKiwi with a keyboard (base) while a leader arm — for example a second
    LeKiwi in ``role="leader"`` — positions the arm.

    Args:
        leader: The leader robot whose arm joints are relayed. Must support
            ``get_observation()``.
        base_source: An action source (e.g. :class:`KeyboardTeleop`) that
            produces the full action; its trailing ``num_base_joints`` values
            are taken as the base velocities.
        num_arm_joints: Number of leading action entries taken from the leader.
        num_base_joints: Number of trailing action entries taken from the base source.
    """

    def __init__(
        self,
        leader: Robot,
        base_source: ActionSource,
        *,
        num_arm_joints: int = 6,
        num_base_joints: int = 3,
    ) -> None:
        """Initialize the composite teleop source."""
        self._leader = leader
        self._base_source = base_source
        self._num_arm_joints = num_arm_joints
        self._num_base_joints = num_base_joints
        self._leader_owned = False

    def connect(self, *, bus: _CallbackBus, session_id: str) -> None:
        """Connect the leader (if needed) and forward to the base source.

        Args:
            bus: Callback bus forwarded to the base source.
            session_id: Session identifier forwarded to the base source.
        """
        if not self._leader.is_connected():
            self._leader.connect()
            self._leader_owned = True
        self._base_source.connect(bus=bus, session_id=session_id)

    def update(
        self,
        robot_state: RobotObservation,
        camera_frames: Mapping[str, Frame],
        step: int,
    ) -> np.ndarray:
        """Return the full action: leader arm joints + base source velocities.

        Args:
            robot_state: The follower observation, forwarded to the base source.
            camera_frames: Camera frames, forwarded to the base source.
            step: Current control-loop step, forwarded to the base source.

        Returns:
            An action of shape ``(num_arm_joints + num_base_joints,)``.
        """
        leader_obs = self._leader.get_observation()
        base_action = self._base_source.update(robot_state, camera_frames, step)
        arm = np.asarray(leader_obs.joint_positions[: self._num_arm_joints], dtype=np.float32)
        base = np.asarray(
            base_action[self._num_arm_joints : self._num_arm_joints + self._num_base_joints],
            dtype=np.float32,
        )
        return np.concatenate((arm, base))

    def disconnect(self) -> None:
        """Disconnect the base source and the leader (if this source connected it)."""
        self._base_source.disconnect()
        if self._leader_owned:
            with contextlib.suppress(Exception):
                self._leader.disconnect()
