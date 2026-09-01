"""Composite action source: combine any number of action sources by joint indices.

Lets you drive a robot from multiple action sources at once — for example a
:class:`~physicalai.runtime.TeleopSource` leader for the arm joints plus a
:class:`~physicalai_common_extras.KeyboardTeleop` for the base, or a leader
for the left arm and a :class:`~physicalai.runtime.PolicySource` for the
right arm. Each channel's source fills a subset of the full action.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from physicalai.config import export_config

if TYPE_CHECKING:
    from collections.abc import Mapping

    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation
    from physicalai.runtime._callback_bus import _CallbackBus
    from physicalai.runtime.action_sources.base import ActionSource


@export_config(class_path="physicalai_common_extras.CompositeChannel")
class CompositeChannel:
    """One contribution to a :class:`CompositeSource`.

    Args:
        source: The action source producing the values for ``indices``.
        indices: Output entries (into the robot's full action) filled by this
            channel's source. Must be disjoint across channels.
    """

    def __init__(self, source: ActionSource, indices: list[int]) -> None:
        """Initialize the channel."""
        self.source = source
        self.indices = list(indices)


@export_config(class_path="physicalai_common_extras.CompositeSource")
class CompositeSource:
    """Combine multiple action sources, each filling a subset of the action.

    The runtime always sends a full action, so the composed action has as many
    entries as the robot's observation. Each channel's source fills the action
    entries given by its ``indices``; the channels must cover every joint
    exactly once (disjoint indices whose union spans the full action), which
    is validated against the observation at runtime.

    Args:
        channels: Ordered channel list; ``connect``/``disconnect`` are
            forwarded to every channel's source.

    Raises:
        ValueError: If the channels' indices overlap.
    """

    def __init__(self, channels: list[CompositeChannel]) -> None:
        """Initialize the composite source."""
        self._channels = list(channels)
        self._validate_disjoint()

    def connect(self, *, bus: _CallbackBus, session_id: str) -> None:
        """Connect every channel's source.

        Args:
            bus: Callback bus forwarded to each source.
            session_id: Session identifier forwarded to each source.
        """
        for channel in self._channels:
            channel.source.connect(bus=bus, session_id=session_id)

    def update(
        self,
        robot_state: RobotObservation,
        camera_frames: Mapping[str, Frame],
        step: int,
    ) -> np.ndarray:
        """Return the composed action: each channel's output at its indices.

        Args:
            robot_state: The follower observation; its length defines the full
                action size and is forwarded to each source.
            camera_frames: Camera frames, forwarded to each source.
            step: Current control-loop step, forwarded to each source.

        Returns:
            The full action vector for the robot.

        Raises:
            ValueError: If the channels do not cover every joint exactly once,
                or a channel's output length does not match its indices.
        """
        num_joints = len(robot_state.joint_positions)
        self._validate_coverage(num_joints)
        action = np.zeros(num_joints, dtype=np.float32)
        for channel in self._channels:
            out = np.asarray(
                channel.source.update(robot_state, camera_frames, step),
                dtype=np.float32,
            )
            if out.shape[0] != len(channel.indices):
                msg = (
                    f"channel {channel.source!r} produced {out.shape[0]} values but has {len(channel.indices)} indices"
                )
                raise ValueError(msg)
            action[channel.indices] = out
        return action

    def disconnect(self) -> None:
        """Disconnect every channel's source."""
        for channel in self._channels:
            channel.source.disconnect()

    def _validate_disjoint(self) -> None:
        seen: set[int] = set()
        for channel in self._channels:
            for index in channel.indices:
                if index in seen:
                    msg = f"composite indices must be disjoint; duplicate index {index}"
                    raise ValueError(msg)
                seen.add(index)

    def _validate_coverage(self, num_joints: int) -> None:
        covered = {index for channel in self._channels for index in channel.indices}
        expected = set(range(num_joints))
        if covered == expected:
            return
        missing = sorted(expected - covered)
        out_of_range = sorted(covered - expected)
        details = []
        if missing:
            details.append(f"missing joints {missing}")
        if out_of_range:
            details.append(f"out-of-range indices {out_of_range}")
        msg = f"composite channels must cover joints 0..{num_joints - 1}: {'; '.join(details)}"
        raise ValueError(msg)
