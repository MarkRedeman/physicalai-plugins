"""Reusable action sources and runtime callbacks for PhysicalAI plugins.

These classes implement the :class:`physicalai.runtime.ActionSource`
protocol (plus a console callback) and can be wired into a ``RobotRuntime``
via a ``physicalai run --config`` YAML:

- :class:`CompositeSource` / :class:`CompositeChannel` — combine any number
  of action sources, each filling a subset of the action by joint indices.
- :class:`KeyboardTeleop` — WASD/QE base control with the arm held in place.
- :class:`SineWaveSource` — sinusoidal joint targets (``move_joints``).
- :class:`HoldPoseSource` — echo the current observation (``read_joints``).
- :class:`JointLogger` — print observed joint positions each tick.
"""

from __future__ import annotations

from physicalai_common_extras.composite import CompositeChannel, CompositeSource
from physicalai_common_extras.keyboard import KeyboardTeleop
from physicalai_common_extras.motion import HoldPoseSource, JointLogger, SineWaveSource

__all__ = [
    "CompositeChannel",
    "CompositeSource",
    "HoldPoseSource",
    "JointLogger",
    "KeyboardTeleop",
    "SineWaveSource",
]
