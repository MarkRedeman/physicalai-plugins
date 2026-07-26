"""MuJoCo SO-101 robot constants."""

from __future__ import annotations

from typing import Final

SO101_JOINT_ORDER: Final = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

NUM_JOINTS: Final = 6

JOINT_LIMITS_DEG: Final = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-96.8, 96.8),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-157.2, 162.8),
    "gripper": (-10.0, 100.0),
}
