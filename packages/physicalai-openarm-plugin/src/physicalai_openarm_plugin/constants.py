"""OpenArm hardware constants based on its documented Damiao configuration."""

from __future__ import annotations

from typing import Final

OPENARM_JOINT_ORDER: Final[tuple[str, ...]] = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "gripper",
)
NUM_OPENARM_JOINTS: Final[int] = len(OPENARM_JOINT_ORDER)
NUM_BIMANUAL_OPENARM_JOINTS: Final[int] = NUM_OPENARM_JOINTS * 2

OPENARM_MOTOR_CONFIG: Final[dict[str, tuple[int, int, str]]] = {
    "joint_1": (0x01, 0x11, "dm8009"),
    "joint_2": (0x02, 0x12, "dm8009"),
    "joint_3": (0x03, 0x13, "dm4340"),
    "joint_4": (0x04, 0x14, "dm4340"),
    "joint_5": (0x05, 0x15, "dm4310"),
    "joint_6": (0x06, 0x16, "dm4310"),
    "joint_7": (0x07, 0x17, "dm4310"),
    "gripper": (0x08, 0x18, "dm4310"),
}

MOTOR_LIMITS: Final[dict[str, tuple[float, float, float]]] = {
    "dm4310": (12.5, 30.0, 10.0),
    "dm4340": (12.5, 8.0, 28.0),
    "dm8009": (12.5, 45.0, 54.0),
}

DEFAULT_POSITION_KP: Final[tuple[float, ...]] = (240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0, 25.0)
DEFAULT_POSITION_KD: Final[tuple[float, ...]] = (5.0, 5.0, 5.0, 5.0, 0.5, 0.5, 0.5, 0.5)

LEFT_JOINT_LIMITS_DEG: Final[dict[str, tuple[float, float]]] = {
    "joint_1": (-75.0, 75.0),
    "joint_2": (-90.0, 9.0),
    "joint_3": (-85.0, 85.0),
    "joint_4": (0.0, 135.0),
    "joint_5": (-85.0, 85.0),
    "joint_6": (-40.0, 40.0),
    "joint_7": (-80.0, 80.0),
    "gripper": (-65.0, 0.0),
}
RIGHT_JOINT_LIMITS_DEG: Final[dict[str, tuple[float, float]]] = {
    "joint_1": (-75.0, 75.0),
    "joint_2": (-9.0, 90.0),
    "joint_3": (-85.0, 85.0),
    "joint_4": (0.0, 135.0),
    "joint_5": (-85.0, 85.0),
    "joint_6": (-40.0, 40.0),
    "joint_7": (-80.0, 80.0),
    "gripper": (-65.0, 0.0),
}

CAN_CMD_ENABLE: Final[int] = 0xFC
CAN_CMD_DISABLE: Final[int] = 0xFD
CAN_CMD_REFRESH: Final[int] = 0xCC
CAN_PARAM_ID: Final[int] = 0x7FF
