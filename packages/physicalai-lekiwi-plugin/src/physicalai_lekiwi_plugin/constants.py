from __future__ import annotations

from enum import IntEnum
from typing import Final

import numpy as np

LEKIWI_JOINT_ORDER: Final = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
    "base_left_wheel",
    "base_back_wheel",
    "base_right_wheel",
)

LEKIWI_ARM_JOINTS: Final = LEKIWI_JOINT_ORDER[:6]
LEKIWI_BASE_JOINTS: Final = LEKIWI_JOINT_ORDER[6:]

LEKIWI_MOTOR_IDS: Final = {
    "arm_shoulder_pan": 1,
    "arm_shoulder_lift": 2,
    "arm_elbow_flex": 3,
    "arm_wrist_flex": 4,
    "arm_wrist_roll": 5,
    "arm_gripper": 6,
    "base_left_wheel": 7,
    "base_back_wheel": 8,
    "base_right_wheel": 9,
}

LEKIWI_JOINT_LIMITS_DEG: Final = {
    "arm_shoulder_pan": (-150.0, 150.0),
    "arm_shoulder_lift": (-1.0, 170.0),
    "arm_elbow_flex": (-200.0, 1.0),
    "arm_wrist_flex": (-80.0, 90.0),
    "arm_wrist_roll": (-90.0, 90.0),
    "arm_gripper": (-0.0, 270.0),
}

TICKS_PER_REVOLUTION: Final = 4096
STEPS_PER_DEG: Final = TICKS_PER_REVOLUTION / 360.0
RADIANS_PER_TICK: Final = 2.0 * np.pi / TICKS_PER_REVOLUTION

MAX_SPEED_RAD_S: Final = 4.712389
MAX_SPEED_DEG_S: Final = float(np.degrees(MAX_SPEED_RAD_S))

POSITION_MODE: Final = 0
VELOCITY_MODE: Final = 1

ARM_P_COEFFICIENT: Final = 16
ARM_I_COEFFICIENT: Final = 0
ARM_D_COEFFICIENT: Final = 32

WHEEL_RADIUS: Final = 0.05
BASE_RADIUS: Final = 0.125
MAX_RAW_WHEEL: Final = 3000
WHEEL_ANGLES_DEG: Final = (240.0, 0.0, 120.0)
WHEEL_OFFSET_DEG: Final = -90.0

PROTOCOL_VERSION: Final = 0

VALID_ROLES: Final = frozenset({"leader", "follower"})


class STS3215Addr(IntEnum):
    RETURN_DELAY_TIME = 7
    MAX_TORQUE_LIMIT = 16
    P_COEFFICIENT = 21
    D_COEFFICIENT = 22
    I_COEFFICIENT = 23
    PROTECTION_CURRENT = 28
    OPERATING_MODE = 33
    OVERLOAD_TORQUE = 36
    TORQUE_ENABLE = 40
    ACCELERATION = 41
    GOAL_POSITION = 42
    PRESENT_POSITION = 56
    GOAL_VELOCITY = 48
    PRESENT_VELOCITY = 62
    MAXIMUM_ACCELERATION = 85


class STS3215Len(IntEnum):
    RETURN_DELAY_TIME = 1
    MAX_TORQUE_LIMIT = 2
    P_COEFFICIENT = 1
    D_COEFFICIENT = 1
    I_COEFFICIENT = 1
    PROTECTION_CURRENT = 2
    OPERATING_MODE = 1
    OVERLOAD_TORQUE = 1
    TORQUE_ENABLE = 1
    ACCELERATION = 1
    GOAL_POSITION = 2
    PRESENT_POSITION = 2
    GOAL_VELOCITY = 2
    PRESENT_VELOCITY = 2
    MAXIMUM_ACCELERATION = 1
