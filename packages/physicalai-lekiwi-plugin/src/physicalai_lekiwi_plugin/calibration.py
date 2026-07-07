from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from physicalai_lekiwi_plugin.constants import LEKIWI_JOINT_ORDER, TICKS_PER_REVOLUTION


@dataclass(frozen=True)
class LeKiwiJointCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    @property
    def direction(self) -> int:
        return -1 if self.drive_mode == 1 else 1


@dataclass(frozen=True)
class LeKiwiCalibration:
    joints: dict[str, LeKiwiJointCalibration]

    @classmethod
    def from_path(cls, path: str | Path) -> LeKiwiCalibration:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: object) -> LeKiwiCalibration:
        if not isinstance(data, dict):
            msg = "Calibration file must be a JSON object mapping joint names to calibration data"
            raise TypeError(msg)

        required_joints = set(LEKIWI_JOINT_ORDER)
        missing = required_joints - data.keys()
        if missing:
            msg = f"Calibration file is missing joints: {sorted(missing)}"
            raise ValueError(msg)

        joints: dict[str, LeKiwiJointCalibration] = {}
        for name in LEKIWI_JOINT_ORDER:
            cal = data[name]
            if not isinstance(cal, dict):
                msg = f"Joint '{name}' calibration must be a dict"
                raise TypeError(msg)
            for key in ("id", "drive_mode", "homing_offset", "range_min", "range_max"):
                if key not in cal:
                    msg = f"Joint '{name}' missing required calibration key '{key}'"
                    raise ValueError(msg)
            if cal["drive_mode"] not in {0, 1}:
                msg = f"Joint '{name}' drive_mode must be 0 or 1, got {cal['drive_mode']}"
                raise ValueError(msg)

            joint = LeKiwiJointCalibration(
                id=int(cal["id"]),
                drive_mode=int(cal["drive_mode"]),
                homing_offset=int(cal["homing_offset"]),
                range_min=int(cal["range_min"]),
                range_max=int(cal["range_max"]),
            )
            if joint.range_min >= joint.range_max:
                msg = f"Joint '{name}': range_min ({joint.range_min}) must be less than range_max ({joint.range_max})"
                raise ValueError(msg)
            if not (0 <= joint.range_min < TICKS_PER_REVOLUTION):
                msg = (
                    f"Joint '{name}': range_min ({joint.range_min}) is outside the valid "
                    f"STS3215 encoder range [0, {TICKS_PER_REVOLUTION - 1}]"
                )
                raise ValueError(msg)
            if not (0 <= joint.range_max < TICKS_PER_REVOLUTION):
                msg = (
                    f"Joint '{name}': range_max ({joint.range_max}) is outside the valid "
                    f"STS3215 encoder range [0, {TICKS_PER_REVOLUTION - 1}]"
                )
                raise ValueError(msg)
            joints[name] = joint

        ids = [j.id for j in joints.values()]
        if any(servo_id <= 0 for servo_id in ids):
            bad = {n: j.id for n, j in joints.items() if j.id <= 0}
            msg = f"All servo IDs must be positive integers, got: {bad}"
            raise ValueError(msg)
        if len(set(ids)) != len(ids):
            msg = "All servo IDs must be unique across joints."
            raise ValueError(msg)

        return cls(joints=joints)
