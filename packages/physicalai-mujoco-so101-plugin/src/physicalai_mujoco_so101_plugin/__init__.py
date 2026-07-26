# ruff: file-ignore[undocumented-public-package, import-outside-top-level]
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from physicalai_mujoco_so101_plugin.mujoco_robot import MuJoCoSO101 as MuJoCoSO101
    from physicalai_mujoco_so101_plugin.mujoco_robot import (
        MuJoCoSO101Observation as MuJoCoSO101Observation,
    )

__all__ = [
    "MuJoCoSO101",
    "MuJoCoSO101Observation",
]


def __getattr__(name: str) -> object:
    if name == "MuJoCoSO101":
        from physicalai_mujoco_so101_plugin.mujoco_robot import MuJoCoSO101

        return MuJoCoSO101
    if name == "MuJoCoSO101Observation":
        from physicalai_mujoco_so101_plugin.mujoco_robot import MuJoCoSO101Observation

        return MuJoCoSO101Observation
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
