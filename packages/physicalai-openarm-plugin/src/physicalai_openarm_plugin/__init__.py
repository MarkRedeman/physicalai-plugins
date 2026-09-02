# ruff: noqa: PLC0415

"""Direct SocketCAN/Damiao OpenArm plugin for PhysicalAI."""

from __future__ import annotations

from typing import TYPE_CHECKING

from physicalai_openarm_plugin._urdf import get_urdf_path as get_urdf_path

if TYPE_CHECKING:
    from physicalai_openarm_plugin.bimanual import BimanualOpenArmFollower as BimanualOpenArmFollower
    from physicalai_openarm_plugin.bimanual import BimanualOpenArmLeader as BimanualOpenArmLeader
    from physicalai_openarm_plugin.openarm import OpenArmFollower as OpenArmFollower
    from physicalai_openarm_plugin.openarm import OpenArmLeader as OpenArmLeader
    from physicalai_openarm_plugin.openarm import OpenArmObservation as OpenArmObservation

__all__ = [
    "BimanualOpenArmFollower",
    "BimanualOpenArmLeader",
    "OpenArmFollower",
    "OpenArmLeader",
    "OpenArmObservation",
    "get_urdf_path",
]


def __getattr__(name: str) -> object:
    if name in {"OpenArmFollower", "OpenArmLeader", "OpenArmObservation"}:
        from physicalai_openarm_plugin import openarm

        return getattr(openarm, name)
    if name in {"BimanualOpenArmFollower", "BimanualOpenArmLeader"}:
        from physicalai_openarm_plugin import bimanual

        return getattr(bimanual, name)
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
