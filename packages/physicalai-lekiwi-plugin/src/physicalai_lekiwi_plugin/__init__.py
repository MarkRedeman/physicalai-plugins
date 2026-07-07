from __future__ import annotations

from typing import TYPE_CHECKING

from physicalai_lekiwi_plugin._urdf import get_urdf_path as get_urdf_path

if TYPE_CHECKING:
    from physicalai_lekiwi_plugin.lekiwi import LeKiwi as LeKiwi
    from physicalai_lekiwi_plugin.lekiwi import LeKiwiObservation as LeKiwiObservation

__all__ = [
    "LeKiwi",
    "LeKiwiObservation",
    "get_urdf_path",
]


def __getattr__(name: str) -> object:
    if name == "LeKiwi":
        from physicalai_lekiwi_plugin.lekiwi import LeKiwi

        return LeKiwi
    if name == "LeKiwiObservation":
        from physicalai_lekiwi_plugin.lekiwi import LeKiwiObservation

        return LeKiwiObservation
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
