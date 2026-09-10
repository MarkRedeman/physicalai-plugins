# ruff: noqa: RUF067

"""LeKiwi mobile manipulator plugin for PhysicalAI.

Provides a :class:`LeKiwi` driver compatible with the ``physicalai.robot.Robot`` protocol.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from physicalai_lekiwi_plugin._urdf import get_urdf_path as get_urdf_path

if TYPE_CHECKING:
    from physicalai_lekiwi_plugin.calibration import LeKiwiCalibration as LeKiwiCalibration
    from physicalai_lekiwi_plugin.lekiwi import LeKiwi as LeKiwi
    from physicalai_lekiwi_plugin.lekiwi import LeKiwiObservation as LeKiwiObservation
    from physicalai_lekiwi_plugin.lekiwi_leader import LeKiwiLeader as LeKiwiLeader
    from physicalai_lekiwi_plugin.lekiwi_leader import LeKiwiLeaderObservation as LeKiwiLeaderObservation
    from physicalai_lekiwi_plugin.teleop.composite import CompositeTeleop as CompositeTeleop
    from physicalai_lekiwi_plugin.teleop.gamepad import GamepadTeleop as GamepadTeleop
    from physicalai_lekiwi_plugin.teleop.keyboard import KeyboardTeleop as KeyboardTeleop

__all__ = [
    "CompositeTeleop",
    "GamepadTeleop",
    "KeyboardTeleop",
    "LeKiwi",
    "LeKiwiCalibration",
    "LeKiwiLeader",
    "LeKiwiLeaderObservation",
    "LeKiwiObservation",
    "get_urdf_path",
]

_LAZY_IMPORTS = {
    "LeKiwi": ("physicalai_lekiwi_plugin.lekiwi", "LeKiwi"),
    "LeKiwiObservation": ("physicalai_lekiwi_plugin.lekiwi", "LeKiwiObservation"),
    "LeKiwiLeader": ("physicalai_lekiwi_plugin.lekiwi_leader", "LeKiwiLeader"),
    "LeKiwiLeaderObservation": ("physicalai_lekiwi_plugin.lekiwi_leader", "LeKiwiLeaderObservation"),
    "LeKiwiCalibration": ("physicalai_lekiwi_plugin.calibration", "LeKiwiCalibration"),
    "CompositeTeleop": ("physicalai_lekiwi_plugin.teleop.composite", "CompositeTeleop"),
    "GamepadTeleop": ("physicalai_lekiwi_plugin.teleop.gamepad", "GamepadTeleop"),
    "KeyboardTeleop": ("physicalai_lekiwi_plugin.teleop.keyboard", "KeyboardTeleop"),
}


def __getattr__(name: str) -> object:
    import_path = _LAZY_IMPORTS.get(name)
    if import_path is not None:
        module_name, attribute_name = import_path
        module = __import__(module_name, fromlist=[attribute_name])
        return getattr(module, attribute_name)
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
