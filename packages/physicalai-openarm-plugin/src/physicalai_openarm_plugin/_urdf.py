"""URDF path utility for bundled OpenArm visualization assets."""

from __future__ import annotations

import importlib.resources as ir
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path


def get_urdf_path() -> Path:
    """Return the directory containing the bundled OpenArm URDF assets."""
    with ir.as_file(ir.files("physicalai_openarm_plugin")) as package_path:
        for candidate in (package_path.parent / "urdf", package_path.parent.parent / "urdf"):
            if candidate.exists():
                return candidate
    return package_path.parent / "urdf"
