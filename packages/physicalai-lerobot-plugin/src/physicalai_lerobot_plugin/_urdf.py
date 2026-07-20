from __future__ import annotations

import importlib.resources as ir
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path


def get_urdf_path() -> Path:
    traversal = ir.files("physicalai_lerobot_plugin")
    with ir.as_file(traversal) as p:
        return p.parent.parent.joinpath("urdf")
