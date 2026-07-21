from __future__ import annotations

import importlib.resources as ir
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path


def get_urdf_path() -> Path:
    traversal = ir.files("physicalai_bimanual_so101_plugin")
    with ir.as_file(traversal) as p:
        candidates = (p.parent / "urdf", p.parent.parent / "urdf")
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[1]
