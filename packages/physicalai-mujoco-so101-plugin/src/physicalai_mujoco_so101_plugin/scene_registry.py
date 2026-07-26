from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from physicalai_mujoco_so101_plugin._urdf import get_urdf_path


@dataclass(frozen=True)
class SceneConfig:
    scene_id: str
    display_name: str
    description: str
    scene_xml_relpath: str
    free_joints: tuple[str, ...] = ()
    target_bodies: tuple[str, ...] = ()
    spawn_center: tuple[float, float] = (0.24, 0.0)
    spawn_min_r: float = 0.08
    spawn_max_r: float = 0.34
    spawn_angle_half_deg: float = 125.0
    block_min_sep: float = 0.09
    target_min_sep: float = 0.11

    @property
    def scene_xml_path(self) -> Path:
        return get_urdf_path() / self.scene_xml_relpath


_SCENES: dict[str, SceneConfig] = {
    "pick_lift": SceneConfig(
        scene_id="pick_lift",
        display_name="Pick & Lift",
        description="Three colored cubes and a target disc on a tabletop",
        scene_xml_relpath="scenes/pick_lift/scene.xml",
        free_joints=("block1:joint", "block2:joint", "block3:joint"),
        target_bodies=("target",),
    ),
}


def get_scene(scene_id: str) -> SceneConfig:
    if scene_id not in _SCENES:
        msg = f"Unknown scene {scene_id!r}. Available: {list(_SCENES)}"
        raise KeyError(msg)
    return _SCENES[scene_id]


def list_scenes() -> dict[str, SceneConfig]:
    return dict(_SCENES)
