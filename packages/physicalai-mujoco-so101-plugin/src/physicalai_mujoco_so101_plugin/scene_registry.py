from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from physicalai_mujoco_so101_plugin._urdf import get_urdf_path

ResetFn = Callable[[object, object, np.random.Generator], None]


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


# ---------------------------------------------------------------------------
# Scene reset functions
# ---------------------------------------------------------------------------

def _pick_lift_reset(model: object, data: object, rng: np.random.Generator) -> None:
    import mujoco

    block_joints = [f"block{i}:joint" for i in range(1, 4)]
    target_body = "target"
    center = (0.24, 0.0)
    min_r, max_r = 0.08, 0.34
    angle_half_deg = 125.0
    block_min_sep, target_min_sep = 0.09, 0.11

    def sample_xy():
        r = float(rng.uniform(min_r, max_r))
        theta = float(rng.uniform(-np.radians(angle_half_deg), np.radians(angle_half_deg)))
        return (center[0] + r * np.cos(theta), center[1] + r * np.sin(theta))

    tx, ty = sample_xy()
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_body)
    if tid >= 0:
        model.body_pos[tid] = [tx, ty, 0.001]

    positions: list[tuple[float, float]] = []
    for _ in block_joints:
        best: tuple[float, float] | None = None
        for _ in range(200):
            x, y = sample_xy()
            best = (x, y)
            far_from_target = np.hypot(x - tx, y - ty) >= target_min_sep
            if far_from_target and all(np.hypot(x - px, y - py) >= block_min_sep for px, py in positions):
                positions.append((x, y))
                break
        else:
            if best is not None:
                positions.append(best)

    for joint_name, (x, y) in zip(block_joints, positions, strict=True):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            continue
        qpos_adr = int(model.jnt_qposadr[jid])
        dof_adr = int(model.jnt_dofadr[jid])
        yaw = float(rng.uniform(0.0, 2.0 * np.pi))
        data.qpos[qpos_adr : qpos_adr + 3] = [x, y, 0.02]
        data.qpos[qpos_adr + 3 : qpos_adr + 7] = [np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)]
        data.qvel[dof_adr : dof_adr + 6] = 0.0

    mujoco.mj_forward(model, data)


def _pick_place_reset(model: object, data: object, rng: np.random.Generator) -> None:
    import mujoco

    block_joints = ("obj1:joint", "obj2:joint")
    target_body = "target_zone"
    center = (0.26, 0.0)
    min_r, max_r = 0.06, 0.30
    angle_half_deg = 135.0
    block_min_sep, target_min_sep = 0.10, 0.08

    def sample_xy():
        r = float(rng.uniform(min_r, max_r))
        theta = float(rng.uniform(-np.radians(angle_half_deg), np.radians(angle_half_deg)))
        return (center[0] + r * np.cos(theta), center[1] + r * np.sin(theta))

    tx, ty = sample_xy()
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_body)
    if tid >= 0:
        model.body_pos[tid] = [tx, ty, 0.001]

    positions: list[tuple[float, float]] = []
    for _ in block_joints:
        best: tuple[float, float] | None = None
        for _ in range(200):
            x, y = sample_xy()
            best = (x, y)
            far_from_target = np.hypot(x - tx, y - ty) >= target_min_sep
            if far_from_target and all(np.hypot(x - px, y - py) >= block_min_sep for px, py in positions):
                positions.append((x, y))
                break
        else:
            if best is not None:
                positions.append(best)

    for joint_name, (x, y) in zip(block_joints, positions, strict=True):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            continue
        qpos_adr = int(model.jnt_qposadr[jid])
        dof_adr = int(model.jnt_dofadr[jid])
        yaw = float(rng.uniform(0.0, 2.0 * np.pi))
        data.qpos[qpos_adr : qpos_adr + 3] = [x, y, 0.02]
        data.qpos[qpos_adr + 3 : qpos_adr + 7] = [np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)]
        data.qvel[dof_adr : dof_adr + 6] = 0.0

    mujoco.mj_forward(model, data)


def _yahtzee_reset(model: object, data: object, rng: np.random.Generator) -> None:
    import mujoco

    die_joints = [f"die{i}:joint" for i in range(1, 7)]

    cx, cy = 0.30, 0.0
    cup_jitter = 0.005

    for idx, joint_name in enumerate(die_joints):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            continue
        qpos_adr = int(model.jnt_qposadr[jid])
        dof_adr = int(model.jnt_dofadr[jid])

        r = float(rng.uniform(0.06, 0.18))
        theta = float(rng.uniform(0.0, 2.0 * np.pi))
        x = cx + float(rng.uniform(-cup_jitter, cup_jitter)) + r * float(np.cos(theta))
        y = cy + float(rng.uniform(-cup_jitter, cup_jitter)) + r * float(np.sin(theta))

        yaw = float(rng.uniform(0.0, 2.0 * np.pi))
        tilt = float(rng.uniform(-0.3, 0.3))
        c = np.cos(yaw / 2.0)
        s = np.sin(yaw / 2.0)
        drop_z = float(rng.uniform(0.12, 0.18))
        data.qpos[qpos_adr : qpos_adr + 3] = [x, y, drop_z]
        data.qpos[qpos_adr + 3 : qpos_adr + 7] = [c * np.sin(tilt / 2.0), s * np.sin(tilt / 2.0), s * np.cos(tilt / 2.0), c * np.cos(tilt / 2.0)]

        vx = float(rng.uniform(-0.5, 0.5))
        vy = float(rng.uniform(-0.5, 0.5))
        vz = float(rng.uniform(-0.4, -0.05))
        wx = float(rng.uniform(-15.0, 15.0))
        wy = float(rng.uniform(-15.0, 15.0))
        wz = float(rng.uniform(-8.0, 8.0))
        data.qvel[dof_adr : dof_adr + 6] = [vx, vy, vz, wx, wy, wz]

    mujoco.mj_forward(model, data)


# ---------------------------------------------------------------------------
# Scene configurations
# ---------------------------------------------------------------------------

_SCENES: dict[str, SceneConfig] = {
    "pick_lift": SceneConfig(
        scene_id="pick_lift",
        display_name="Pick & Lift",
        description="Three colored cubes and a target disc on a tabletop",
        scene_xml_relpath="scenes/pick_lift/scene.xml",
        free_joints=("block1:joint", "block2:joint", "block3:joint"),
        target_bodies=("target",),
    ),
    "pick_place": SceneConfig(
        scene_id="pick_place",
        display_name="Pick & Place",
        description="Two objects (cube + cylinder) and a target zone",
        scene_xml_relpath="scenes/pick_place/scene.xml",
        free_joints=("obj1:joint", "obj2:joint"),
        target_bodies=("target_zone",),
        spawn_center=(0.26, 0.0),
        spawn_min_r=0.06,
        spawn_max_r=0.30,
        spawn_angle_half_deg=135.0,
        block_min_sep=0.10,
        target_min_sep=0.08,
    ),
    "yahtzee": SceneConfig(
        scene_id="yahtzee",
        display_name="Yahtzee",
        description="Pick up 6 dice and place them in the cup",
        scene_xml_relpath="scenes/yahtzee/scene.xml",
        free_joints=("die1:joint", "die2:joint", "die3:joint",
                      "die4:joint", "die5:joint", "die6:joint"),
        target_bodies=(),
        spawn_center=(0.30, 0.0),
        spawn_min_r=0.06,
        spawn_max_r=0.18,
        spawn_angle_half_deg=160.0,
        block_min_sep=0.018,
        target_min_sep=0.02,
    ),
}

_RESET_FUNCTIONS: dict[str, ResetFn] = {
    "pick_lift": _pick_lift_reset,
    "pick_place": _pick_place_reset,
    "yahtzee": _yahtzee_reset,
}


def get_scene(scene_id: str) -> SceneConfig:
    if scene_id not in _SCENES:
        msg = f"Unknown scene {scene_id!r}. Available: {list(_SCENES)}"
        raise KeyError(msg)
    return _SCENES[scene_id]


def list_scenes() -> dict[str, SceneConfig]:
    return dict(_SCENES)


def get_reset_fn(scene_id: str) -> ResetFn | None:
    return _RESET_FUNCTIONS.get(scene_id)
