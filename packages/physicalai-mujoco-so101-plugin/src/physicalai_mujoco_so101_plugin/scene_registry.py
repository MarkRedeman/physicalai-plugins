"""Scene definitions and reset behavior for the MuJoCo SO-101 simulation."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from physicalai_mujoco_so101_plugin._urdf import get_urdf_path

if TYPE_CHECKING:
    from pathlib import Path

ResetFn = Callable[[object, object, np.random.Generator], None]


@dataclass(frozen=True)
class SceneConfig:
    """Metadata and reset configuration for a simulation scene."""

    scene_id: str
    display_name: str
    description: str
    scene_xml_relpath: str
    free_joints: tuple[str, ...] = ()
    target_bodies: tuple[str, ...] = ()
    spawn_center: tuple[float, float] = (0.22, 0.0)
    spawn_min_r: float = 0.05
    spawn_max_r: float = 0.14
    spawn_angle_half_deg: float = 50.0
    block_min_sep: float = 0.09
    target_min_sep: float = 0.11

    @property
    def scene_xml_path(self) -> Path:
        """Absolute path to this scene's XML model."""
        return get_urdf_path() / self.scene_xml_relpath


# ---------------------------------------------------------------------------
# Scene reset functions
# ---------------------------------------------------------------------------


def _pick_lift_reset(model: object, data: object, rng: np.random.Generator) -> None:  # noqa: PLR0914
    import mujoco  # noqa: PLC0415

    block_joints = [f"block{i}:joint" for i in range(1, 4)]
    target_body = "target"
    center = (0.24, 0.0)
    min_r, max_r = 0.08, 0.34
    angle_half_deg = 125.0
    block_min_sep, target_min_sep = 0.09, 0.11

    def sample_xy() -> tuple[float, float]:
        r = float(rng.uniform(min_r, max_r))
        theta = float(rng.uniform(-np.radians(angle_half_deg), np.radians(angle_half_deg)))
        return (center[0] + r * np.cos(theta), center[1] + r * np.sin(theta))

    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_body)
    if tid >= 0:
        tx, ty = float(model.body_pos[tid][0]), float(model.body_pos[tid][1])
    else:
        tx, ty = center

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


def _pick_place_reset(model: object, data: object, rng: np.random.Generator) -> None:  # noqa: PLR0914
    import mujoco  # noqa: PLC0415

    block_joints = ("obj1:joint", "obj2:joint")
    target_body = "target_zone"
    center = (0.26, 0.0)
    min_r, max_r = 0.06, 0.30
    angle_half_deg = 135.0
    block_min_sep, target_min_sep = 0.10, 0.08

    def sample_xy() -> tuple[float, float]:
        r = float(rng.uniform(min_r, max_r))
        theta = float(rng.uniform(-np.radians(angle_half_deg), np.radians(angle_half_deg)))
        return (center[0] + r * np.cos(theta), center[1] + r * np.sin(theta))

    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_body)
    if tid >= 0:
        tx, ty = float(model.body_pos[tid][0]), float(model.body_pos[tid][1])
    else:
        tx, ty = center

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


def _single_pick_place_reset(model: object, data: object, rng: np.random.Generator) -> None:  # noqa: PLR0914
    import mujoco  # noqa: PLC0415

    block_joints = ("block1:joint",)
    target_body = "target"
    # Compact arc in front of the SO-101 (easy teleop reach).
    center = (0.22, 0.0)
    min_r, max_r = 0.05, 0.14
    angle_half_deg = 50.0
    _block_min_sep, target_min_sep = 0.09, 0.11

    def sample_xy() -> tuple[float, float]:
        r = float(rng.uniform(min_r, max_r))
        theta = float(rng.uniform(-np.radians(angle_half_deg), np.radians(angle_half_deg)))
        return (center[0] + r * np.cos(theta), center[1] + r * np.sin(theta))

    # Keep the green plate fixed; only respawn the cube away from it.
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target_body)
    if tid >= 0:
        tx, ty = float(model.body_pos[tid][0]), float(model.body_pos[tid][1])
    else:
        tx, ty = center

    positions: list[tuple[float, float]] = []
    for _ in block_joints:
        best: tuple[float, float] | None = None
        for _ in range(200):
            x, y = sample_xy()
            best = (x, y)
            far_from_target = np.hypot(x - tx, y - ty) >= target_min_sep
            if far_from_target:
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


def _garment_fold_reset(model: object, data: object, rng: np.random.Generator) -> None:  # noqa: ARG001
    import mujoco  # noqa: PLC0415

    home = {
        "left_shoulder_pan": -1.1,
        "left_shoulder_lift": 0.3,
        "left_elbow_flex": 0.8,
        "left_wrist_flex": 0.3,
        "right_shoulder_pan": 1.1,
        "right_shoulder_lift": 0.3,
        "right_elbow_flex": 0.8,
        "right_wrist_flex": 0.3,
    }

    for joint_name, val in home.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            continue
        qpos_adr = int(model.jnt_qposadr[jid])
        dof_adr = int(model.jnt_dofadr[jid])
        data.qpos[qpos_adr] = val
        data.qvel[dof_adr] = 0.0
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
        if aid >= 0:
            data.ctrl[aid] = val

    if model.nflex > 0:
        first_vertex_body = int(model.flex_vertbodyid[0])
        flex_qpos_adr = min(
            int(model.jnt_qposadr[j]) for j in range(model.njnt) if int(model.jnt_bodyid[j]) == first_vertex_body
        )
        flex_dof_adr = min(
            int(model.jnt_dofadr[j]) for j in range(model.njnt) if int(model.jnt_bodyid[j]) == first_vertex_body
        )
        data.qpos[flex_qpos_adr : flex_qpos_adr + 3 * model.nflexvert] = model.flex_vert.ravel()
        data.qvel[flex_dof_adr : flex_dof_adr + 3 * model.nflexvert] = 0.0

    mujoco.mj_forward(model, data)


def _yahtzee_reset(model: object, data: object, rng: np.random.Generator) -> None:  # noqa: PLR0914
    import mujoco  # noqa: PLC0415

    die_joints = [f"die{i}:joint" for i in range(1, 7)]

    cx, cy = 0.30, 0.0
    cup_jitter = 0.005

    for joint_name in die_joints:
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
        data.qpos[qpos_adr + 3 : qpos_adr + 7] = [
            c * np.sin(tilt / 2.0),
            s * np.sin(tilt / 2.0),
            s * np.cos(tilt / 2.0),
            c * np.cos(tilt / 2.0),
        ]

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
    "single_pick_place": SceneConfig(
        scene_id="single_pick_place",
        display_name="Single Pick & Place",
        description="One block and a target disc",
        scene_xml_relpath="scenes/single_pick_place/scene.xml",
        free_joints=("block1:joint",),
        target_bodies=("target",),
        # Keep spawns inside comfortable SO-101 teleop reach (was up to ~0.5 m).
        spawn_center=(0.22, 0.0),
        spawn_min_r=0.05,
        spawn_max_r=0.14,
        spawn_angle_half_deg=50.0,
        target_min_sep=0.11,
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
        free_joints=("die1:joint", "die2:joint", "die3:joint", "die4:joint", "die5:joint", "die6:joint"),
        target_bodies=(),
        spawn_center=(0.30, 0.0),
        spawn_min_r=0.06,
        spawn_max_r=0.18,
        spawn_angle_half_deg=160.0,
        block_min_sep=0.018,
        target_min_sep=0.02,
    ),
    "garment_fold": SceneConfig(
        scene_id="garment_fold",
        display_name="Garment Fold",
        description="Fold a flexible garment lying flat on a table",
        scene_xml_relpath="scenes/garment_fold/scene.xml",
    ),
}

_RESET_FUNCTIONS: dict[str, ResetFn] = {
    "pick_lift": _pick_lift_reset,
    "single_pick_place": _single_pick_place_reset,
    "pick_place": _pick_place_reset,
    "yahtzee": _yahtzee_reset,
    "garment_fold": _garment_fold_reset,
}


def get_scene(scene_id: str) -> SceneConfig:
    """Return the scene configuration identified by `scene_id`.

    Raises:
        KeyError: If `scene_id` is not registered.
    """
    if scene_id not in _SCENES:
        msg = f"Unknown scene {scene_id!r}. Available: {list(_SCENES)}"
        raise KeyError(msg)
    return _SCENES[scene_id]


def list_scenes() -> dict[str, SceneConfig]:
    """Return all scene configurations by ID."""
    return dict(_SCENES)


def get_reset_fn(scene_id: str) -> ResetFn | None:
    """Return the reset callback for `scene_id`, if one is registered."""
    return _RESET_FUNCTIONS.get(scene_id)
