# ruff: file-ignore[undocumented-public-module, undocumented-public-class, undocumented-public-method, undocumented-magic-method, undocumented-public-init, import-outside-top-level]
from __future__ import annotations

import contextlib
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TYPE_CHECKING, ClassVar

import numpy as np
from loguru import logger

from physicalai_mujoco_so101_plugin.constants import NUM_JOINTS, SO101_JOINT_ORDER

if TYPE_CHECKING:

    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation


@dataclass
class MuJoCoSO101Observation:
    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        return self.joint_positions


@dataclass(frozen=True)
class CameraConfig:
    name: str
    device: str
    width: int = 640
    height: int = 480
    fps: int = 30
    mirror_horizontal: bool = False


class MuJoCoSO101:
    JOINT_ORDER: ClassVar[tuple[str, ...]] = SO101_JOINT_ORDER
    NUM_JOINTS: ClassVar[int] = NUM_JOINTS
    BLOCK_FREEJOINTS: ClassVar[tuple[str, ...]] = ("block1:joint", "block2:joint", "block3:joint")
    TARGET_BODY_NAME: ClassVar[str] = "target"
    SPAWN_CENTER: ClassVar[tuple[float, float]] = (0.24, 0.0)
    SPAWN_MIN_R: ClassVar[float] = 0.08
    SPAWN_MAX_R: ClassVar[float] = 0.34
    SPAWN_ANGLE_HALF_DEG: ClassVar[float] = 125.0
    BLOCK_MIN_SEP: ClassVar[float] = 0.09
    TARGET_MIN_SEP: ClassVar[float] = 0.11

    def __init__(
        self,
        model_path: str,
        *,
        substeps: int = 1,
        enable_viewer: bool = False,
        cameras: list[CameraConfig | dict] | None = None,
        model: object = None,
        data: object = None,
    ) -> None:
        self._model_path = model_path
        self._substeps = substeps
        self._enable_viewer = enable_viewer
        self._cameras = [cam if isinstance(cam, CameraConfig) else CameraConfig(**cam) for cam in (cameras or [])]
        self._model = model
        self._data = data
        self._viewer = None
        self._camera_devices: dict[str, object] = {}
        self._camera_renderers: dict[str, object] = {}
        self._camera_last_frame_ts: dict[str, float] = {}
        self._block_joint_addrs: list[tuple[int, int]] = []
        self._target_body_id: int | None = None
        self._last_sim_time: float | None = None
        self._rng = np.random.default_rng()

    @property
    def joint_names(self) -> list[str]:
        return list(self.JOINT_ORDER)

    @property
    def device_ids(self) -> tuple[str, ...]:
        stem = Path(self._model_path).stem
        return (f"mujoco:{stem}",)

    def connect(self) -> None:
        if self.is_connected():
            return
        import mujoco

        logger.info("Loading MuJoCo model from {}", self._model_path)
        self._model = mujoco.MjModel.from_xml_path(self._model_path)
        self._data = mujoco.MjData(self._model)
        mujoco.mj_forward(self._model, self._data)
        self._last_sim_time = float(self._data.time)
        self._init_block_joint_addrs()
        logger.info(
            "MuJoCo SO101 connected ({} joints, timestep={})",
            self.NUM_JOINTS,
            self._model.opt.timestep,
        )

        if self._enable_viewer:
            try:
                import mujoco.viewer

                self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
                logger.info("MuJoCo viewer opened")
            except Exception as exc:  # ruff: ignore[blind-except]
                logger.warning("Failed to open MuJoCo viewer: {}", exc)
                self._enable_viewer = False

        self._init_cameras()

    def disconnect(self) -> None:
        for renderer in self._camera_renderers.values():
            with contextlib.suppress(Exception):
                renderer.close()
        self._camera_renderers.clear()
        self._camera_devices.clear()
        self._camera_last_frame_ts.clear()
        self._block_joint_addrs.clear()
        self._target_body_id = None
        self._last_sim_time = None

        if self._viewer is not None:
            with contextlib.suppress(Exception):
                self._viewer.close()
            self._viewer = None
        self._model = None
        self._data = None
        logger.info("MuJoCo SO101 disconnected")

    def is_connected(self) -> bool:
        return self._model is not None

    def _step_and_sync(self) -> None:
        import mujoco

        for _ in range(self._substeps):
            mujoco.mj_step(self._model, self._data)

        if self._viewer is not None:
            if self._viewer.is_running():
                self._viewer.sync()
                self._handle_viewer_reset()
            else:
                logger.info("MuJoCo viewer closed by user")
                self._enable_viewer = False
                self._viewer = None

        self._render_cameras()

    def _init_cameras(self) -> None:
        if not self._cameras:
            return

        import mujoco

        try:
            import pyfakewebcam
        except ImportError as exc:
            logger.warning("pyfakewebcam unavailable, disabling cameras: {}", exc)
            return

        for config in self._cameras:
            try:
                cam = pyfakewebcam.FakeWebcam(config.device, config.width, config.height)
                renderer = mujoco.Renderer(self._model, config.height, config.width)
            except OSError as exc:
                logger.warning("Camera '{}' unavailable: {}", config.name, exc)
                continue
            self._camera_devices[config.name] = cam
            self._camera_renderers[config.name] = renderer
            self._camera_last_frame_ts[config.name] = 0.0
            logger.info(
                "Camera started: {} -> {} ({}x{}@{} fps)",
                config.name, config.device, config.width, config.height, config.fps,
            )

    def _init_block_joint_addrs(self) -> None:
        import mujoco

        self._block_joint_addrs.clear()
        for joint_name in self.BLOCK_FREEJOINTS:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if jid < 0:
                continue
            qpos_addr = int(self._model.jnt_qposadr[jid])
            dof_addr = int(self._model.jnt_dofadr[jid])
            self._block_joint_addrs.append((qpos_addr, dof_addr))

        target_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, self.TARGET_BODY_NAME)
        self._target_body_id = int(target_id) if target_id >= 0 else None

    def _handle_viewer_reset(self) -> None:
        current = float(self._data.time)
        if self._last_sim_time is None:
            self._last_sim_time = current
            return
        if current + 1e-9 < self._last_sim_time:
            logger.info("Viewer reset detected; randomizing blocks")
            self._randomize_blocks()
            current = float(self._data.time)
        self._last_sim_time = current

    def _sample_spawn_xy(self) -> tuple[float, float]:
        r = float(self._rng.uniform(self.SPAWN_MIN_R, self.SPAWN_MAX_R))
        theta = float(self._rng.uniform(-np.radians(self.SPAWN_ANGLE_HALF_DEG), np.radians(self.SPAWN_ANGLE_HALF_DEG)))
        return (
            self.SPAWN_CENTER[0] + r * float(np.cos(theta)),
            self.SPAWN_CENTER[1] + r * float(np.sin(theta)),
        )

    def _sample_target_and_blocks(self, count: int) -> tuple[tuple[float, float], list[tuple[float, float]]]:
        target_xy = self._sample_spawn_xy()

        positions: list[tuple[float, float]] = []

        for _ in range(count):
            best: tuple[float, float] | None = None
            for _ in range(200):
                x, y = self._sample_spawn_xy()
                cand = (x, y)
                best = cand
                far_from_target = np.hypot(x - target_xy[0], y - target_xy[1]) >= self.TARGET_MIN_SEP
                if far_from_target and all(np.hypot(x - px, y - py) >= self.BLOCK_MIN_SEP for px, py in positions):
                    positions.append(cand)
                    break
            else:
                if best is not None:
                    positions.append(best)
        return target_xy, positions

    def _randomize_blocks(self) -> None:
        import mujoco

        if not self._block_joint_addrs:
            return

        target_xy, positions = self._sample_target_and_blocks(len(self._block_joint_addrs))
        if self._target_body_id is not None:
            self._model.body_pos[self._target_body_id] = [target_xy[0], target_xy[1], 0.001]

        for (qpos_addr, dof_addr), (x, y) in zip(self._block_joint_addrs, positions, strict=True):
            yaw = float(self._rng.uniform(0.0, 2.0 * np.pi))
            self._data.qpos[qpos_addr : qpos_addr + 3] = [x, y, 0.02]
            self._data.qpos[qpos_addr + 3 : qpos_addr + 7] = [np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)]
            self._data.qvel[dof_addr : dof_addr + 6] = 0.0
        mujoco.mj_forward(self._model, self._data)

    def _render_cameras(self) -> None:
        now = time.monotonic()
        for config in self._cameras:
            renderer = self._camera_renderers.get(config.name)
            cam = self._camera_devices.get(config.name)
            if renderer is None or cam is None:
                continue

            period = 1.0 / config.fps
            if now - self._camera_last_frame_ts[config.name] < period:
                continue

            try:
                renderer.update_scene(self._data, camera=config.name)
                frame = renderer.render()[:, :, :3][::-1, :, :]
            except RuntimeError as exc:
                logger.debug("Camera render error for '{}': {}", config.name, exc)
                continue

            if config.mirror_horizontal:
                frame = frame[:, ::-1, :]

            try:
                cam.schedule_frame(frame)
            except RuntimeError as exc:
                logger.debug("Camera publish error for '{}': {}", config.name, exc)
                continue

            self._camera_last_frame_ts[config.name] = now

    def get_observation(self) -> RobotObservation:
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)

        self._step_and_sync()

        positions = np.empty(self.NUM_JOINTS, dtype=np.float64)
        velocities = np.empty(self.NUM_JOINTS, dtype=np.float64)

        for i, name in enumerate(self.JOINT_ORDER):
            pos, vel = self._read_joint_state(name)
            positions[i] = np.degrees(pos)
            velocities[i] = np.degrees(vel)

        return MuJoCoSO101Observation(
            joint_positions=positions.astype(np.float32),
            timestamp=time.monotonic(),
            sensor_data={"velocities": velocities.astype(np.float32)},
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:  # ruff: ignore[unused-method-argument]
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)

        if action.shape != (self.NUM_JOINTS,):
            msg = f"Expected action shape ({self.NUM_JOINTS},), got {action.shape}"
            raise ValueError(msg)

        for i in range(self.NUM_JOINTS):
            self._data.ctrl[i] = float(np.radians(action[i]))

    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray:
        import mujoco

        if not self.is_connected():
            msg = "Robot is not connected."
            raise ConnectionError(msg)

        renderer = mujoco.Renderer(self._model, height, width)
        renderer.update_scene(self._data, camera=camera_name)
        rgb = renderer.render()
        renderer.close()
        return rgb

    def _read_joint_state(self, name: str) -> tuple[float, float]:
        import mujoco

        jnt_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jnt_id < 0:
            msg = f"Joint {name!r} not found in MuJoCo model"
            raise ValueError(msg)
        qpos_adr = self._model.jnt_qposadr[jnt_id]
        dof_adr = self._model.jnt_dofadr[jnt_id]
        return float(self._data.qpos[qpos_adr]), float(self._data.qvel[dof_adr])

    def __getstate__(self) -> dict:
        return {
            "_model_path": self._model_path,
            "_substeps": self._substeps,
            "_enable_viewer": self._enable_viewer,
            "_cameras": [asdict(cam) for cam in self._cameras],
        }

    def __setstate__(self, state: dict) -> None:
        self._model_path = state["_model_path"]
        self._substeps = state["_substeps"]
        self._enable_viewer = state.get("_enable_viewer", False)
        self._cameras = [CameraConfig(**cam) for cam in state.get("_cameras", [])]
        self._model = None
        self._data = None
        self._viewer = None
        self._camera_devices = {}
        self._camera_renderers = {}
        self._camera_last_frame_ts = {}
        self._block_joint_addrs = []
        self._target_body_id = None
        self._last_sim_time = None
        self._rng = np.random.default_rng()
