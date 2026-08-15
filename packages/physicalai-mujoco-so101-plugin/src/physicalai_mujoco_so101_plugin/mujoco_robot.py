"""MuJoCo-backed SO-101 robot implementation."""

from __future__ import annotations

import contextlib
import queue
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TYPE_CHECKING, ClassVar

import numpy as np
from loguru import logger
from physicalai.config import export_config

from physicalai_mujoco_so101_plugin.constants import (
    BIMANUAL_NUM_JOINTS,
    BIMANUAL_SO101_JOINT_ORDER,
    NUM_JOINTS,
    SO101_JOINT_ORDER,
)

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation

    from physicalai_mujoco_so101_plugin.http_server import FrameBuffer, HttpServer, SimCommand


def _signal_owner_shutdown() -> None:
    """Request a graceful exit of the shared owner loop, if this runs inside one.

    The driver runs inside ``python -m physicalai.robot.transport.
    _owner_worker``, where the worker module is loaded as ``__main__``; check
    both module identities for the loop's shutdown event.
    """
    for module_name in ("__main__", "physicalai.robot.transport._owner_worker"):
        module = sys.modules.get(module_name)
        event = getattr(module, "shutdown", None) if module is not None else None
        if isinstance(event, threading.Event):
            event.set()
            return
    logger.warning("Owner shutdown event not found; use Ctrl+C or stop the process to exit")


@dataclass
class MuJoCoSO101Observation:
    """Observation returned by the MuJoCo SO-101 robot."""

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        """Joint positions represented as the robot state."""
        return self.joint_positions


@dataclass(frozen=True)
class CameraConfig:
    """Output configuration for a virtual camera.

    When ``device`` is ``None`` the camera is served only over HTTP
    (MJPEG/snapshot). Setting ``device`` to a v4l2loopback node also
    publishes frames there via pyfakewebcam.
    """

    name: str
    device: str | None = None
    width: int = 640
    height: int = 480
    fps: int = 30
    mirror_horizontal: bool = False


@export_config(class_path="physicalai_mujoco_so101_plugin.mujoco_robot.MuJoCoSO101")
class MuJoCoSO101:
    """SO-101 robot simulated with MuJoCo."""

    JOINT_ORDER: ClassVar[tuple[str, ...]] = SO101_JOINT_ORDER
    NUM_JOINTS: ClassVar[int] = NUM_JOINTS
    DEFAULT_BLOCK_FREEJOINTS: ClassVar[tuple[str, ...]] = ("block1:joint", "block2:joint", "block3:joint")
    DEFAULT_TARGET_BODY_NAME: ClassVar[str] = "target"
    DEFAULT_SPAWN_CENTER: ClassVar[tuple[float, float]] = (0.24, 0.0)
    DEFAULT_SPAWN_MIN_R: ClassVar[float] = 0.08
    DEFAULT_SPAWN_MAX_R: ClassVar[float] = 0.34
    DEFAULT_SPAWN_ANGLE_HALF_DEG: ClassVar[float] = 125.0
    DEFAULT_BLOCK_MIN_SEP: ClassVar[float] = 0.09
    DEFAULT_TARGET_MIN_SEP: ClassVar[float] = 0.11

    def __init__(
        self,
        model_path: str,
        *,
        substeps: int = 1,
        enable_viewer: bool = False,
        cameras: list[CameraConfig | dict] | None = None,
        model: object = None,
        data: object = None,
        scene_config: dict | None = None,
        http_host: str = "127.0.0.1",
        http_port: int = 0,
    ) -> None:
        """Initialize a disconnected simulation robot."""
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
        self._frame_buffers: dict[str, FrameBuffer] = {}
        self._commands: queue.Queue[SimCommand] = queue.Queue()
        self._http_host = http_host
        self._http_port = http_port
        self._http_server: HttpServer | None = None
        self._block_joint_addrs: list[tuple[int, int]] = []
        self._target_body_id: int | None = None
        self._last_sim_time: float | None = None
        self._rng = np.random.default_rng()
        self._pending_scene_switch: bool = False
        self._current_scene_id: str | None = None
        self._scene_on_reset: object | None = None
        self._scene_xml_mtimes: dict[str, float] = {}

        if scene_config is not None:
            self._free_joints: tuple[str, ...] = tuple(scene_config["free_joints"])
            self._target_body_name: str = scene_config["target_bodies"][0] if scene_config["target_bodies"] else ""
            self._spawn_center: tuple[float, float] = tuple(scene_config["spawn_center"])
            self._spawn_min_r: float = scene_config["spawn_min_r"]
            self._spawn_max_r: float = scene_config["spawn_max_r"]
            self._spawn_angle_half_deg: float = scene_config["spawn_angle_half_deg"]
            self._block_min_sep: float = scene_config["block_min_sep"]
            self._target_min_sep: float = scene_config["target_min_sep"]
            self._current_scene_id = scene_config.get("scene_id")
            if self._current_scene_id:
                from physicalai_mujoco_so101_plugin.scene_registry import get_reset_fn  # noqa: PLC0415

                self._scene_on_reset = get_reset_fn(self._current_scene_id)
        else:
            self._free_joints: tuple[str, ...] = self.DEFAULT_BLOCK_FREEJOINTS
            self._target_body_name: str = self.DEFAULT_TARGET_BODY_NAME
            self._spawn_center: tuple[float, float] = self.DEFAULT_SPAWN_CENTER
            self._spawn_min_r: float = self.DEFAULT_SPAWN_MIN_R
            self._spawn_max_r: float = self.DEFAULT_SPAWN_MAX_R
            self._spawn_angle_half_deg: float = self.DEFAULT_SPAWN_ANGLE_HALF_DEG
            self._block_min_sep: float = self.DEFAULT_BLOCK_MIN_SEP
            self._target_min_sep: float = self.DEFAULT_TARGET_MIN_SEP

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names."""
        return list(self.JOINT_ORDER)

    @property
    def device_ids(self) -> tuple[str, ...]:
        """This simulation's device identifier."""
        stem = Path(self._model_path).stem
        return (f"mujoco:{stem}",)

    def connect(self) -> None:
        """Load the model and initialize simulation resources."""
        if self.is_connected():
            return
        import mujoco  # noqa: PLC0415

        logger.info("Loading MuJoCo model from {}", self._model_path)
        self._model = mujoco.MjModel.from_xml_path(self._model_path)
        self._data = mujoco.MjData(self._model)
        mujoco.mj_forward(self._model, self._data)
        self._last_sim_time = float(self._data.time)
        self._init_block_joint_addrs()
        self._scene_xml_mtimes = self._snapshot_scene_xml_mtimes()
        logger.info(
            "MuJoCo SO101 connected ({} joints, timestep={})",
            self.NUM_JOINTS,
            self._model.opt.timestep,
        )

        if self._enable_viewer:
            try:
                import mujoco.viewer  # noqa: PLC0415

                self._viewer = mujoco.viewer.launch_passive(
                    self._model,
                    self._data,
                    key_callback=self._key_callback,
                )
                logger.info("MuJoCo viewer opened")
            except Exception as exc:  # noqa: BLE001
                logger.warning("Failed to open MuJoCo viewer: {}", exc)
                self._enable_viewer = False

        self._init_cameras()
        self._start_http_server()

    def disconnect(self) -> None:
        """Release simulation resources."""
        self._stop_http_server()
        for renderer in self._camera_renderers.values():
            with contextlib.suppress(Exception):
                renderer.close()
        self._camera_renderers.clear()
        self._camera_devices.clear()
        self._camera_last_frame_ts.clear()
        self._frame_buffers.clear()
        self._block_joint_addrs.clear()
        self._target_body_id = None
        self._last_sim_time = None

        self._close_viewer()
        self._model = None
        self._data = None
        self._current_scene_id = None
        self._scene_on_reset = None
        self._pending_scene_switch = False
        logger.info("MuJoCo SO101 disconnected")

    def is_connected(self) -> bool:
        """Return whether the simulation model is loaded."""
        return self._model is not None

    def _step_and_sync(self) -> None:
        import mujoco  # noqa: PLC0415

        self._drain_commands()
        self._check_scene_xml_camera()

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

        import mujoco  # noqa: PLC0415

        for config in self._cameras:
            try:
                renderer = mujoco.Renderer(self._model, config.height, config.width)
            except OSError as exc:
                logger.warning("Camera '{}' renderer unavailable: {}", config.name, exc)
                continue
            self._camera_renderers[config.name] = renderer
            if config.name not in self._frame_buffers:
                from physicalai_mujoco_so101_plugin.http_server import FrameBuffer  # noqa: PLC0415

                self._frame_buffers[config.name] = FrameBuffer(config.name)
            self._camera_last_frame_ts[config.name] = 0.0

            if config.device:
                self._init_v4l2_device(config)
            logger.info(
                "Camera started: {} ({}x{}@{} fps, v4l2={})",
                config.name,
                config.width,
                config.height,
                config.fps,
                config.device or "off",
            )

    def _init_v4l2_device(self, config: CameraConfig) -> None:
        try:
            import pyfakewebcam  # noqa: PLC0415
        except ImportError as exc:
            logger.warning("pyfakewebcam unavailable, camera '{}' v4l2 output disabled: {}", config.name, exc)
            return
        try:
            self._camera_devices[config.name] = pyfakewebcam.FakeWebcam(config.device, config.width, config.height)
        except OSError as exc:
            logger.warning("Camera '{}' v4l2 device '{}' unavailable: {}", config.name, config.device, exc)

    def _init_block_joint_addrs(self) -> None:
        import mujoco  # noqa: PLC0415

        self._block_joint_addrs.clear()
        for joint_name in self._free_joints:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if jid < 0:
                continue
            qpos_addr = int(self._model.jnt_qposadr[jid])
            dof_addr = int(self._model.jnt_dofadr[jid])
            self._block_joint_addrs.append((qpos_addr, dof_addr))

        target_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, self._target_body_name)
        self._target_body_id = int(target_id) if target_id >= 0 else None

    def _key_callback(self, key: int) -> None:
        if key in {ord("n"), ord("N")} and not self._pending_scene_switch:
            self._pending_scene_switch = True

    def _collect_scene_xml_paths(self) -> list[Path]:
        root_path = Path(self._model_path).resolve()
        visited: set[Path] = set()
        ordered_paths: list[Path] = []

        def walk(path: Path) -> None:
            normalized = path.resolve()
            if normalized in visited:
                return
            visited.add(normalized)
            ordered_paths.append(normalized)

            try:
                from defusedxml import ElementTree  # noqa: PLC0415

                root = ElementTree.parse(normalized).getroot()
            except (ElementTree.ParseError, OSError):
                return

            for include in root.findall(".//include"):
                include_file = include.get("file")
                if not include_file:
                    continue
                include_path = (normalized.parent / include_file).resolve()
                walk(include_path)

        walk(root_path)
        return ordered_paths

    def _snapshot_scene_xml_mtimes(self) -> dict[str, float]:
        mtimes: dict[str, float] = {}
        for xml_path in self._collect_scene_xml_paths():
            try:
                mtimes[str(xml_path)] = xml_path.stat().st_mtime
            except OSError:
                continue
        return mtimes

    def _check_scene_xml_camera(self) -> None:
        current_mtimes = self._snapshot_scene_xml_mtimes()
        if current_mtimes == self._scene_xml_mtimes:
            return
        logger.info("Scene XML changed, updating camera")
        self._scene_xml_mtimes = current_mtimes
        self._update_camera_from_xml()

    def _update_camera_from_xml(self) -> None:  # noqa: C901, PLR0912, PLR0915
        import mujoco  # noqa: PLC0415
        from defusedxml import ElementTree  # noqa: PLC0415

        roots = []
        for xml_path in self._collect_scene_xml_paths():
            try:
                roots.append(ElementTree.parse(xml_path).getroot())
            except ElementTree.ParseError as exc:
                logger.warning("Failed to parse XML {}: {}", xml_path, exc)

        if not roots:
            return

        def find_first(xpath: str) -> object | None:
            for root in roots:
                elem = root.find(xpath)
                if elem is not None:
                    return elem
            return None

        cam = find_first(".//camera[@name='overview']")
        if cam is None:
            return

        for body_name in ("overview_camera_rig", "overview_camera_tilt", "camera_mount", "camera_mount_wrist"):
            body_elem = find_first(f".//body[@name='{body_name}']")
            body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_elem is None or body_id < 0:
                continue

            pos_str = body_elem.get("pos")
            if pos_str:
                self._model.body_pos[body_id] = [float(x) for x in pos_str.split()]

            euler_str = body_elem.get("euler")
            if euler_str:
                euler_vals = [float(x) for x in euler_str.split()]
                if len(euler_vals) == 3:  # noqa: PLR2004
                    quat = np.zeros(4, dtype=np.float64)
                    mujoco.mju_euler2Quat(quat, euler_vals, "xyz")
                    self._model.body_quat[body_id] = quat
                    logger.info("Updated {}: {}", body_name, euler_vals)
                else:
                    logger.warning("Invalid {} euler values: {}", body_name, euler_str)

            quat_str = body_elem.get("quat")
            if quat_str:
                quat_vals = [float(x) for x in quat_str.split()]
                if len(quat_vals) == 4:  # noqa: PLR2004
                    self._model.body_quat[body_id] = quat_vals
                    logger.info("Updated {} quat: {}", body_name, quat_vals)
                else:
                    logger.warning("Invalid {} quat values: {}", body_name, quat_str)

        def update_camera_pose(camera_name: str, camera_elem: object) -> None:
            camera_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if camera_id < 0:
                return

            pos_str = camera_elem.get("pos")
            if pos_str:
                pos = [float(x) for x in pos_str.split()]
                self._model.cam_pos[camera_id] = pos

            fovy_str = camera_elem.get("fovy")
            if fovy_str:
                self._model.cam_fovy[camera_id] = float(fovy_str)

            xyaxes_str = camera_elem.get("xyaxes")
            euler_str = camera_elem.get("euler")
            quat_str = camera_elem.get("quat")

            if quat_str:
                quat_vals = [float(x) for x in quat_str.split()]
                if len(quat_vals) == 4:  # noqa: PLR2004
                    self._model.cam_quat[camera_id] = quat_vals
                    logger.info("Updated camera {} quat: {}", camera_name, quat_vals)
                else:
                    logger.warning("Invalid {} camera quat values: {}", camera_name, quat_str)
            elif xyaxes_str:
                vals = [float(x) for x in xyaxes_str.split()]
                if len(vals) == 6:  # noqa: PLR2004
                    mat = np.empty(9, dtype=np.float64)
                    mat[:3] = vals[:3]
                    mat[3:6] = vals[3:]
                    mat[6:] = np.cross(vals[:3], vals[3:])
                    quat = np.zeros(4, dtype=np.float64)
                    mujoco.mju_mat2Quat(quat, mat)
                    self._model.cam_quat[camera_id] = quat
                    logger.info("Updated camera {} xyaxes: {}", camera_name, vals)
                else:
                    logger.warning("Invalid {} camera xyaxes values: {}", camera_name, xyaxes_str)
            elif euler_str:
                euler_vals = [float(x) for x in euler_str.split()]
                if len(euler_vals) == 3:  # noqa: PLR2004
                    quat = np.zeros(4, dtype=np.float64)
                    mujoco.mju_euler2Quat(quat, euler_vals, "xyz")
                    self._model.cam_quat[camera_id] = quat
                    logger.info("Updated camera {} euler: {} -> quat={}", camera_name, euler_vals, quat.tolist())
                else:
                    logger.warning("Invalid {} camera euler values: {}", camera_name, euler_str)
            else:
                logger.info("No orientation attr (euler/xyaxes/quat) on {} camera", camera_name)

        update_camera_pose("overview", cam)
        wrist_cam = find_first(".//camera[@name='wrist']")
        if wrist_cam is not None:
            update_camera_pose("wrist", wrist_cam)

        mujoco.mj_forward(self._model, self._data)

    def _switch_to_scene(self, scene_id: str) -> None:
        import mujoco  # noqa: PLC0415

        from physicalai_mujoco_so101_plugin.scene_registry import get_scene  # noqa: PLC0415

        scene = get_scene(scene_id)
        xml_path = scene.scene_xml_path
        if not xml_path.exists():
            logger.error("Scene XML not found: {}", xml_path)
            return

        new_model = mujoco.MjModel.from_xml_path(str(xml_path))
        new_data = mujoco.MjData(new_model)
        mujoco.mj_forward(new_model, new_data)

        for renderer in self._camera_renderers.values():
            with contextlib.suppress(Exception):
                renderer.close()
        self._camera_renderers.clear()
        self._camera_devices.clear()

        self._model_path = str(xml_path)
        self._model = new_model
        self._data = new_data

        if self._viewer is not None and self._viewer.is_running():
            with self._viewer.lock():
                sim = self._viewer._get_sim()  # noqa: SLF001
                if sim is not None:
                    sim.m = new_model
                    sim.d = new_data

        self._free_joints = scene.free_joints
        self._target_body_name = scene.target_bodies[0] if scene.target_bodies else ""
        self._spawn_center = scene.spawn_center
        self._spawn_min_r = scene.spawn_min_r
        self._spawn_max_r = scene.spawn_max_r
        self._spawn_angle_half_deg = scene.spawn_angle_half_deg
        self._block_min_sep = scene.block_min_sep
        self._target_min_sep = scene.target_min_sep

        self._last_sim_time = None
        self._init_block_joint_addrs()
        self._init_cameras()

        self._scene_xml_mtimes = self._snapshot_scene_xml_mtimes()

        self._current_scene_id = scene_id
        from physicalai_mujoco_so101_plugin.scene_registry import get_reset_fn  # noqa: PLC0415

        self._scene_on_reset = get_reset_fn(scene_id)
        logger.info(
            "Switched to scene '{}' ({} bodies, {} geoms, {} joints)",
            scene_id,
            new_model.nbody,
            new_model.ngeom,
            new_model.njnt,
        )

    def _close_viewer(self) -> None:
        if self._viewer is not None:
            with contextlib.suppress(Exception):
                self._viewer.close()
            self._viewer = None

    def _check_pending_scene_switch(self) -> None:
        if not self._pending_scene_switch:
            return
        self._pending_scene_switch = False

        from physicalai_mujoco_so101_plugin.scene_registry import list_scenes  # noqa: PLC0415

        try:  # noqa: PLW0717
            scene_ids = list(list_scenes().keys())
            if not scene_ids:
                logger.warning("No scenes available for switching")
                return

            current = self._current_scene_id
            idx = 0 if current is None or current not in scene_ids else scene_ids.index(current)
            next_idx = (idx + 1) % len(scene_ids)
            self._switch_to_scene(scene_ids[next_idx])
        except Exception as exc:  # noqa: BLE001
            logger.warning("Failed to switch scene: {}", exc)

    def _handle_viewer_reset(self) -> None:
        current = float(self._data.time)
        if self._last_sim_time is None:
            self._last_sim_time = current
            return
        if current + 1e-9 < self._last_sim_time:
            logger.info("Viewer reset detected; randomizing")
            if self._scene_on_reset is not None:
                self._scene_on_reset(self._model, self._data, self._rng)
            else:
                self._randomize_blocks()
            if self._viewer is not None and self._viewer.is_running():
                self._viewer.sync()
            current = float(self._data.time)
        self._last_sim_time = current

    def _sample_spawn_xy(self) -> tuple[float, float]:
        r = float(self._rng.uniform(self._spawn_min_r, self._spawn_max_r))
        theta = float(
            self._rng.uniform(-np.radians(self._spawn_angle_half_deg), np.radians(self._spawn_angle_half_deg)),
        )
        return (
            self._spawn_center[0] + r * float(np.cos(theta)),
            self._spawn_center[1] + r * float(np.sin(theta)),
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
                far_from_target = np.hypot(x - target_xy[0], y - target_xy[1]) >= self._target_min_sep
                if far_from_target and all(np.hypot(x - px, y - py) >= self._block_min_sep for px, py in positions):
                    positions.append(cand)
                    break
            else:
                if best is not None:
                    positions.append(best)
        return target_xy, positions

    def _randomize_blocks(self) -> None:
        import mujoco  # noqa: PLC0415

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
            if renderer is None:
                continue

            period = 1.0 / config.fps
            if now - self._camera_last_frame_ts[config.name] < period:
                continue

            try:
                renderer.update_scene(self._data, camera=config.name)
                frame = renderer.render()[:, :, :3][::-1, :, :]
            except (RuntimeError, ValueError) as exc:
                logger.debug("Camera render error for '{}': {}", config.name, exc)
                continue

            if config.mirror_horizontal:
                frame = frame[:, ::-1, :]
            frame = np.ascontiguousarray(frame)

            buffer = self._frame_buffers.get(config.name)
            if buffer is not None:
                buffer.put(frame)

            cam = self._camera_devices.get(config.name)
            if cam is not None:
                try:
                    cam.schedule_frame(frame)
                except RuntimeError as exc:
                    logger.debug("Camera publish error for '{}': {}", config.name, exc)
                    continue

            self._camera_last_frame_ts[config.name] = now

    def _start_http_server(self) -> None:
        if self._http_port <= 0:
            return

        from physicalai_mujoco_so101_plugin.http_server import HttpServer, build_app  # noqa: PLC0415

        app = build_app(
            service_name="mujoco-so101",
            buffers=self._frame_buffers,
            commands=self._commands,
            get_status=self._http_status,
        )
        server = HttpServer(app, self._http_host, self._http_port)
        try:
            server.start()
        except (OSError, RuntimeError) as exc:
            logger.warning("Failed to start HTTP server on {}:{}: {}", self._http_host, self._http_port, exc)
            return
        self._http_server = server
        logger.info("HTTP camera server running at {}", server.url)

    def _stop_http_server(self) -> None:
        if self._http_server is None:
            return
        with contextlib.suppress(Exception):
            self._http_server.stop()
        self._http_server = None

    def _http_status(self) -> dict[str, object]:
        from physicalai_mujoco_so101_plugin.scene_registry import list_scenes  # noqa: PLC0415

        return {
            "connected": self.is_connected(),
            "scene": self._current_scene_id,
            "scenes": sorted(list_scenes()),
            "cameras": [
                {
                    "name": config.name,
                    "width": config.width,
                    "height": config.height,
                    "fps": config.fps,
                    "device": config.device,
                    "rendering": config.name in self._camera_renderers,
                }
                for config in self._cameras
            ],
        }

    def _drain_commands(self) -> None:
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            self._handle_command(command)

    def _handle_command(self, command: SimCommand) -> None:
        from physicalai_mujoco_so101_plugin.http_server import (  # noqa: PLC0415
            ResetCommand,
            ShutdownCommand,
            SwitchSceneCommand,
        )

        if isinstance(command, ResetCommand):
            logger.info("Scene reset requested via HTTP")
            if self._scene_on_reset is not None:
                self._scene_on_reset(self._model, self._data, self._rng)
            else:
                self._randomize_blocks()
        elif isinstance(command, SwitchSceneCommand):
            logger.info("Scene switch requested via HTTP: {}", command.scene_id)
            try:
                self._switch_to_scene(command.scene_id)
            except KeyError as exc:
                logger.warning("Scene switch failed: {}", exc)
        elif isinstance(command, ShutdownCommand):
            logger.info("Shutdown requested via HTTP")
            _signal_owner_shutdown()

    def get_observation(self) -> RobotObservation:
        """Return the current simulated joint observation.

        Raises:
            ConnectionError: If the robot is not connected.
        """
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)

        self._check_pending_scene_switch()
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

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:  # noqa: ARG002
        """Apply joint-angle targets to the simulation actuators.

        Raises:
            ConnectionError: If the robot is not connected.
            ValueError: If `action` does not match the robot joint count.
        """
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)

        if action.shape != (self.NUM_JOINTS,):
            msg = f"Expected action shape ({self.NUM_JOINTS},), got {action.shape}"
            raise ValueError(msg)

        for i in range(self.NUM_JOINTS):
            self._data.ctrl[i] = float(np.radians(action[i]))

    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray:
        """Render an RGB image from a named camera.

        Returns:
            The rendered RGB image.

        Raises:
            ConnectionError: If the robot is not connected.
        """
        import mujoco  # noqa: PLC0415

        if not self.is_connected():
            msg = "Robot is not connected."
            raise ConnectionError(msg)

        renderer = mujoco.Renderer(self._model, height, width)
        renderer.update_scene(self._data, camera=camera_name)
        rgb = renderer.render()
        renderer.close()
        return rgb

    def _read_joint_state(self, name: str) -> tuple[float, float]:
        import mujoco  # noqa: PLC0415

        jnt_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jnt_id < 0:
            msg = f"Joint {name!r} not found in MuJoCo model"
            raise ValueError(msg)
        qpos_adr = self._model.jnt_qposadr[jnt_id]
        dof_adr = self._model.jnt_dofadr[jnt_id]
        return float(self._data.qpos[qpos_adr]), float(self._data.qvel[dof_adr])

    def __getstate__(self) -> dict:
        """Return serializable construction state."""
        return {
            "_model_path": self._model_path,
            "_substeps": self._substeps,
            "_enable_viewer": self._enable_viewer,
            "_cameras": [asdict(cam) for cam in self._cameras],
            "_free_joints": self._free_joints,
            "_target_body_name": self._target_body_name,
            "_spawn_center": self._spawn_center,
            "_spawn_min_r": self._spawn_min_r,
            "_spawn_max_r": self._spawn_max_r,
            "_spawn_angle_half_deg": self._spawn_angle_half_deg,
            "_block_min_sep": self._block_min_sep,
            "_target_min_sep": self._target_min_sep,
            "_current_scene_id": self._current_scene_id,
            "_http_host": self._http_host,
            "_http_port": self._http_port,
        }

    def __setstate__(self, state: dict) -> None:
        """Restore serializable construction state."""
        self._model_path = state["_model_path"]
        self._substeps = state["_substeps"]
        self._enable_viewer = state.get("_enable_viewer", False)
        self._cameras = [CameraConfig(**cam) for cam in state.get("_cameras", [])]
        self._free_joints = state.get("_free_joints", self.DEFAULT_BLOCK_FREEJOINTS)
        self._target_body_name = state.get("_target_body_name", self.DEFAULT_TARGET_BODY_NAME)
        self._spawn_center = state.get("_spawn_center", self.DEFAULT_SPAWN_CENTER)
        self._spawn_min_r = state.get("_spawn_min_r", self.DEFAULT_SPAWN_MIN_R)
        self._spawn_max_r = state.get("_spawn_max_r", self.DEFAULT_SPAWN_MAX_R)
        self._spawn_angle_half_deg = state.get("_spawn_angle_half_deg", self.DEFAULT_SPAWN_ANGLE_HALF_DEG)
        self._block_min_sep = state.get("_block_min_sep", self.DEFAULT_BLOCK_MIN_SEP)
        self._target_min_sep = state.get("_target_min_sep", self.DEFAULT_TARGET_MIN_SEP)
        self._current_scene_id = state.get("_current_scene_id")
        if self._current_scene_id:
            from physicalai_mujoco_so101_plugin.scene_registry import get_reset_fn  # noqa: PLC0415

            self._scene_on_reset = get_reset_fn(self._current_scene_id)
        else:
            self._scene_on_reset = None
        self._model = None
        self._data = None
        self._viewer = None
        self._camera_devices = {}
        self._camera_renderers = {}
        self._camera_last_frame_ts = {}
        self._frame_buffers = {}
        self._commands = queue.Queue()
        self._http_host = state.get("_http_host", "127.0.0.1")
        self._http_port = state.get("_http_port", 0)
        self._http_server = None
        self._block_joint_addrs = []
        self._target_body_id = None
        self._last_sim_time = None
        self._rng = np.random.default_rng()
        self._pending_scene_switch = False
        self._scene_xml_mtimes = {}


class BiMuJoCoSO101(MuJoCoSO101):
    """Bimanual SO-101 simulated with a single MuJoCo model.

    Runs both arms in one model with ``left_*`` then ``right_*`` joints
    (12 total). ``send_action`` writes into the model actuator array, so the
    dual-arm XML must declare its actuators in ``BIMANUAL_SO101_JOINT_ORDER``.
    """

    JOINT_ORDER: ClassVar[tuple[str, ...]] = BIMANUAL_SO101_JOINT_ORDER
    NUM_JOINTS: ClassVar[int] = BIMANUAL_NUM_JOINTS
