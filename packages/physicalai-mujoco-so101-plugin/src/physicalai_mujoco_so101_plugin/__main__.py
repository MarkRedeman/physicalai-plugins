# ruff: file-ignore[undocumented-public-function, import-outside-top-level]
"""CLI entrypoint for the MuJoCo SO-101 simulation.

Usage:

    physicalai-mujoco-so101 start --model <path> [options]

Start a MuJoCo SO-101 simulation as a zenoh robot owner, making it
discoverable and controllable from PhysicalAI Studio.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
from pathlib import Path

from loguru import logger


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="physicalai-mujoco-so101",
        description="MuJoCo SO-101 simulation for PhysicalAI Studio",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    start = sub.add_parser("start", help="Start the MuJoCo simulation as a zenoh robot owner")
    start.add_argument(
        "--model",
        type=str,
        default=None,
        help="Path to MuJoCo model XML or URDF (default: bundled SO101 model)",
    )
    start.add_argument(
        "--name",
        type=str,
        default="mujoco-so101",
        help="Zenoh robot name (default: mujoco-so101)",
    )
    start.add_argument(
        "--rate-hz",
        type=float,
        default=50.0,
        help="Owner control loop rate in Hz (default: 50)",
    )
    start.add_argument(
        "--substeps",
        type=int,
        default=10,
        help="MuJoCo simulation steps per control cycle (default: 10, real-time at 50 Hz with dt=0.002)",
    )
    start.add_argument(
        "--scene",
        type=str,
        default="pick_lift",
        help="Scene name (default: pick_lift)",
    )
    start.add_argument(
        "--allow-remote",
        action="store_true",
        default=False,
        help="Allow remote zenoh connections (default: loopback only)",
    )
    start.add_argument(
        "--idle-timeout",
        type=float,
        default=10.0,
        help="Seconds with zero subscribers before self-exit (default: 10)",
    )
    start.add_argument(
        "--no-gui",
        action="store_true",
        default=False,
        help="Disable MuJoCo interactive viewer window",
    )
    start.add_argument(
        "--no-cameras",
        action="store_true",
        default=False,
        help="Disable v4l2loopback camera output",
    )
    start.add_argument(
        "--wrist-video-id",
        type=int,
        default=60,
        help="v4l2loopback video ID for the wrist camera (default: 60)",
    )
    start.add_argument(
        "--overview-video-id",
        type=int,
        default=61,
        help="v4l2loopback video ID for the overview camera (default: 61)",
    )
    return parser


def _resolve_model_and_scene(model_arg: str | None, scene_arg: str) -> tuple[str, object | None]:
    if model_arg is not None:
        path = Path(model_arg).resolve()
        if not path.exists():
            logger.error("Model file not found: {}", path)
            sys.exit(1)
        return str(path), None

    from physicalai_mujoco_so101_plugin.scene_registry import get_scene

    scene = get_scene(scene_arg)
    xml_path = scene.scene_xml_path
    if not xml_path.exists():
        logger.error("Scene XML not found: {}", xml_path)
        sys.exit(1)
    return str(xml_path), scene


def _start(args: argparse.Namespace) -> None:
    model_path, scene_config = _resolve_model_and_scene(args.model, args.scene)

    cameras: list[dict[str, object]] = []
    if not args.no_cameras:
        if args.wrist_video_id < 0 or args.overview_video_id < 0:
            msg = "Video IDs must be non-negative integers"
            raise ValueError(msg)
        if args.wrist_video_id == args.overview_video_id:
            msg = "Wrist and overview video IDs must be different"
            raise ValueError(msg)
        wrist_device = f"/dev/video{args.wrist_video_id}"
        overview_device = f"/dev/video{args.overview_video_id}"
        cameras = [
            {
                "name": "wrist",
                "device": wrist_device,
                "width": 640,
                "height": 480,
                "fps": 30,
                "mirror_horizontal": True,
            },
            {
                "name": "overview",
                "device": overview_device,
                "width": 640,
                "height": 480,
                "fps": 30,
                "mirror_horizontal": True,
            },
        ]

    robot_kwargs = {
        "model_path": model_path,
        "substeps": args.substeps,
        "enable_viewer": not args.no_gui,
        "cameras": cameras,
    }
    if scene_config is not None:
        robot_kwargs["scene_config"] = scene_config

    from physicalai.robot.transport import SharedRobot

    robot = SharedRobot(
        name=args.name,
        robot_class="physicalai_mujoco_so101_plugin.mujoco_robot.MuJoCoSO101",
        robot_kwargs=robot_kwargs,
        allow_remote=args.allow_remote,
        rate_hz=args.rate_hz,
        idle_timeout=args.idle_timeout,
    )

    logger.info("Connecting MuJoCo SO-101 as zenoh owner '{}' ...", args.name)
    robot.connect()
    logger.info(
        "MuJoCo SO-101 running (model={}, rate={} Hz, substeps={})",
        model_path,
        args.rate_hz,
        args.substeps,
    )

    shutdown = threading.Event()

    def _signal_handler(signum: int, frame: object) -> None:
        _ = frame, signum
        logger.info("Shutdown requested")
        shutdown.set()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        shutdown.wait()
    except KeyboardInterrupt:
        pass
    finally:
        robot.disconnect()
        logger.info("MuJoCo SO-101 stopped")


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    if args.command == "start":
        _start(args)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
