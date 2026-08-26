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
from dataclasses import asdict
from pathlib import Path

from loguru import logger
from physicalai.config import to_config
from physicalai.robot.transport import SharedRobot

from physicalai_mujoco_so101_plugin.mujoco_robot import BiMuJoCoSO101, MuJoCoSO101


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
        "--bimanual",
        action="store_true",
        default=False,
        help="Run the dual-arm SO101 (bimanual) model (default scene: garment_fold)",
    )
    start.add_argument(
        "--scene",
        type=str,
        default=None,
        help="Scene name (default: garment_fold for bimanual, single_pick_place otherwise)",
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
        default=None,
        help=(
            "Seconds with zero subscribers before self-exit "
            "(default: 10 without HTTP, disabled with HTTP so stream viewers keep the sim alive)"
        ),
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
        help="Disable camera rendering entirely (HTTP streams and v4l2loopback)",
    )
    start.add_argument(
        "--http-host",
        type=str,
        default="127.0.0.1",
        help="Host for the camera/control HTTP server (default: 127.0.0.1)",
    )
    start.add_argument(
        "--http-port",
        type=int,
        default=8080,
        help="Port for the camera/control HTTP server (default: 8080)",
    )
    start.add_argument(
        "--no-http",
        action="store_true",
        default=False,
        help="Disable the camera/control HTTP server",
    )
    start.add_argument(
        "--v4l2",
        action="store_true",
        default=False,
        help="Also publish cameras to v4l2loopback devices (requires modprobe v4l2loopback)",
    )
    start.add_argument(
        "--wrist-video-id",
        type=int,
        default=60,
        help="v4l2loopback video ID for the wrist camera (only with --v4l2, default: 60)",
    )
    start.add_argument(
        "--right-wrist-video-id",
        type=int,
        default=61,
        help="v4l2loopback video ID for the right wrist camera (bimanual, only with --v4l2, default: 61)",
    )
    start.add_argument(
        "--overview-video-id",
        type=int,
        default=62,
        help="v4l2loopback video ID for the overview camera (only with --v4l2, default: 62)",
    )
    return parser


def _resolve_model_and_scene(
    model_arg: str | None,
    scene_arg: str | None,
    *,
    bimanual: bool,
) -> tuple[str, object | None]:
    if model_arg is not None:
        path = Path(model_arg).resolve()
        if not path.exists():
            logger.error("Model file not found: {}", path)
            sys.exit(1)
        return str(path), None

    from physicalai_mujoco_so101_plugin.scene_registry import get_scene  # noqa: PLC0415

    scene_id = scene_arg or ("garment_fold" if bimanual else "single_pick_place")
    scene = get_scene(scene_id)
    xml_path = scene.scene_xml_path
    if not xml_path.exists():
        logger.error("Scene XML not found: {}", xml_path)
        sys.exit(1)
    return str(xml_path), scene


def _start(args: argparse.Namespace) -> None:
    model_path, scene_config = _resolve_model_and_scene(args.model, args.scene, bimanual=args.bimanual)

    http_enabled = not args.no_http and args.http_port > 0

    cameras: list[dict[str, object]] = []
    if not args.no_cameras:
        if args.v4l2:
            video_ids = [args.wrist_video_id, args.overview_video_id]
            if args.bimanual:
                video_ids.append(args.right_wrist_video_id)
            if any(v < 0 for v in video_ids):
                msg = "Video IDs must be non-negative integers"
                raise ValueError(msg)
            if len(set(video_ids)) != len(video_ids):
                msg = "Wrist and overview video IDs must be different"
                raise ValueError(msg)

        def _device(video_id: int) -> str | None:
            return f"/dev/video{video_id}" if args.v4l2 else None

        left_wrist_name = "left_wrist" if args.bimanual else "wrist"
        cameras = [
            {
                "name": left_wrist_name,
                "device": _device(args.wrist_video_id),
                "width": 640,
                "height": 480,
                "fps": 30,
                "mirror_horizontal": True,
            },
            {
                "name": "overview",
                "device": _device(args.overview_video_id),
                "width": 640,
                "height": 480,
                "fps": 30,
                "mirror_horizontal": True,
            },
        ]
        if args.bimanual:
            cameras.append(
                {
                    "name": "right_wrist",
                    "device": _device(args.right_wrist_video_id),
                    "width": 640,
                    "height": 480,
                    "fps": 30,
                    "mirror_horizontal": True,
                },
            )

    robot_kwargs = {
        "model_path": model_path,
        "substeps": args.substeps,
        "enable_viewer": not args.no_gui,
        "cameras": cameras,
        "http_host": args.http_host,
        "http_port": args.http_port if http_enabled else 0,
    }
    if scene_config is not None:
        robot_kwargs["scene_config"] = asdict(scene_config)

    idle_timeout = args.idle_timeout
    if idle_timeout is None and not http_enabled:
        idle_timeout = 10.0

    robot_cls = BiMuJoCoSO101 if args.bimanual else MuJoCoSO101
    robot = SharedRobot.from_config(
        to_config(robot_cls(**robot_kwargs)),
        name=args.name,
        allow_remote=args.allow_remote,
        rate_hz=args.rate_hz,
        idle_timeout=idle_timeout,
    )

    logger.info("Connecting MuJoCo SO-101 as zenoh owner '{}' ...", args.name)
    robot.connect()
    logger.info(
        "MuJoCo SO-101 running (model={}, rate={} Hz, substeps={}, bimanual={})",
        model_path,
        args.rate_hz,
        args.substeps,
        args.bimanual,
    )
    if http_enabled:
        logger.info("Camera/control HTTP server: http://{}:{}", args.http_host, args.http_port)

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
        if http_enabled:
            _stop_owner_over_http(args.http_host, args.http_port)
        robot.disconnect()
        logger.info("MuJoCo SO-101 stopped")


def _stop_owner_over_http(host: str, port: int) -> None:
    """Ask the detached owner to exit via its HTTP control endpoint.

    The owner subprocess is detached from the CLI, so interrupting the CLI
    does not stop it; when the HTTP server is enabled it runs with idle exit
    disabled. Issue ``POST /shutdown`` so the simulation shuts down cleanly.
    """
    import urllib.error  # noqa: PLC0415
    import urllib.request  # noqa: PLC0415

    try:
        # host/port are local CLI args, scheme hardcoded to http
        req = urllib.request.Request(f"http://{host}:{port}/shutdown", method="POST")
        with urllib.request.urlopen(req, timeout=5):  # nosec B310  # noqa: S310
            logger.info("Owner shutdown requested via HTTP")
    except (OSError, urllib.error.URLError):
        logger.debug("HTTP shutdown endpoint unavailable; owner will self-manage")


def main() -> None:
    """Parse command-line arguments and run the requested command."""
    parser = _build_parser()
    args = parser.parse_args()

    if args.command == "start":
        _start(args)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
