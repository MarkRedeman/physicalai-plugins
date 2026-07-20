"""Connect to a LeKiwi robot and print joint positions in a loop.

For calibrated mode::

    uv run python examples/read_joints.py --calibration /path/to/calibration.json

For uncalibrated (ticks) mode::

    uv run python examples/read_joints.py --uncalibrated
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from physicalai.robot import connect


def main() -> None:
    parser = argparse.ArgumentParser(description="Read joint positions from a LeKiwi robot.")
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port (default /dev/ttyACM0).",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=1_000_000,
        help="Serial baudrate (default 1_000_000).",
    )
    parser.add_argument(
        "--calibration",
        default=None,
        help="Path to calibration JSON file (required for normalized mode).",
    )
    parser.add_argument(
        "--uncalibrated",
        action="store_true",
        help="Use uncalibrated (raw ticks) mode.",
    )
    parser.add_argument(
        "--num-readings",
        type=int,
        default=50,
        help="Number of readings (default 50, 0 = infinite).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Seconds between readings (default 0.1).",
    )

    args = parser.parse_args()

    from physicalai_lekiwi_plugin import LeKiwi, LeKiwiCalibration

    if args.uncalibrated:
        robot = LeKiwi.uncalibrated(port=args.port, baudrate=args.baudrate)
    else:
        if args.calibration is None:
            parser.error("--calibration is required unless --uncalibrated is set")
        cal = LeKiwiCalibration.from_path(args.calibration)
        robot = LeKiwi(port=args.port, baudrate=args.baudrate, calibration=cal)

    running = True

    def _signal_handler(signum: int, frame: object | None) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    print(f"Connecting to LeKiwi on {args.port} ...", file=sys.stderr)
    with connect(robot) as robot:
        print(f"Connected. Joint order: {robot.joint_names}", file=sys.stderr)
        readings = 0
        while running:
            obs = robot.get_observation()
            arm_str = "  ".join(f"{v:8.2f}" for v in obs.joint_positions[:6])
            base_str = "  ".join(f"{v:8.2f}" for v in obs.joint_positions[6:])
            print(f"[{obs.timestamp:13.3f}]  arm: {arm_str}  |  base: {base_str}")
            readings += 1
            if args.num_readings and readings >= args.num_readings:
                break
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
