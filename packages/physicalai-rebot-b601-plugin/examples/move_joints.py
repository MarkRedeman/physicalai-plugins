"""Send smooth sinusoidal position targets to a reBot arm.

This is a basic actuation smoke test. It moves each joint through a small
sine wave for a fixed duration. Use with the arm suspended or in a safe
configuration — the arm will move!

Usage examples::

    # B601-DM
    uv run python examples/move_joints.py --robot dm --port /dev/ttyACM0 --can-adapter damiao --duration 10

    # B601-RS
    uv run python examples/move_joints.py --robot rs --port can0 --duration 10

    # Arm 102 leader (will raise RuntimeError — read-only)
    uv run python examples/move_joints.py --robot leader --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

import numpy as np
from physicalai.robot import connect


def _build_robot(args: argparse.Namespace) -> object:
    robot_type = args.robot
    if robot_type == "dm":
        from physicalai_rebot_b601_plugin.dm import ReBotB601DM

        return ReBotB601DM(
            port=args.port,
            can_adapter=args.can_adapter,
        )
    if robot_type == "rs":
        from physicalai_rebot_b601_plugin.rs import ReBotB601RS

        return ReBotB601RS(
            port=args.port,
            can_adapter=args.can_adapter,
        )
    if robot_type == "leader":
        from physicalai_rebot_b601_plugin.leader import ReBotArm102Leader

        return ReBotArm102Leader(
            port=args.port,
            baudrate=args.baudrate,
        )

    msg = f"Unknown robot type: {robot_type!r}"
    raise ValueError(msg)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send sinusoidal position targets to a reBot arm (actuation smoke test)."
    )
    parser.add_argument(
        "--robot",
        choices=["dm", "rs", "leader"],
        required=True,
        help="Robot arm type.",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port or CAN channel.",
    )
    parser.add_argument(
        "--can-adapter",
        default=None,
        help='CAN adapter ("damiao"/"socketcan") for DM or RS.',
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=1_000_000,
        help="UART baud rate for leader arm (default 1_000_000).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Duration in seconds (default 5).",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=10.0,
        help="Sine-wave amplitude in degrees (default 10).",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.25,
        help="Sine-wave frequency in Hz (default 0.25).",
    )

    args = parser.parse_args()

    if args.port is None:
        args.port = {"dm": "/dev/ttyACM0", "rs": "can0", "leader": "/dev/ttyUSB0"}[args.robot]
    if args.can_adapter is None:
        args.can_adapter = {"dm": "damiao", "rs": "socketcan"}.get(args.robot, "")

    robot = _build_robot(args)

    running = True

    def _signal_handler(signum: int, frame: object | None) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    print(f"Connecting to {args.robot} on {args.port} ...", file=sys.stderr)

    with connect(robot) as arm:
        num_joints = len(arm.joint_names)
        print(f"Connected. Joint order: {arm.joint_names}", file=sys.stderr)
        print(f"Running for {args.duration}s, amplitude={args.amplitude}°, freq={args.frequency}Hz", file=sys.stderr)

        start = time.monotonic()
        phase_offsets = [i * 2.0 * math.pi / num_joints for i in range(num_joints)]

        while running:
            t = time.monotonic() - start
            if t > args.duration:
                break

            action = np.array([
                args.amplitude * math.sin(2.0 * math.pi * args.frequency * t + phase)
                for phase in phase_offsets
            ], dtype=np.float32)

            obs = arm.get_observation()
            arm.send_action(action)

            pos_str = "  ".join(f"{v:8.2f}" for v in obs.joint_positions)
            cmd_str = "  ".join(f"{v:8.2f}" for v in action)
            print(f"[{t:6.2f}s]  pos: {pos_str}  |  cmd: {cmd_str}")

            time.sleep(0.05)

    print("Done.", file=sys.stderr)


if __name__ == "__main__":
    main()
