"""Connect to a reBot arm and print joint positions in a loop.

Usage examples::

    # B601-DM
    uv run python examples/read_joints.py --robot dm --port /dev/ttyACM0 --can-adapter damiao

    # B601-RS
    uv run python examples/read_joints.py --robot rs --port can0

    # Arm 102 leader
    uv run python examples/read_joints.py --robot leader --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

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
    parser = argparse.ArgumentParser(description="Read joint positions from a reBot arm.")
    parser.add_argument(
        "--robot",
        choices=["dm", "rs", "leader"],
        required=True,
        help="Robot arm type.",
    )
    parser.add_argument(
        "--port",
        default=None,
        help='Serial port or CAN channel (default depends on robot type, e.g. /dev/ttyACM0 for dm).',
    )
    parser.add_argument(
        "--can-adapter",
        default=None,
        help='CAN adapter for DM ("damiao"/"socketcan", default "damiao") or RS ("socketcan", default).',
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=1_000_000,
        help="UART baud rate for leader arm (default 1_000_000).",
    )
    parser.add_argument(
        "--num-readings",
        type=int,
        default=50,
        help="Number of readings to take before exiting (default 50, 0 = infinite).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Seconds between readings (default 0.1).",
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
        print(f"Connected. Joint order: {arm.joint_names}", file=sys.stderr)
        readings = 0
        while running:
            obs = arm.get_observation()
            pos_str = "  ".join(f"{v:8.2f}" for v in obs.joint_positions)
            print(f"[{obs.timestamp:13.3f}]  {pos_str}")
            readings += 1
            if args.num_readings and readings >= args.num_readings:
                break
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
