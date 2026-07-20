"""Relay joint positions from a leader arm to a follower arm in real time.

Connects to a ReBotArm102Leader (FashionStar UART) and a B601-DM or B601-RS
follower, then copies observed joint positions to the follower in a loop.

Usage examples::

    # Leader → B601-DM
    uv run python examples/teleoperation.py \\
        --leader-port /dev/ttyUSB0 \\
        --follower-type dm \\
        --follower-port /dev/ttyACM0 \\
        --follower-can-adapter damiao

    # Leader → B601-RS
    uv run python examples/teleoperation.py \\
        --leader-port /dev/ttyUSB0 \\
        --follower-type rs \\
        --follower-port can0

Press Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import numpy as np
from physicalai.robot import connect


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Relay a leader arm positions to a follower arm."
    )
    parser.add_argument(
        "--leader-port",
        default="/dev/ttyUSB0",
        help="Leader arm serial port (default /dev/ttyUSB0).",
    )
    parser.add_argument(
        "--leader-baudrate",
        type=int,
        default=1_000_000,
        help="Leader arm UART baud rate (default 1_000_000).",
    )
    parser.add_argument(
        "--follower-type",
        choices=["dm", "rs"],
        required=True,
        help="Follower robot type.",
    )
    parser.add_argument(
        "--follower-port",
        required=True,
        help="Follower serial port or CAN channel.",
    )
    parser.add_argument(
        "--follower-can-adapter",
        default=None,
        help='Follower CAN adapter ("damiao"/"socketcan"). Defaults to "damiao" for dm, "socketcan" for rs.',
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Teleoperation loop rate in Hz (default 30).",
    )

    args = parser.parse_args()

    if args.follower_can_adapter is None:
        args.follower_can_adapter = {"dm": "damiao", "rs": "socketcan"}[args.follower_type]

    from physicalai_rebot_b601_plugin.leader import ReBotArm102Leader

    leader = ReBotArm102Leader(port=args.leader_port, baudrate=args.leader_baudrate)

    if args.follower_type == "dm":
        from physicalai_rebot_b601_plugin.dm import ReBotB601DM

        follower = ReBotB601DM(port=args.follower_port, can_adapter=args.follower_can_adapter)
    else:
        from physicalai_rebot_b601_plugin.rs import ReBotB601RS

        follower = ReBotB601RS(port=args.follower_port, can_adapter=args.follower_can_adapter)

    running = True

    def _signal_handler(signum: int, frame: object | None) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    with connect(leader) as l, connect(follower) as f:
        print(f"Leader joint order:   {l.joint_names}", file=sys.stderr)
        print(f"Follower joint order: {f.joint_names}", file=sys.stderr)
        print(f"Teleoperating at {args.rate} Hz. Press Ctrl+C to stop.", file=sys.stderr)

        period = 1.0 / args.rate

        while running:
            loop_start = time.monotonic()

            obs = l.get_observation()
            follower_action = obs.joint_positions.copy()
            f.send_action(follower_action)

            pos_str = "  ".join(f"{v:7.2f}" for v in obs.joint_positions)
            print(f"[{obs.timestamp:13.3f}]  {pos_str}")

            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    main()
