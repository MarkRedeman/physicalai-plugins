"""Keyboard-driven base control for LeKiwi using WASD.

Controls the 3-wheel holonomic base via single-key input::

    W  — forward
    S  — backward
    A  — rotate left (counter-clockwise)
    D  — rotate right (clockwise)
    Q  — strafe left
    E  — strafe right
    X / Enter — stop and exit

Usage::

    uv run python examples/wasd_drive.py

The arm joints remain idle at their current position — only the base moves.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
from physicalai.robot import connect


def main() -> None:
    parser = argparse.ArgumentParser(description="WASD keyboard drive for the LeKiwi base.")
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
        "--vx",
        type=float,
        default=0.15,
        help="Forward/back linear velocity in m/s (default 0.15).",
    )
    parser.add_argument(
        "--vy",
        type=float,
        default=0.10,
        help="Strafe linear velocity in m/s (default 0.10).",
    )
    parser.add_argument(
        "--vtheta",
        type=float,
        default=0.5,
        help="Rotational velocity in rad/s (default 0.5).",
    )

    args = parser.parse_args()

    from physicalai_lekiwi_plugin import LeKiwi

    robot = LeKiwi.uncalibrated(port=args.port, baudrate=args.baudrate)

    print(f"Connecting to LeKiwi on {args.port} ...", file=sys.stderr)

    with connect(robot) as robot:
        print("Connected. Joint order: " + ", ".join(robot.joint_names), file=sys.stderr)
        print(file=sys.stderr)
        print("WASD drive controls:", file=sys.stderr)
        print("  W  — forward", file=sys.stderr)
        print("  S  — backward", file=sys.stderr)
        print("  A  — rotate left", file=sys.stderr)
        print("  D  — rotate right", file=sys.stderr)
        print("  Q  — strafe left", file=sys.stderr)
        print("  E  — strafe right", file=sys.stderr)
        print("  X  — stop and exit", file=sys.stderr)
        print(file=sys.stderr)

        try:
            while True:
                key = input("cmd> ").strip().lower()

                vx = vy = vtheta = 0.0
                if key == "w":
                    vx = args.vx
                elif key == "s":
                    vx = -args.vx
                elif key == "a":
                    vtheta = args.vtheta
                elif key == "d":
                    vtheta = -args.vtheta
                elif key == "q":
                    vy = args.vy
                elif key == "e":
                    vy = -args.vy
                elif key in ("", "x"):
                    break
                else:
                    print(f"  Unknown key: {key!r}", file=sys.stderr)
                    continue

                action = np.zeros(9, dtype=np.float32)
                action[6] = vx
                action[7] = vy
                action[8] = vtheta
                robot.send_action(action)

                status = f"  vx={vx:+.2f}  vy={vy:+.2f}  vtheta={vtheta:+.2f}"
                print(status, file=sys.stderr)

        except KeyboardInterrupt:
            pass

        robot.send_action(np.zeros(9, dtype=np.float32))
        print("Stopped.", file=sys.stderr)


if __name__ == "__main__":
    main()
