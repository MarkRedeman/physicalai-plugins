"""Send smooth sinusoidal position targets to the LeKiwi arm and base.

Moves the 6 arm joints through a small sine wave while gently oscillating the
base. Use with the robot suspended or on a stand — the arm will move and the
base will spin in place!

Usage::

    uv run python examples/move_joints.py --duration 10
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

import numpy as np
from physicalai.robot import connect


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send sinusoidal targets to a LeKiwi robot (actuation smoke test)."
    )
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
        "--duration",
        type=float,
        default=5.0,
        help="Duration in seconds (default 5).",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=10.0,
        help="Sine-wave amplitude for arm joints in degrees (default 10).",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.25,
        help="Sine-wave frequency in Hz (default 0.25).",
    )
    parser.add_argument(
        "--base-amplitude",
        type=float,
        default=15.0,
        help="Sine-wave amplitude for base rotation in deg/s (default 15).",
    )

    args = parser.parse_args()

    from physicalai_lekiwi_plugin import LeKiwi

    robot = LeKiwi.uncalibrated(port=args.port, baudrate=args.baudrate)

    running = True

    def _signal_handler(signum: int, frame: object | None) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    print(f"Connecting to LeKiwi on {args.port} ...", file=sys.stderr)

    with connect(robot) as robot:
        num_joints = len(robot.joint_names)
        print(f"Connected. Joint order: {robot.joint_names}", file=sys.stderr)
        print(
            f"Running for {args.duration}s, arm amplitude={args.amplitude} deg, "
            f"base amplitude={args.base_amplitude} deg/s",
            file=sys.stderr,
        )

        start = time.monotonic()

        while running:
            t = time.monotonic() - start
            if t > args.duration:
                break

            action = np.zeros(num_joints, dtype=np.float32)
            for i in range(6):
                action[i] = args.amplitude * math.sin(
                    2.0 * math.pi * args.frequency * t + i * 2.0 * math.pi / 6
                )

            theta_vel = args.base_amplitude * math.sin(
                2.0 * math.pi * args.frequency * t
            )
            action[6:] = [0.0, 0.0, theta_vel]

            robot.send_action(action)

            obs = robot.get_observation()
            arm_str = "  ".join(f"{v:8.2f}" for v in obs.joint_positions)
            print(f"[{t:6.2f}s]  joints: {arm_str}")

            time.sleep(0.05)

    robot.send_action(np.zeros(num_joints, dtype=np.float32))
    print("Done.", file=sys.stderr)


if __name__ == "__main__":
    main()
