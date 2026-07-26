"""Teleoperate a running MuJoCo SO-101 simulation via zenoh.

Relay joint positions from the simulation back to itself (or to another
robot) by reading state and sending the same positions as actions.

Usage:
    uv run python examples/teleoperation.py --name mujoco-so101
"""

from __future__ import annotations

import argparse
import time

from loguru import logger


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleoperate a MuJoCo SO-101 simulation via zenoh")
    parser.add_argument("--name", type=str, default="mujoco-so101", help="Zenoh robot name")
    parser.add_argument("--hz", type=float, default=30.0, help="Control frequency")
    args = parser.parse_args()

    from physicalai.robot import connect
    from physicalai.robot.transport import SharedRobot

    robot = SharedRobot.attach(name=args.name)

    with connect(robot) as r:
        logger.info("Connected to {}. Ctrl+C to stop.", args.name)
        period = 1.0 / args.hz

        try:
            while True:
                loop_start = time.monotonic()
                obs = r.get_observation()
                r.send_action(obs.joint_positions)
                elapsed = time.monotonic() - loop_start
                to_sleep = period - elapsed
                if to_sleep > 0:
                    time.sleep(to_sleep)
        except KeyboardInterrupt:
            logger.info("Stopped.")


if __name__ == "__main__":
    main()
