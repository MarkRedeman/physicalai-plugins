"""Teleoperate a remote robot via WebSocket by relaying joint positions."""

import argparse
import time

import numpy as np

from physicalai.robot import connect
from physicalai_websocket_robot_plugin import WebSocketRobot


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleoperate a remote robot via WebSocket")
    parser.add_argument("url", help="WebSocket URL (e.g. ws://192.168.1.100:8765)")
    parser.add_argument("--hz", type=float, default=10.0, help="Control frequency in Hz")
    args = parser.parse_args()

    robot = WebSocketRobot(args.url)
    period = 1.0 / args.hz

    with connect(robot) as r:
        obs = r.get_observation()
        print(f"Connected. Control rate: {args.hz} Hz. Press Ctrl+C to stop.")

        try:
            while True:
                loop_start = time.monotonic()

                obs = r.get_observation()
                r.send_action(obs.joint_positions)

                elapsed = time.monotonic() - loop_start
                sleep = max(0.0, period - elapsed)
                time.sleep(sleep)

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
