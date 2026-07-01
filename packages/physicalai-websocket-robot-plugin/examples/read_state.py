"""Read and print robot state from a remote WebSocket robot."""

import argparse

import numpy as np

from physicalai.robot import connect
from physicalai_websocket_robot_plugin import WebSocketRobot


def main() -> None:
    parser = argparse.ArgumentParser(description="Read joints from a WebSocket robot")
    parser.add_argument("url", help="WebSocket URL (e.g. ws://192.168.1.100:8765)")
    args = parser.parse_args()

    robot = WebSocketRobot(args.url)

    with connect(robot) as r:
        obs = r.get_observation()
        print(f"Joint positions: {obs.joint_positions}")
        print(f"Joint names: {r.joint_names}")


if __name__ == "__main__":
    main()
