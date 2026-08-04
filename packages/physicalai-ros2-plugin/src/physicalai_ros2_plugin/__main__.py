"""Start a ROS 2 robot as a PhysicalAI SharedRobot owner."""

from __future__ import annotations

import argparse
import signal
import threading

from physicalai.config import to_config
from physicalai.robot.transport import SharedRobot

from physicalai_ros2_plugin.robot import ROS2Robot


def main() -> None:
    """Parse owner settings and expose a ROS 2 robot over SharedRobot."""
    parser = argparse.ArgumentParser(description="Expose a ROS 2 robot as a PhysicalAI SharedRobot owner")
    parser.add_argument("--name", required=True, help="SharedRobot name")
    parser.add_argument("--joint-names", required=True, nargs="+", help="Ordered ROS joint names")
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--command-topic", default="/joint_trajectory_controller/joint_trajectory")
    parser.add_argument("--allow-remote", action="store_true")
    args = parser.parse_args()
    robot = ROS2Robot(args.joint_names, state_topic=args.state_topic, command_topic=args.command_topic)
    shared = SharedRobot.from_config(to_config(robot), name=args.name, allow_remote=args.allow_remote)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    shared.connect()
    try:
        stop.wait()
    finally:
        shared.disconnect()


if __name__ == "__main__":
    main()
