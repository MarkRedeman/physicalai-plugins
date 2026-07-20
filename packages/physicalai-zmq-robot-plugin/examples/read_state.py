"""Read and print robot state from a remote ZMQ robot."""

import argparse

from physicalai.robot import connect
from physicalai_zmq_robot_plugin import ZMQRobot


def main() -> None:
    parser = argparse.ArgumentParser(description="Read joints from a ZMQ robot")
    parser.add_argument("endpoint", help="ZMQ endpoint (e.g. tcp://192.168.1.100:5555)")
    args = parser.parse_args()

    robot = ZMQRobot(args.endpoint)

    with connect(robot) as r:
        obs = r.get_observation()
        print(f"Joint positions: {obs.joint_positions}")
        print(f"Joint names: {r.joint_names}")


if __name__ == "__main__":
    main()
