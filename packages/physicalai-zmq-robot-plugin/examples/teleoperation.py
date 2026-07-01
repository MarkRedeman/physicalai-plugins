"""Teleoperate a remote robot via ZMQ by relaying joint positions."""

import argparse
import time

from physicalai.robot import connect
from physicalai_zmq_robot_plugin import ZMQRobot


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleoperate a remote robot via ZMQ")
    parser.add_argument("endpoint", help="ZMQ endpoint (e.g. tcp://192.168.1.100:5555)")
    parser.add_argument("--hz", type=float, default=10.0, help="Control frequency in Hz")
    args = parser.parse_args()

    robot = ZMQRobot(args.endpoint)
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
