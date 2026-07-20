from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobot as ZMQRobot
    from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobotObservation as ZMQRobotObservation

__all__ = [
    "ZMQRobot",
    "ZMQRobotObservation",
]


def __getattr__(name: str) -> object:
    if name == "ZMQRobot":
        from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobot

        return ZMQRobot
    if name == "ZMQRobotObservation":
        from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobotObservation

        return ZMQRobotObservation
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
