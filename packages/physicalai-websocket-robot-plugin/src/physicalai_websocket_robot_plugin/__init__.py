from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from physicalai_websocket_robot_plugin.websocket_robot import WebSocketRobot as WebSocketRobot
    from physicalai_websocket_robot_plugin.websocket_robot import WebSocketRobotObservation as WebSocketRobotObservation

__all__ = [
    "WebSocketRobot",
    "WebSocketRobotObservation",
]


def __getattr__(name: str) -> object:
    if name == "WebSocketRobot":
        from physicalai_websocket_robot_plugin.websocket_robot import WebSocketRobot

        return WebSocketRobot
    if name == "WebSocketRobotObservation":
        from physicalai_websocket_robot_plugin.websocket_robot import WebSocketRobotObservation

        return WebSocketRobotObservation
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
