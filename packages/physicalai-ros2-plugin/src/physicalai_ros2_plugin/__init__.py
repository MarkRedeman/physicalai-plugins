"""Generic ROS 2 robot adapter for PhysicalAI."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from physicalai_ros2_plugin.robot import ROS2Observation as ROS2Observation
    from physicalai_ros2_plugin.robot import ROS2Robot as ROS2Robot

__all__ = ["ROS2Observation", "ROS2Robot"]


def __getattr__(name: str) -> object:
    if name == "ROS2Observation":
        from physicalai_ros2_plugin.robot import ROS2Observation

        return ROS2Observation
    if name == "ROS2Robot":
        from physicalai_ros2_plugin.robot import ROS2Robot

        return ROS2Robot
    msg = f"module {__name__!r} has no attribute {name!r}"
    raise AttributeError(msg)
