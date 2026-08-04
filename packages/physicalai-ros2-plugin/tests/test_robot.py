from __future__ import annotations

import sys
import types
from dataclasses import dataclass, field
from unittest.mock import MagicMock

import numpy as np
import pytest

from physicalai_ros2_plugin.robot import ROS2Robot


@dataclass
class _JointState:
    name: list[str]
    position: list[float]
    velocity: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)


class _Duration:
    sec: int = 0
    nanosec: int = 0


class _Point:
    def __init__(self) -> None:
        self.positions: list[float] = []
        self.time_from_start = _Duration()


class _Trajectory:
    def __init__(self) -> None:
        self.joint_names: list[str] = []
        self.points: list[_Point] = []


class _Node:
    def __init__(self) -> None:
        self.publisher = MagicMock()

    def create_publisher(self, *_: object) -> MagicMock:
        return self.publisher

    def create_subscription(self, *_: object) -> MagicMock:
        return MagicMock()

    def destroy_node(self) -> None:
        return None


class _Executor:
    def add_node(self, _: object) -> None:
        return None

    def spin_once(self, timeout_sec: float) -> None:
        return None

    def shutdown(self) -> None:
        return None


@pytest.fixture
def mock_ros(monkeypatch: pytest.MonkeyPatch) -> types.SimpleNamespace:
    rclpy = types.ModuleType("rclpy")
    rclpy.ok = MagicMock(return_value=True)
    rclpy.create_node = MagicMock(return_value=_Node())
    rclpy.shutdown = MagicMock()
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.SingleThreadedExecutor = _Executor
    sensor_msgs = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.JointState = _JointState
    sensor_msgs.Image = object
    trajectory_msgs = types.ModuleType("trajectory_msgs.msg")
    trajectory_msgs.JointTrajectory = _Trajectory
    trajectory_msgs.JointTrajectoryPoint = _Point
    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Float64MultiArray = object
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.executors", rclpy_executors)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs)
    monkeypatch.setitem(sys.modules, "trajectory_msgs.msg", trajectory_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs)
    return types.SimpleNamespace(rclpy=rclpy)


class TestROS2Robot:
    def test_rejects_invalid_configuration(self) -> None:
        with pytest.raises(ValueError, match="unique"):
            ROS2Robot(["joint", "joint"])
        with pytest.raises(ValueError, match="Only"):
            ROS2Robot(["joint"], state_message_type="custom")

    def test_converts_joint_state_and_extensions(self) -> None:
        robot = ROS2Robot(["joint_2", "joint_1"], angle_unit="degrees")
        robot._on_joint_state(_JointState(["joint_1", "joint_2"], [0.0, np.pi / 2], [1.0, 2.0], [3.0, 4.0]))
        robot._on_sensor("temperature", types.SimpleNamespace(data=[25.0]))

        observation = robot.get_observation()

        np.testing.assert_allclose(observation.joint_positions, [90.0, 0.0])
        assert observation.sensor_data is not None
        np.testing.assert_allclose(observation.sensor_data["velocities"], [2.0, 1.0])
        np.testing.assert_allclose(observation.sensor_data["effort"], [4.0, 3.0])
        np.testing.assert_allclose(observation.sensor_data["temperature"], [25.0])

    def test_ignores_incomplete_joint_state(self) -> None:
        robot = ROS2Robot(["joint_1", "joint_2"])
        robot._on_joint_state(_JointState(["joint_1"], [0.0]))
        with pytest.raises(ConnectionError, match="No complete"):
            robot.get_observation()

    def test_publishes_radian_trajectory(self, mock_ros: types.SimpleNamespace) -> None:
        robot = ROS2Robot(["joint_1"], angle_unit="degrees")
        robot._on_joint_state(_JointState(["joint_1"], [0.0]))
        publisher = MagicMock()
        robot._node = object()
        robot._publisher = publisher

        robot.send_action(np.array([180.0], dtype=np.float32), goal_time=0.2)

        message = publisher.publish.call_args.args[0]
        assert message.joint_names == ["joint_1"]
        np.testing.assert_allclose(message.points[0].positions, [np.pi])
        assert message.points[0].time_from_start.sec == 0
        assert message.points[0].time_from_start.nanosec == 200_000_000

    def test_rejects_stale_state(self, mock_ros: types.SimpleNamespace) -> None:
        robot = ROS2Robot(["joint_1"], command_timeout=0.001)
        robot._on_joint_state(_JointState(["joint_1"], [0.0]))
        robot._state_timestamp = 0.0
        robot._node = object()
        robot._publisher = MagicMock()

        with pytest.raises(ConnectionError, match="stale"):
            robot.send_action(np.array([0.0], dtype=np.float32))

    def test_disconnect_preserves_external_ros_context(self, mock_ros: types.SimpleNamespace) -> None:
        robot = ROS2Robot(["joint_1"])
        robot._rclpy = mock_ros.rclpy
        robot._node = _Node()
        robot._publisher = robot._node.publisher
        robot._executor = _Executor()

        robot.disconnect()

        mock_ros.rclpy.shutdown.assert_not_called()
