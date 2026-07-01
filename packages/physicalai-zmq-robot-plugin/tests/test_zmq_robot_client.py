from __future__ import annotations

import json
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import zmq

from physicalai_zmq_robot_plugin.zmq_robot import ZMQRobot, ZMQRobotObservation


@pytest.fixture
def mock_socket():
    socket = MagicMock(spec=zmq.Socket)
    socket.recv_string.return_value = json.dumps(
        {"event": "features_read", "features": ["j1", "j2", "j3"]}
    )
    return socket


@pytest.fixture
def mock_context(mock_socket):
    ctx = MagicMock(spec=zmq.Context)
    ctx.socket.return_value = mock_socket
    return ctx


@pytest.fixture
def robot(mock_context):
    with patch("physicalai_zmq_robot_plugin.zmq_robot.zmq.Context", return_value=mock_context):
        r = ZMQRobot("tcp://localhost:5555")
        r.connect()
        return r


class TestZMQRobotObservation:
    def test_state_property(self):
        obs = ZMQRobotObservation(
            joint_positions=np.array([1.0, 2.0, 3.0], dtype=np.float32),
            timestamp=12345.0,
        )
        assert np.allclose(obs.state, np.array([1.0, 2.0, 3.0]))
        assert obs.timestamp == 12345.0

    def test_defaults(self):
        obs = ZMQRobotObservation(
            joint_positions=np.array([1.0, 2.0], dtype=np.float32),
            timestamp=0.0,
        )
        assert obs.sensor_data is None
        assert obs.images is None


class TestZMQRobotConnect:
    def test_connect_success(self, mock_context, mock_socket):
        with patch("physicalai_zmq_robot_plugin.zmq_robot.zmq.Context", return_value=mock_context):
            robot = ZMQRobot("tcp://localhost:5555")
            robot.connect()
            assert robot.is_connected()
            assert robot._joint_names == ["j1", "j2", "j3"]
            mock_socket.connect.assert_called_once_with("tcp://localhost:5555")

    def test_connect_idempotent(self, robot):
        robot.connect()
        assert robot.is_connected()

    def test_connect_no_features(self, mock_context, mock_socket):
        mock_socket.recv_string.return_value = json.dumps({"event": "features_read"})
        with patch("physicalai_zmq_robot_plugin.zmq_robot.zmq.Context", return_value=mock_context):
            robot = ZMQRobot("tcp://localhost:5555")
            robot.connect()
            assert robot._joint_names == []

    def test_joint_names(self, robot):
        assert robot.joint_names == ["j1", "j2", "j3"]
        assert robot.joint_names is not robot._joint_names


class TestZMQRobotLifecycle:
    def test_disconnect(self, robot):
        robot.disconnect()
        assert not robot.is_connected()

    def test_disconnect_idempotent(self, robot):
        robot.disconnect()
        robot.disconnect()
        assert not robot.is_connected()

    def test_context_manager(self, mock_context, mock_socket):
        from physicalai.robot import connect

        with patch("physicalai_zmq_robot_plugin.zmq_robot.zmq.Context", return_value=mock_context):
            robot = ZMQRobot("tcp://localhost:5555")
            with connect(robot) as r:
                assert r.is_connected()
            assert not r.is_connected()

    def test_get_observation_before_connect(self):
        robot = ZMQRobot("tcp://localhost:5555")
        with pytest.raises(ConnectionError, match="not connected"):
            robot.get_observation()

    def test_send_action_before_connect(self):
        robot = ZMQRobot("tcp://localhost:5555")
        with pytest.raises(ConnectionError, match="not connected"):
            robot.send_action(np.array([0.0, 0.0, 0.0]))


class TestZMQRobotObservation:
    def test_get_observation(self, robot, mock_socket):
        mock_socket.recv_string.return_value = json.dumps(
            {"state": {"j1": 10.0, "j2": 20.0, "j3": 30.0}}
        )
        obs = robot.get_observation()
        assert np.allclose(obs.joint_positions, np.array([10.0, 20.0, 30.0]))

    def test_get_observation_timestamp(self, robot, mock_socket):
        mock_socket.recv_string.return_value = json.dumps(
            {"state": {"j1": 0.0, "j2": 0.0, "j3": 0.0}}
        )
        import time

        before = time.monotonic()
        obs = robot.get_observation()
        after = time.monotonic()
        assert before <= obs.timestamp <= after

    def test_get_observation_empty_state(self, robot, mock_socket):
        mock_socket.recv_string.return_value = json.dumps({})
        obs = robot.get_observation()
        assert np.allclose(obs.joint_positions, np.array([0.0, 0.0, 0.0]))


class TestZMQRobotActions:
    def test_send_action(self, robot, mock_socket):
        mock_socket.recv_string.return_value = json.dumps(
            {"event": "joints_state_was_set"}
        )
        action = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        robot.send_action(action, goal_time=0.5)

        sent_str = mock_socket.send_string.call_args[0][0]
        sent = json.loads(sent_str)
        assert sent["command"] == "set_joints_state"
        assert sent["payload"]["joints"] == {"j1": 1.0, "j2": 2.0, "j3": 3.0}
        assert sent["payload"]["goal_time"] == 0.5

    def test_enable_torque(self, robot, mock_socket):
        mock_socket.recv_string.return_value = json.dumps(
            {"event": "torque_was_enabled"}
        )
        robot.enable_torque()
        sent_str = mock_socket.send_string.call_args[0][0]
        sent = json.loads(sent_str)
        assert sent["command"] == "enable_torque"

    def test_disable_torque(self, robot, mock_socket):
        mock_socket.recv_string.return_value = json.dumps(
            {"event": "torque_was_disabled"}
        )
        robot.disable_torque()
        sent_str = mock_socket.send_string.call_args[0][0]
        sent = json.loads(sent_str)
        assert sent["command"] == "disable_torque"

    def test_send_action_shape_mismatch(self, robot, mock_socket):
        with pytest.raises((ValueError, IndexError)):
            robot.send_action(np.array([1.0, 2.0]))

    def test_zmq_error_raised(self, robot, mock_socket):
        mock_socket.recv_string.side_effect = zmq.ZMQError("Connection refused")
        with pytest.raises(ConnectionError, match="ZMQ error"):
            robot.get_observation()
