from __future__ import annotations

import json
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from physicalai_websocket_robot_plugin.websocket_robot import WebSocketRobot, WebSocketRobotObservation


@pytest.fixture
def mock_ws():
    with patch("physicalai_websocket_robot_plugin.websocket_robot.ws_connect") as mock_connect:
        ws = MagicMock()
        ws.recv.side_effect = [
            json.dumps({"event": "features_read", "features": ["j1", "j2", "j3"]}),
        ]
        mock_connect.return_value = ws
        yield ws


@pytest.fixture
def robot(mock_ws):
    r = WebSocketRobot("ws://localhost:8765")
    r.connect()
    return r


class TestWebSocketRobotObservation:
    def test_state_property(self) -> None:
        obs = WebSocketRobotObservation(
            joint_positions=np.array([1.0, 2.0, 3.0], dtype=np.float32),
            timestamp=12345.0,
        )
        assert np.allclose(obs.state, np.array([1.0, 2.0, 3.0]))
        assert obs.timestamp == 12345.0

    def test_defaults(self) -> None:
        obs = WebSocketRobotObservation(
            joint_positions=np.array([1.0, 2.0], dtype=np.float32),
            timestamp=0.0,
        )
        assert obs.sensor_data is None
        assert obs.images is None


class TestWebSocketRobotConnect:
    def test_connect_success(self, mock_ws) -> None:
        robot = WebSocketRobot("ws://localhost:8765")
        robot.connect()
        assert robot.is_connected()
        assert robot._joint_names == ["j1", "j2", "j3"]

    def test_connect_idempotent(self, robot) -> None:
        robot.connect()
        assert robot.is_connected()

    def test_connect_timeout(self) -> None:
        with patch("physicalai_websocket_robot_plugin.websocket_robot.ws_connect") as mock_connect:
            mock_connect.side_effect = TimeoutError("Connection timed out")
            robot = WebSocketRobot("ws://localhost:8765", connect_timeout=1.0)
            with pytest.raises(ConnectionError, match="timed out"):
                robot.connect()

    def test_connect_no_features(self) -> None:
        with patch("physicalai_websocket_robot_plugin.websocket_robot.ws_connect") as mock_connect:
            ws = MagicMock()
            ws.recv.return_value = json.dumps({"event": "features_read"})
            mock_connect.return_value = ws
            robot = WebSocketRobot("ws://localhost:8765")
            robot.connect()
            assert robot._joint_names == []

    def test_joint_names(self, robot) -> None:
        assert robot.joint_names == ["j1", "j2", "j3"]
        assert robot.joint_names is not robot._joint_names


class TestWebSocketRobotLifecycle:
    def test_disconnect(self, robot) -> None:
        robot.disconnect()
        assert not robot.is_connected()

    def test_disconnect_idempotent(self, robot) -> None:
        robot.disconnect()
        robot.disconnect()
        assert not robot.is_connected()

    def test_context_manager(self, mock_ws) -> None:
        from physicalai.robot import connect

        robot = WebSocketRobot("ws://localhost:8765")
        with connect(robot) as r:
            assert r.is_connected()
        assert not r.is_connected()

    def test_get_observation_before_connect(self) -> None:
        robot = WebSocketRobot("ws://localhost:8765")
        with pytest.raises(ConnectionError, match="not connected"):
            robot.get_observation()

    def test_send_action_before_connect(self) -> None:
        robot = WebSocketRobot("ws://localhost:8765")
        with pytest.raises(ConnectionError, match="not connected"):
            robot.send_action(np.array([0.0, 0.0, 0.0]))


class TestWebSocketRobotObservation:
    def test_get_observation(self, robot, mock_ws) -> None:
        mock_ws.recv.side_effect = [
            json.dumps({"event": "state_read", "state": {"j1": 10.0, "j2": 20.0, "j3": 30.0}}),
        ]
        obs = robot.get_observation()
        assert np.allclose(obs.joint_positions, np.array([10.0, 20.0, 30.0]))

    def test_get_observation_drains_state_updates(self, robot, mock_ws) -> None:
        mock_ws.recv.side_effect = [
            json.dumps({"event": "joints_state_was_updated", "state": {"j1": 1.0, "j2": 2.0, "j3": 3.0}}),
            json.dumps({"event": "joints_state_was_updated", "state": {"j1": 4.0, "j2": 5.0, "j3": 6.0}}),
            json.dumps({"event": "state_read", "state": {"j1": 10.0, "j2": 20.0, "j3": 30.0}}),
        ]
        obs = robot.get_observation()
        assert np.allclose(obs.joint_positions, np.array([10.0, 20.0, 30.0]))

    def test_get_observation_uses_cached_state(self, robot, mock_ws) -> None:
        robot._cached_state = {"j1": 10.0, "j2": 20.0, "j3": 30.0}
        mock_ws.recv.side_effect = [
            json.dumps({"event": "state_read", "state": {"j1": 10.0, "j2": 20.0, "j3": 30.0}}),
        ]
        obs = robot.get_observation()
        assert np.allclose(obs.joint_positions, np.array([10.0, 20.0, 30.0]))

    def test_get_observation_timestamp(self, robot, mock_ws) -> None:
        mock_ws.recv.side_effect = [
            json.dumps({"event": "state_read", "state": {"j1": 0.0, "j2": 0.0, "j3": 0.0}}),
        ]
        import time

        before = time.monotonic()
        obs = robot.get_observation()
        after = time.monotonic()
        assert before <= obs.timestamp <= after


class TestWebSocketRobotActions:
    def test_send_action(self, robot, mock_ws) -> None:
        mock_ws.recv.side_effect = [
            json.dumps({"event": "joints_state_was_set"}),
        ]
        action = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        robot.send_action(action, goal_time=0.5)

        sent = json.loads(mock_ws.send.call_args[0][0])
        assert sent["event"] == "set_joints_state"
        assert sent["payload"]["joints"] == {"j1": 1.0, "j2": 2.0, "j3": 3.0}
        assert sent["payload"]["goal_time"] == 0.5

    def test_enable_torque(self, robot, mock_ws) -> None:
        mock_ws.recv.side_effect = [
            json.dumps({"event": "torque_was_enabled"}),
        ]
        robot.enable_torque()
        sent = json.loads(mock_ws.send.call_args[0][0])
        assert sent["event"] == "enable_torque"

    def test_disable_torque(self, robot, mock_ws) -> None:
        mock_ws.recv.side_effect = [
            json.dumps({"event": "torque_was_disabled"}),
        ]
        robot.disable_torque()
        sent = json.loads(mock_ws.send.call_args[0][0])
        assert sent["event"] == "disable_torque"

    def test_send_action_shape_mismatch(self, robot) -> None:
        with pytest.raises((ValueError, IndexError)):
            robot.send_action(np.array([1.0, 2.0]))
