from __future__ import annotations

import sys
from importlib import import_module
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, call, patch

import numpy as np
import pytest

if TYPE_CHECKING:
    from collections.abc import Generator

sys.modules.setdefault("motorbridge_smart_servo", MagicMock())


def _make_mock_smart_servo() -> MagicMock:
    module = MagicMock()
    bus = MagicMock()
    module.FashionStarServo.return_value = bus
    bus.ping.return_value = True

    servo_angles = [0.0, 10.0, -10.0, 30.0, 40.0, 50.0, 60.0]

    def read_angle_side_effect(servo_id: int, *, multi_turn: bool = True) -> MagicMock:
        sample = MagicMock()
        sample.raw_deg = _make_mock_smart_servo.servo_angles[servo_id]
        sample.filtered_deg = _make_mock_smart_servo.servo_angles[servo_id]
        sample.reliable = True
        return sample

    _make_mock_smart_servo.servo_angles = servo_angles

    bus.read_angle.side_effect = read_angle_side_effect
    return module


@pytest.fixture
def mock_smart_servo() -> Generator[MagicMock]:
    module = _make_mock_smart_servo()
    sys.modules.pop("physicalai_rebot_b601_plugin.leader", None)
    sys.modules.pop("physicalai_rebot_b601_plugin", None)
    pkg = sys.modules.get("physicalai_rebot_b601_plugin")
    if pkg is not None and hasattr(pkg, "leader"):
        del pkg.leader
    with patch.dict(sys.modules, {"motorbridge_smart_servo": module}):
        import_module("physicalai_rebot_b601_plugin.leader")
        yield module


def _create_robot(mock_smart_servo: MagicMock, **kwargs: object) -> object:
    from physicalai_rebot_b601_plugin import ReBotArm102Leader

    return ReBotArm102Leader(**kwargs)


class TestReBotArm102LeaderConstruction:
    def test_defaults(self, mock_smart_servo: MagicMock) -> None:
        robot = _create_robot(mock_smart_servo)

        assert robot.port == "/dev/ttyUSB0"
        assert robot.baudrate == 1_000_000
        assert robot.joint_names == [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_yaw",
            "wrist_roll",
            "gripper",
        ]

    def test_invalid_baudrate_raises(self, mock_smart_servo: MagicMock) -> None:
        from physicalai_rebot_b601_plugin import ReBotArm102Leader

        with pytest.raises(ValueError, match="baudrate"):
            ReBotArm102Leader(baudrate=0)


class TestReBotArm102LeaderLifecycle:
    def test_connect_pings_and_configures(self, mock_smart_servo: MagicMock) -> None:
        robot = _create_robot(mock_smart_servo, port="/dev/ttyUSB1")
        robot.connect()

        mock_smart_servo.FashionStarServo.assert_called_once_with("/dev/ttyUSB1", baudrate=1_000_000)
        bus = mock_smart_servo.FashionStarServo.return_value

        assert bus.ping.call_args_list == [
            call(0),
            call(1),
            call(2),
            call(3),
            call(4),
            call(5),
            call(6),
        ]
        assert bus.unlock.call_count == 7
        assert bus.reset_multi_turn.call_count == 7

    def test_connect_failure_cleans_up(self, mock_smart_servo: MagicMock) -> None:
        bus = mock_smart_servo.FashionStarServo.return_value
        bus.ping.return_value = False
        robot = _create_robot(mock_smart_servo)

        with pytest.raises(ConnectionError, match="did not respond"):
            robot.connect()

        bus.close.assert_called_once()
        assert robot.is_connected() is False

    def test_connect_is_idempotent(self, mock_smart_servo: MagicMock) -> None:
        robot = _create_robot(mock_smart_servo)
        robot.connect()
        robot.connect()

        mock_smart_servo.FashionStarServo.assert_called_once()

    def test_disconnect_closes_bus(self, mock_smart_servo: MagicMock) -> None:
        robot = _create_robot(mock_smart_servo)
        robot.connect()
        bus = mock_smart_servo.FashionStarServo.return_value

        robot.disconnect()

        bus.close.assert_called_once()
        assert robot.is_connected() is False


class TestReBotArm102LeaderObservation:
    def test_observation_returns_angles(self, mock_smart_servo: MagicMock) -> None:
        robot = _create_robot(mock_smart_servo)
        robot.connect()
        obs = robot.get_observation()

        expected = np.array([0, 10, -10, 30, 40, 50, 60], dtype=np.float32)
        np.testing.assert_allclose(obs.joint_positions, expected)
        assert isinstance(obs.timestamp, float)
        assert obs.sensor_data is not None
        assert "raw_positions" in obs.sensor_data
        assert "reliable" in obs.sensor_data

    def test_send_action_raises(self, mock_smart_servo: MagicMock) -> None:
        robot = _create_robot(mock_smart_servo)
        robot.connect()

        with pytest.raises(RuntimeError, match="read-only"):
            robot.send_action(np.zeros(7, dtype=np.float32))
