# ruff: noqa: SLF001

from __future__ import annotations

import sys
from importlib import import_module
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, call, patch

import numpy as np
import pytest

if TYPE_CHECKING:
    from collections.abc import Generator

sys.modules.setdefault("scservo_sdk", MagicMock())


def _make_mock_scservo_sdk() -> MagicMock:
    module = MagicMock()

    port_handler = MagicMock()
    port_handler.openPort.return_value = True
    port_handler.setBaudRate.return_value = True

    packet_handler = MagicMock()
    packet_handler.ping.return_value = (0, 0, 0)
    packet_handler.write1ByteTxRx.return_value = (0, 0)
    packet_handler.write2ByteTxRx.return_value = (0, 0)

    arm_sync_read = MagicMock()
    arm_sync_read.addParam.return_value = True
    arm_sync_read.txRxPacket.return_value = 0
    arm_sync_read.isAvailable.return_value = True

    arm_sync_write = MagicMock()
    arm_sync_write.addParam.return_value = True
    arm_sync_write.txPacket.return_value = 0

    base_sync_read = MagicMock()
    base_sync_read.addParam.return_value = True
    base_sync_read.txRxPacket.return_value = 0
    base_sync_read.isAvailable.return_value = True

    base_sync_write = MagicMock()
    base_sync_write.addParam.return_value = True
    base_sync_write.txPacket.return_value = 0

    module.PortHandler.return_value = port_handler
    module.PacketHandler.return_value = packet_handler
    module.GroupSyncRead.side_effect = [arm_sync_read, base_sync_read]
    module.GroupSyncWrite.side_effect = [arm_sync_write, base_sync_write]

    module.mock_arm_sync_read = arm_sync_read
    module.mock_arm_sync_write = arm_sync_write
    module.mock_base_sync_read = base_sync_read
    module.mock_base_sync_write = base_sync_write
    module.mock_port_handler = port_handler
    module.mock_packet_handler = packet_handler

    return module


@pytest.fixture
def mock_scservo_sdk() -> Generator[MagicMock]:
    module = _make_mock_scservo_sdk()

    for mod_name in list(sys.modules):
        if "physicalai_lekiwi_plugin" in mod_name:
            sys.modules.pop(mod_name, None)

    with patch.dict(sys.modules, {"scservo_sdk": module}):
        import_module("physicalai_lekiwi_plugin.lekiwi")
        yield module


def _create_robot(mock_sdk: MagicMock, **kwargs: object) -> object:
    from physicalai_lekiwi_plugin import LeKiwi

    return LeKiwi.uncalibrated(**kwargs)


class TestLeKiwiConstruction:
    def test_defaults(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)

        assert robot.port == "/dev/ttyACM0"
        assert robot.baudrate == 1_000_000
        assert robot.role == "follower"
        assert robot.joint_names == [
            "arm_shoulder_pan",
            "arm_shoulder_lift",
            "arm_elbow_flex",
            "arm_wrist_flex",
            "arm_wrist_roll",
            "arm_gripper",
            "base_left_wheel",
            "base_back_wheel",
            "base_right_wheel",
        ]
        assert robot.NUM_JOINTS == 9

    def test_invalid_role_raises(self, mock_scservo_sdk: MagicMock) -> None:
        from physicalai_lekiwi_plugin import LeKiwi

        with pytest.raises(ValueError, match="Invalid role"):
            LeKiwi.uncalibrated(role="invalid")

    def test_leader_role(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk, role="leader")
        assert robot.role == "leader"

    def test_invalid_baud_raises(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)
        with pytest.raises(ValueError, match="baudrate"):
            robot.baudrate = -1

    def test_custom_port(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk, port="/dev/ttyACM1")
        assert robot.port == "/dev/ttyACM1"


class TestLeKiwiLifecycle:
    def test_connect(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)
        robot.connect()

        sdk = mock_scservo_sdk
        sdk.PortHandler.assert_called_once_with("/dev/ttyACM0")
        sdk.mock_port_handler.openPort.assert_called_once()
        sdk.mock_port_handler.setBaudRate.assert_called_once_with(1_000_000)

        assert sdk.GroupSyncRead.call_args_list == [
            call(sdk.mock_port_handler, sdk.mock_packet_handler, 56, 2),
            call(sdk.mock_port_handler, sdk.mock_packet_handler, 62, 2),
        ]
        assert sdk.GroupSyncWrite.call_args_list == [
            call(sdk.mock_port_handler, sdk.mock_packet_handler, 42, 2),
            call(sdk.mock_port_handler, sdk.mock_packet_handler, 48, 2),
        ]

        assert sdk.mock_packet_handler.ping.call_count == 9

        torque_call_count = sum(1 for c in sdk.mock_packet_handler.write1ByteTxRx.call_args_list if c[0][2] == 40)
        assert torque_call_count >= 9

    def test_connect_is_idempotent(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)
        robot.connect()
        robot.connect()

        mock_scservo_sdk.PortHandler.assert_called_once()

    def test_connect_failure_cleans_up(self, mock_scservo_sdk: MagicMock) -> None:
        sdk = mock_scservo_sdk
        sdk.mock_port_handler.openPort.return_value = False
        robot = _create_robot(mock_scservo_sdk)

        with pytest.raises(ConnectionError, match="Failed to open"):
            robot.connect()

        assert robot.is_connected() is False

    def test_disconnect(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)
        robot.connect()
        robot.disconnect()

        mock_scservo_sdk.mock_port_handler.closePort.assert_called_once()
        assert robot.is_connected() is False

    def test_disconnect_when_not_connected(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)
        robot.disconnect()

        assert robot.is_connected() is False


class TestLeKiwiObservation:
    def test_observation_returns_correct_shape(self, mock_scservo_sdk: MagicMock) -> None:
        sdk = mock_scservo_sdk
        arm_get_data = sdk.mock_arm_sync_read.getData
        arm_get_data.side_effect = [100, 200, 300, 400, 500, 600]

        base_get_data = sdk.mock_base_sync_read.getData
        base_get_data.side_effect = [50, 75, 100]

        robot = _create_robot(mock_scservo_sdk)
        robot.connect()
        obs = robot.get_observation()

        assert obs.joint_positions.shape == (9,)
        assert obs.joint_positions.dtype == np.float32
        assert isinstance(obs.timestamp, float)
        assert obs.sensor_data is not None
        assert "wheel_velocities_degps" in obs.sensor_data
        assert "x_vel" in obs.sensor_data
        assert "y_vel" in obs.sensor_data
        assert "theta_vel" in obs.sensor_data

        np.testing.assert_allclose(
            obs.joint_positions[:6],
            np.array([100.0, 200.0, 300.0, 400.0, 500.0, 600.0], dtype=np.float32),
            atol=0.5,
        )

    def test_missing_arm_feedback_raises(self, mock_scservo_sdk: MagicMock) -> None:
        sdk = mock_scservo_sdk
        sdk.mock_arm_sync_read.txRxPacket.return_value = 1

        robot = _create_robot(mock_scservo_sdk)
        robot.connect()

        with pytest.raises(ConnectionError, match="Arm sync read failed"):
            robot.get_observation()

    def test_missing_base_feedback_raises(self, mock_scservo_sdk: MagicMock) -> None:
        sdk = mock_scservo_sdk
        sdk.mock_base_sync_read.txRxPacket.return_value = 1

        robot = _create_robot(mock_scservo_sdk)
        robot.connect()

        with pytest.raises(ConnectionError, match="Base sync read failed"):
            robot.get_observation()

    def test_disconnected_observation_raises(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)

        with pytest.raises(ConnectionError, match="not connected"):
            robot.get_observation()


class TestLeKiwiAction:
    def test_send_action_arm_and_base(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)
        robot.connect()

        action = np.array([100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 0.1, 0.0, 0.0], dtype=np.float32)
        robot.send_action(action)

        sdk = mock_scservo_sdk
        sdk.mock_arm_sync_write.clearParam.assert_called()
        assert sdk.mock_arm_sync_write.addParam.call_count == 6

        sdk.mock_base_sync_write.clearParam.assert_called()
        assert sdk.mock_base_sync_write.addParam.call_count == 3

    def test_send_action_wrong_shape_raises(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)
        robot.connect()

        with pytest.raises(ValueError, match="Expected action shape"):
            robot.send_action(np.zeros(6, dtype=np.float32))

    def test_send_action_disconnected_raises(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)

        with pytest.raises(ConnectionError, match="not connected"):
            robot.send_action(np.zeros(9, dtype=np.float32))

    def test_send_action_leader_raises(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk, role="leader")
        robot.connect()

        with pytest.raises(RuntimeError, match="read-only"):
            robot.send_action(np.zeros(9, dtype=np.float32))


class TestLeKiwiKinematics:
    def test_body_to_wheel_raw_zero(self, mock_scservo_sdk: MagicMock) -> None:
        from physicalai_lekiwi_plugin.lekiwi import LeKiwi

        result = LeKiwi._body_to_wheel_raw(0.0, 0.0, 0.0)
        assert result["base_left_wheel"] == 0
        assert result["base_back_wheel"] == 0
        assert result["base_right_wheel"] == 0

    def test_body_to_wheel_raw_forward(self, mock_scservo_sdk: MagicMock) -> None:
        from physicalai_lekiwi_plugin.lekiwi import LeKiwi

        result = LeKiwi._body_to_wheel_raw(0.1, 0.0, 0.0)
        assert isinstance(result["base_left_wheel"], int)
        assert result["base_left_wheel"] != 0

    def test_wheel_raw_to_body_zero(self, mock_scservo_sdk: MagicMock) -> None:
        from physicalai_lekiwi_plugin.lekiwi import LeKiwi

        result = LeKiwi._wheel_raw_to_body(0, 0, 0)
        assert result["x.vel"] == 0.0
        assert result["y.vel"] == 0.0
        assert abs(result["theta.vel"]) < 1e-10

    def test_degps_to_raw_roundtrip(self, mock_scservo_sdk: MagicMock) -> None:
        from physicalai_lekiwi_plugin.lekiwi import LeKiwi

        degps = 100.0
        raw = LeKiwi._degps_to_raw(degps)
        result = LeKiwi._raw_to_degps(raw)
        assert abs(result - degps) < 1.0

    def test_body_wheel_roundtrip(self, mock_scservo_sdk: MagicMock) -> None:
        from physicalai_lekiwi_plugin.lekiwi import LeKiwi

        x, y, theta = 0.1, 0.05, 10.0
        wheel_raw = LeKiwi._body_to_wheel_raw(x, y, theta)
        body = LeKiwi._wheel_raw_to_body(
            wheel_raw["base_left_wheel"],
            wheel_raw["base_back_wheel"],
            wheel_raw["base_right_wheel"],
        )
        assert abs(body["x.vel"] - x) < 0.02
        assert abs(body["y.vel"] - y) < 0.02
        assert abs(body["theta.vel"] - theta) < 2.0


class TestLeKiwiTorque:
    def test_disable_enable_torque(self, mock_scservo_sdk: MagicMock) -> None:
        robot = _create_robot(mock_scservo_sdk)
        robot.connect()
        sdk = mock_scservo_sdk

        robot.set_torque(enabled=False)
        torque_off_calls = [
            c for c in sdk.mock_packet_handler.write1ByteTxRx.call_args_list if c[0][2] == 40 and c[0][3] == 0
        ]
        assert len(torque_off_calls) >= 9

        robot.set_torque(enabled=True)
        torque_on_calls = [
            c for c in sdk.mock_packet_handler.write1ByteTxRx.call_args_list if c[0][2] == 40 and c[0][3] == 1
        ]
        assert len(torque_on_calls) >= 9
