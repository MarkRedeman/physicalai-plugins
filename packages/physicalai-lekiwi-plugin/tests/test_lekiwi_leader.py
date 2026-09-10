from __future__ import annotations

from dataclasses import dataclass
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from physicalai.config import Config

from physicalai_lekiwi_plugin.lekiwi_leader import LeKiwiLeader


@dataclass
class _FakeArmObservation:
    joint_positions: np.ndarray


class _FakeSO101Leader:
    def __init__(self, arm_positions: np.ndarray | None = None) -> None:
        self._connected = False
        self._arm_positions = np.arange(6, dtype=np.float32) if arm_positions is None else arm_positions

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_observation(self) -> _FakeArmObservation:
        return _FakeArmObservation(joint_positions=self._arm_positions)

    @property
    def device_ids(self) -> tuple[str, ...]:
        return ("serial:ttyUSB0",)


class _FakeBaseSource:
    def __init__(self, base_action: np.ndarray) -> None:
        self._base_action = np.asarray(base_action, dtype=np.float32)
        self.connect_calls = 0
        self.disconnect_calls = 0

    def connect(self, *, bus: object, session_id: str) -> None:
        _ = bus, session_id
        self.connect_calls += 1

    def update(self, robot_state: object, camera_frames: object, step: int) -> np.ndarray:
        _ = robot_state, camera_frames, step
        return self._base_action

    def disconnect(self) -> None:
        self.disconnect_calls += 1


def test_defaults_construct_uncalibrated_so101_leader() -> None:
    fake_so101 = _FakeSO101Leader()
    with patch("physicalai_lekiwi_plugin.lekiwi_leader.SO101.uncalibrated", return_value=fake_so101) as uncalibrated:
        robot = LeKiwiLeader(port="/dev/ttyUSB4", baudrate=115_200)

    uncalibrated.assert_called_once_with(
        port="/dev/ttyUSB4",
        baudrate=115_200,
        role="leader",
        unit="ticks",
    )
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
    assert robot.role == "leader"
    assert robot.device_ids == ("serial:ttyUSB0",)
    assert Config.from_instance(robot) == {
        "class_path": "physicalai_lekiwi_plugin.LeKiwiLeader",
        "init_args": {
            "port": "/dev/ttyUSB4",
            "baudrate": 115200,
        },
    }


def test_connect_and_observation_append_keyboard_base_command() -> None:
    fake_so101 = _FakeSO101Leader(np.array([10, 20, 30, 40, 50, 60], dtype=np.float32))
    with patch("physicalai_lekiwi_plugin.lekiwi_leader.SO101.uncalibrated", return_value=fake_so101):
        robot = LeKiwiLeader()

    robot._base_source = _FakeBaseSource(np.array([0.2, -0.1, 30.0], dtype=np.float32))  # noqa: SLF001
    robot.connect()
    obs = robot.get_observation()

    np.testing.assert_allclose(obs.joint_positions[:6], [10, 20, 30, 40, 50, 60])
    np.testing.assert_allclose(obs.joint_positions[6:], [0.2, -0.1, 30.0])
    assert obs.sensor_data is not None
    np.testing.assert_allclose(obs.sensor_data["base_command"], [0.2, -0.1, 30.0])


def test_connect_without_tty_keeps_zero_base_command() -> None:
    fake_so101 = _FakeSO101Leader(np.array([1, 2, 3, 4, 5, 6], dtype=np.float32))
    with patch("physicalai_lekiwi_plugin.lekiwi_leader.SO101.uncalibrated", return_value=fake_so101):
        robot = LeKiwiLeader()

    robot._base_source = MagicMock()  # noqa: SLF001
    robot._base_source.connect.side_effect = RuntimeError("interactive TTY required")  # noqa: SLF001

    robot.connect()
    obs = robot.get_observation()

    assert robot.is_connected() is True
    np.testing.assert_allclose(obs.joint_positions[:6], [1, 2, 3, 4, 5, 6])
    np.testing.assert_allclose(obs.joint_positions[6:], [0.0, 0.0, 0.0])


def test_connect_with_closed_stdin_error_keeps_zero_base_command() -> None:
    fake_so101 = _FakeSO101Leader(np.array([1, 2, 3, 4, 5, 6], dtype=np.float32))
    with patch("physicalai_lekiwi_plugin.lekiwi_leader.SO101.uncalibrated", return_value=fake_so101):
        robot = LeKiwiLeader()

    robot._base_source = MagicMock()  # noqa: SLF001
    robot._base_source.connect.side_effect = ValueError("I/O operation on closed file")  # noqa: SLF001

    robot.connect()
    obs = robot.get_observation()

    assert robot.is_connected() is True
    np.testing.assert_allclose(obs.joint_positions[:6], [1, 2, 3, 4, 5, 6])
    np.testing.assert_allclose(obs.joint_positions[6:], [0.0, 0.0, 0.0])


def test_send_action_raises_read_only_error() -> None:
    fake_so101 = _FakeSO101Leader()
    with patch("physicalai_lekiwi_plugin.lekiwi_leader.SO101.uncalibrated", return_value=fake_so101):
        robot = LeKiwiLeader()

    with pytest.raises(RuntimeError, match="read-only"):
        robot.send_action(np.zeros(9, dtype=np.float32))


def test_use_gamepad_selects_gamepad_source() -> None:
    fake_so101 = _FakeSO101Leader()
    fake_gamepad_source = MagicMock()
    with (
        patch("physicalai_lekiwi_plugin.lekiwi_leader.SO101.uncalibrated", return_value=fake_so101),
        patch("physicalai_lekiwi_plugin.lekiwi_leader.GamepadTeleop", return_value=fake_gamepad_source),
    ):
        robot = LeKiwiLeader(use_gamepad=True)

    assert robot._base_source is fake_gamepad_source  # noqa: SLF001


def test_use_gamepad_fails_fast_when_gamepad_connect_fails() -> None:
    fake_so101 = _FakeSO101Leader()
    with patch("physicalai_lekiwi_plugin.lekiwi_leader.SO101.uncalibrated", return_value=fake_so101):
        robot = LeKiwiLeader(use_gamepad=True)

    robot._base_source = MagicMock()  # noqa: SLF001
    robot._base_source.connect.side_effect = RuntimeError("No gamepad detected")  # noqa: SLF001

    robot.connect()

    with pytest.raises(RuntimeError, match="gamepad setup failed"):
        robot.get_observation()

    assert fake_so101.is_connected() is True


def test_calibration_from_lekiwi_dict_remaps_arm_joint_names() -> None:
    lekiwi_calibration = {
        "arm_shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": 100, "range_max": 4000},
        "arm_shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": 0, "range_min": 100, "range_max": 4000},
        "arm_elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": 0, "range_min": 100, "range_max": 4000},
        "arm_wrist_flex": {"id": 4, "drive_mode": 0, "homing_offset": 0, "range_min": 100, "range_max": 4000},
        "arm_wrist_roll": {"id": 5, "drive_mode": 0, "homing_offset": 0, "range_min": 100, "range_max": 4000},
        "arm_gripper": {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": 100, "range_max": 4000},
    }

    with patch("physicalai_lekiwi_plugin.lekiwi_leader.SO101Calibration.from_dict", return_value=object()) as from_dict:
        LeKiwiLeader._calibration_from_dict(lekiwi_calibration)  # noqa: SLF001

    called_dict = from_dict.call_args.args[0]
    assert sorted(called_dict) == [
        "elbow_flex",
        "gripper",
        "shoulder_lift",
        "shoulder_pan",
        "wrist_flex",
        "wrist_roll",
    ]
