# ruff: noqa: S301, S403, SLF001

from __future__ import annotations

import json
import pickle
from unittest.mock import MagicMock, PropertyMock

import numpy as np
import pytest
from physicalai.config import to_config


class MockRobotConfig:
    pass


_MOCK_CONFIG_CLS_PATH = f"{__name__}.MockRobotConfig"


def _make_adapter(
    mock_robot: MagicMock,
    role: str = "follower",
) -> object:
    from physicalai_lerobot_plugin.lerobot_adapter import LeRobotAdapter

    return LeRobotAdapter(
        config_cls_path=_MOCK_CONFIG_CLS_PATH,
        config_kwargs={},
        role=role,
        _robot=mock_robot,
    )


SO100_POS_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]


@pytest.fixture
def mock_lerobot_robot() -> MagicMock:
    robot = MagicMock()
    robot._is_connected_state = False

    type(robot).is_connected = PropertyMock(side_effect=lambda: robot._is_connected_state)

    def connect(calibrate: bool = True) -> None:  # noqa: FBT001, FBT002
        robot._is_connected_state = True

    robot.connect.side_effect = connect

    def disconnect() -> None:
        robot._is_connected_state = False

    robot.disconnect.side_effect = disconnect

    def get_observation() -> dict:
        return {key: float(i * 10) for i, key in enumerate(SO100_POS_KEYS)}

    robot.get_observation.side_effect = get_observation

    def send_action(action: dict) -> dict:
        return action

    robot.send_action.side_effect = send_action

    return robot


class TestLeRobotAdapterConstruction:
    def test_defaults(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)

        assert not adapter.is_connected()
        with pytest.raises(RuntimeError, match="not yet discovered"):
            _ = adapter.NUM_JOINTS

    def test_invalid_role(self, mock_lerobot_robot: MagicMock) -> None:

        with pytest.raises(ValueError, match="Invalid role"):
            _make_adapter(mock_lerobot_robot, role="invalid")

    def test_exports_json_safe_config_and_device_id(self) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(_MOCK_CONFIG_CLS_PATH, {"port": "/dev/serial/by-id/mock"})

        config = to_config(adapter)
        assert json.loads(json.dumps(config)) == config
        assert config["init_args"]["config_cls_path"] == _MOCK_CONFIG_CLS_PATH
        assert adapter.device_ids == ("serial:mock",)

    def test_pickle_discards_live_robot(self, mock_lerobot_robot: MagicMock) -> None:
        adapter = _make_adapter(mock_lerobot_robot)

        restored = pickle.loads(pickle.dumps(adapter))

        assert restored.robot is None
        assert restored.device_ids == ()

    def test_teleoperator_exports_json_safe_config(self) -> None:
        from physicalai_lerobot_plugin.lerobot_adapter import LeRobotTeleoperatorAdapter

        adapter = LeRobotTeleoperatorAdapter(_MOCK_CONFIG_CLS_PATH, {"port": "/dev/ttyUSB1"})

        config = to_config(adapter)
        assert json.loads(json.dumps(config)) == config
        assert config["init_args"]["config_cls_path"] == _MOCK_CONFIG_CLS_PATH
        assert adapter.device_ids == ("serial:ttyUSB1",)


class TestLeRobotAdapterLifecycle:
    def test_connect_and_disconnect(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)

        assert not adapter.is_connected()
        adapter.connect()
        assert adapter.is_connected()
        assert mock_lerobot_robot.connect.called

        adapter.disconnect()
        assert not adapter.is_connected()

    def test_connect_is_idempotent(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)

        adapter.connect()
        adapter.connect()
        assert mock_lerobot_robot.connect.call_count == 1

    def test_disconnect_is_idempotent(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)

        adapter.disconnect()
        adapter.disconnect()
        assert mock_lerobot_robot.disconnect.call_count == 0


class TestLeRobotAdapterAutoDetection:
    def test_connect_discovers_joint_order(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)

        adapter.connect()
        assert adapter.joint_names == [
            "elbow_flex",
            "gripper",
            "shoulder_lift",
            "shoulder_pan",
            "wrist_flex",
            "wrist_roll",
        ]
        assert adapter.NUM_JOINTS == 6

    def test_get_observation_auto_discovers(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)

        obs = adapter.get_observation()
        assert adapter.NUM_JOINTS == 6
        np.testing.assert_array_almost_equal(
            obs.joint_positions,
            np.array([20.0, 50.0, 10.0, 0.0, 30.0, 40.0], dtype=np.float32),
        )

    def test_joint_names_autodetected_from_pos_suffix(self, mock_lerobot_robot: MagicMock) -> None:

        custom_obs = {"j1.pos": 1.0, "j2.pos": 2.0, "j3.pos": 3.0}
        mock_lerobot_robot.get_observation.side_effect = None
        mock_lerobot_robot.get_observation.return_value = custom_obs

        adapter = _make_adapter(mock_lerobot_robot)
        obs = adapter.get_observation()
        assert adapter.joint_names == ["j1", "j2", "j3"]
        np.testing.assert_array_almost_equal(
            obs.joint_positions,
            np.array([1.0, 2.0, 3.0], dtype=np.float32),
        )

    def test_properties_raise_before_discovery(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)

        with pytest.raises(RuntimeError, match="not yet discovered"):
            _ = adapter.joint_names

        with pytest.raises(RuntimeError, match="not yet discovered"):
            _ = adapter.NUM_JOINTS


class TestLeRobotAdapterObservation:
    def test_observation_has_correct_structure(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)

        obs = adapter.get_observation()
        assert obs.timestamp > 0
        assert obs.state is obs.joint_positions
        assert isinstance(obs.joint_positions, np.ndarray)
        assert obs.joint_positions.dtype == np.float32


class TestLeRobotAdapterAction:
    def test_send_action_follower(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)
        adapter.connect()

        sorted_pos_keys = sorted(SO100_POS_KEYS)
        action = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float32)
        adapter.send_action(action)

        expected_dict = dict(zip(sorted_pos_keys, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0], strict=True))
        mock_lerobot_robot.send_action.assert_called_once_with(expected_dict)

    def test_send_action_leader_raises(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot, role="leader")

        action = np.zeros(3, dtype=np.float32)
        with pytest.raises(RuntimeError, match="Cannot send actions to a leader"):
            adapter.send_action(action)

    def test_send_action_wrong_shape(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)
        adapter.connect()

        action = np.zeros(3, dtype=np.float32)
        with pytest.raises(ValueError, match="Expected action shape"):
            adapter.send_action(action)

    def test_send_action_raises_before_discovery(self, mock_lerobot_robot: MagicMock) -> None:

        adapter = _make_adapter(mock_lerobot_robot)
        action = np.zeros(6, dtype=np.float32)

        with pytest.raises(RuntimeError, match="not yet discovered"):
            adapter.send_action(action)
