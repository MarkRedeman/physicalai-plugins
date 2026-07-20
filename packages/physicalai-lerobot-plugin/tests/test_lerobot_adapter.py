# ruff: noqa: SLF001

from __future__ import annotations

from unittest.mock import MagicMock, PropertyMock

import numpy as np
import pytest

SO100_JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
SO100_POS_KEYS = [f"{j}.pos" for j in SO100_JOINT_ORDER]


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
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=SO100_JOINT_ORDER,
        )

        assert adapter.NUM_JOINTS == 6
        assert adapter.joint_names == SO100_JOINT_ORDER

    def test_custom_key_mappings(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=["j1", "j2"],
            obs_position_keys=["motor_1_pos", "motor_2_pos"],
            act_position_keys=["goal_1_pos", "goal_2_pos"],
        )

        assert adapter.NUM_JOINTS == 2

    def test_invalid_role(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        with pytest.raises(ValueError, match="Invalid role"):
            LeRobotAdapter(
                robot=mock_lerobot_robot,
                joint_order=SO100_JOINT_ORDER,
                role="invalid",
            )

    def test_key_count_mismatch(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        with pytest.raises(ValueError, match="obs_position_keys length"):
            LeRobotAdapter(
                robot=mock_lerobot_robot,
                joint_order=SO100_JOINT_ORDER,
                obs_position_keys=["only_one"],
            )


class TestLeRobotAdapterLifecycle:
    def test_connect_and_disconnect(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=SO100_JOINT_ORDER,
        )

        assert not adapter.is_connected()
        adapter.connect()
        assert adapter.is_connected()
        assert mock_lerobot_robot.connect.called

        adapter.disconnect()
        assert not adapter.is_connected()

    def test_connect_is_idempotent(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=SO100_JOINT_ORDER,
        )

        adapter.connect()
        adapter.connect()
        assert mock_lerobot_robot.connect.call_count == 1

    def test_disconnect_is_idempotent(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=SO100_JOINT_ORDER,
        )

        adapter.disconnect()
        adapter.disconnect()
        assert mock_lerobot_robot.disconnect.call_count == 0


class TestLeRobotAdapterObservation:
    def test_get_observation(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=SO100_JOINT_ORDER,
        )

        obs = adapter.get_observation()
        np.testing.assert_array_almost_equal(
            obs.joint_positions,
            np.array([0.0, 10.0, 20.0, 30.0, 40.0, 50.0], dtype=np.float32),
        )
        assert obs.timestamp > 0
        assert obs.state is obs.joint_positions

    def test_get_observation_with_custom_keys(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        custom_obs = {"custom_pos_1": 15.0, "custom_pos_2": 25.0}
        mock_lerobot_robot.get_observation.side_effect = None
        mock_lerobot_robot.get_observation.return_value = custom_obs

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=["j1", "j2"],
            obs_position_keys=["custom_pos_1", "custom_pos_2"],
            act_position_keys=["custom_pos_1", "custom_pos_2"],
        )

        obs = adapter.get_observation()
        np.testing.assert_array_almost_equal(
            obs.joint_positions,
            np.array([15.0, 25.0], dtype=np.float32),
        )


class TestLeRobotAdapterAction:
    def test_send_action_follower(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=SO100_JOINT_ORDER,
        )

        action = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float32)
        adapter.send_action(action)

        expected_dict = dict(zip(SO100_POS_KEYS, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0], strict=True))
        mock_lerobot_robot.send_action.assert_called_once_with(expected_dict)

    def test_send_action_leader_raises(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=SO100_JOINT_ORDER,
            role="leader",
        )

        action = np.zeros(6, dtype=np.float32)
        with pytest.raises(RuntimeError, match="Cannot send actions to a leader"):
            adapter.send_action(action)

    def test_send_action_wrong_shape(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=SO100_JOINT_ORDER,
        )

        action = np.zeros(3, dtype=np.float32)
        with pytest.raises(ValueError, match="Expected action shape"):
            adapter.send_action(action)

    def test_send_action_with_custom_keys(self, mock_lerobot_robot: MagicMock) -> None:
        from physicalai_lerobot_plugin import LeRobotAdapter

        adapter = LeRobotAdapter(
            robot=mock_lerobot_robot,
            joint_order=["j1", "j2"],
            act_position_keys=["goal_pos_1", "goal_pos_2"],
        )

        action = np.array([10.0, 20.0], dtype=np.float32)
        adapter.send_action(action)

        expected_dict = {"goal_pos_1": 10.0, "goal_pos_2": 20.0}
        mock_lerobot_robot.send_action.assert_called_once_with(expected_dict)
