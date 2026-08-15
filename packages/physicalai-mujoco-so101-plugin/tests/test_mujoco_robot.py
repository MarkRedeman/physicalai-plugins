from __future__ import annotations

import sys
import threading
import types
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from physicalai.config import to_config

from physicalai_mujoco_so101_plugin.http_server import ResetCommand, ShutdownCommand, SwitchSceneCommand
from physicalai_mujoco_so101_plugin.mujoco_robot import BiMuJoCoSO101, MuJoCoSO101, MuJoCoSO101Observation


@pytest.fixture
def mock_mujoco() -> MagicMock:
    """Mock MuJoCo functions at the function-call level.

    Yields:
        MagicMock: The mock context.
    """
    with (
        patch("mujoco.MjModel.from_xml_path") as mock_from_xml,
        patch("mujoco.MjData") as mock_data_cls,
        patch("mujoco.mj_forward"),
        patch("mujoco.mj_step"),
        patch("mujoco.mj_name2id", return_value=0),
        patch("mujoco.mjtObj", create=True),
    ):
        mock_model = MagicMock()
        mock_model.nq = 6
        mock_model.nv = 6
        mock_model.nu = 6
        mock_model.opt.timestep = 0.005
        mock_model.jnt_qposadr = [0, 1, 2, 3, 4, 5]
        mock_model.jnt_dofadr = [0, 1, 2, 3, 4, 5]
        mock_from_xml.return_value = mock_model

        mock_data = MagicMock()
        mock_data.qpos = np.array([0.0, 0.5, -0.3, 0.1, -0.2, 0.8])
        mock_data.qvel = np.array([0.0, 0.1, -0.05, 0.02, -0.01, 0.0])
        mock_data.ctrl = np.zeros(6)
        mock_data_cls.return_value = mock_data

        yield mock_model


@pytest.fixture
def mock_mujoco_bimanual() -> MagicMock:
    """Mock MuJoCo with a 12-DOF bimanual model.

    Yields:
        MagicMock: The mock context.
    """
    with (
        patch("mujoco.MjModel.from_xml_path") as mock_from_xml,
        patch("mujoco.MjData") as mock_data_cls,
        patch("mujoco.mj_forward"),
        patch("mujoco.mj_step"),
        patch("mujoco.mj_name2id", return_value=0),
        patch("mujoco.mjtObj", create=True),
    ):
        mock_model = MagicMock()
        mock_model.nq = 12
        mock_model.nv = 12
        mock_model.nu = 12
        mock_model.opt.timestep = 0.005
        mock_model.jnt_qposadr = list(range(12))
        mock_model.jnt_dofadr = list(range(12))
        mock_from_xml.return_value = mock_model

        mock_data = MagicMock()
        mock_data.qpos = np.array([0.0] * 12)
        mock_data.qvel = np.array([0.0] * 12)
        mock_data.ctrl = np.zeros(12)
        mock_data_cls.return_value = mock_data

        yield mock_model


class TestMuJoCoSO101ObservationRead:
    def test_state_property(self) -> None:
        obs = MuJoCoSO101Observation(
            joint_positions=np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float32),
            timestamp=12345.0,
        )
        assert np.allclose(obs.state, np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))
        assert obs.timestamp == 12345.0

    def test_defaults(self) -> None:
        obs = MuJoCoSO101Observation(
            joint_positions=np.array([0.0] * 6, dtype=np.float32),
            timestamp=0.0,
        )
        assert obs.sensor_data is None
        assert obs.images is None


class TestMuJoCoSO101Construction:
    def test_default_construction(self) -> None:
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        assert robot._model_path == "/fake/model.xml"  # noqa: SLF001
        assert robot._substeps == 1  # noqa: SLF001
        assert robot._model is None  # noqa: SLF001
        assert robot._data is None  # noqa: SLF001

    def test_custom_substeps(self) -> None:
        robot = MuJoCoSO101(model_path="/fake/model.xml", substeps=5)
        assert robot._substeps == 5  # noqa: SLF001

    def test_joint_names(self) -> None:
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        expected = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        assert robot.joint_names == expected

    def test_device_ids(self) -> None:
        robot = MuJoCoSO101(model_path="/path/to/so101.xml")
        assert robot.device_ids == ("mujoco:so101",)

    def test_exports_owner_construction_recipe(self) -> None:
        robot = MuJoCoSO101(model_path="/fake/model.xml", substeps=3, enable_viewer=True)

        assert to_config(robot) == {
            "class_path": "physicalai_mujoco_so101_plugin.mujoco_robot.MuJoCoSO101",
            "init_args": {
                "model_path": "/fake/model.xml",
                "substeps": 3,
                "enable_viewer": True,
            },
        }


class TestMuJoCoSO101Connect:
    def test_connect_success(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        assert robot.is_connected()

    def test_connect_idempotent(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        robot.connect()
        assert robot.is_connected()

    def test_disconnect(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        robot.disconnect()
        assert not robot.is_connected()
        assert robot._scene_on_reset is None  # noqa: SLF001

    def test_disconnect_idempotent(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.disconnect()
        robot.disconnect()

    @pytest.mark.parametrize("key", [ord("n"), ord("N")])
    def test_scene_switch_key(self, key: int) -> None:
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot._key_callback(key)  # noqa: SLF001
        assert robot._pending_scene_switch is True  # noqa: SLF001


class TestMuJoCoSO101ObservationReadBack:
    def test_get_observation_returns_degrees(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()

        obs = robot.get_observation()

        assert isinstance(obs, MuJoCoSO101Observation)
        assert obs.joint_positions.shape == (6,)
        assert obs.sensor_data is not None
        assert "velocities" in obs.sensor_data
        assert obs.sensor_data["velocities"].shape == (6,)
        assert obs.timestamp > 0

    def test_get_observation_before_connect(self) -> None:
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        with pytest.raises(ConnectionError, match="not connected"):
            robot.get_observation()


class TestMuJoCoSO101Actions:
    def test_send_action(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()

        action = np.array([10.0, 20.0, -5.0, 0.0, 15.0, 30.0], dtype=np.float32)
        robot.send_action(action, goal_time=0.1)

    def test_send_action_wrong_shape(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()

        with pytest.raises(ValueError, match="Expected action shape"):
            robot.send_action(np.array([1.0, 2.0, 3.0]))

    def test_send_action_before_connect(self) -> None:
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        with pytest.raises(ConnectionError, match="not connected"):
            robot.send_action(np.array([0.0] * 6))


class TestMuJoCoSO101Pickling:
    def test_getstate_before_connect(self) -> None:
        robot = MuJoCoSO101(model_path="/fake/model.xml", substeps=3)
        state = robot.__getstate__()
        assert state == {
            "_model_path": "/fake/model.xml",
            "_substeps": 3,
            "_enable_viewer": False,
            "_cameras": [],
            "_free_joints": ("block1:joint", "block2:joint", "block3:joint"),
            "_target_body_name": "target",
            "_spawn_center": (0.24, 0.0),
            "_spawn_min_r": 0.08,
            "_spawn_max_r": 0.34,
            "_spawn_angle_half_deg": 125.0,
            "_block_min_sep": 0.09,
            "_target_min_sep": 0.11,
            "_current_scene_id": None,
            "_http_host": "127.0.0.1",
            "_http_port": 0,
        }

    def test_getstate_after_connect(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        state = robot.__getstate__()
        assert "_model_path" in state
        assert "_substeps" in state
        assert "_enable_viewer" in state
        assert "_cameras" in state
        assert "_model" not in state

    def test_setstate(self) -> None:
        state = {"_model_path": "/fake/model.xml", "_substeps": 2, "_enable_viewer": True, "_cameras": []}
        robot = MuJoCoSO101.__new__(MuJoCoSO101)
        robot.__setstate__(state)
        assert robot._model_path == "/fake/model.xml"  # noqa: SLF001
        assert robot._substeps == 2  # noqa: SLF001
        assert robot._enable_viewer is True  # noqa: SLF001
        assert robot._model is None  # noqa: SLF001
        assert robot._data is None  # noqa: SLF001
        assert robot._viewer is None  # noqa: SLF001
        assert robot._http_host == "127.0.0.1"  # noqa: SLF001
        assert robot._http_port == 0  # noqa: SLF001

    def test_setstate_restores_http_config(self) -> None:
        state = {
            "_model_path": "/fake/model.xml",
            "_substeps": 1,
            "_enable_viewer": False,
            "_cameras": [],
            "_http_host": "0.0.0.0",  # noqa: S104
            "_http_port": 9000,
        }
        robot = MuJoCoSO101.__new__(MuJoCoSO101)
        robot.__setstate__(state)
        assert robot._http_host == "0.0.0.0"  # noqa: SLF001, S104
        assert robot._http_port == 9000  # noqa: SLF001


class TestBiMuJoCoSO101:
    def test_joint_names(self) -> None:
        robot = BiMuJoCoSO101(model_path="/fake/model.xml")
        assert len(robot.joint_names) == 12
        assert robot.joint_names[:6] == [
            "left_shoulder_pan",
            "left_shoulder_lift",
            "left_elbow_flex",
            "left_wrist_flex",
            "left_wrist_roll",
            "left_gripper",
        ]
        assert robot.joint_names[6:] == [
            "right_shoulder_pan",
            "right_shoulder_lift",
            "right_elbow_flex",
            "right_wrist_flex",
            "right_wrist_roll",
            "right_gripper",
        ]

    def test_num_joints(self) -> None:
        robot = BiMuJoCoSO101(model_path="/fake/model.xml")
        assert robot.NUM_JOINTS == 12

    def test_exports_owner_construction_recipe(self) -> None:
        robot = BiMuJoCoSO101(model_path="/fake/model.xml", substeps=2)

        assert to_config(robot) == {
            "class_path": "physicalai_mujoco_so101_plugin.mujoco_robot.BiMuJoCoSO101",
            "init_args": {
                "model_path": "/fake/model.xml",
                "substeps": 2,
            },
        }

    def test_connect_and_observation(self, mock_mujoco_bimanual: MagicMock) -> None:
        _ = mock_mujoco_bimanual
        robot = BiMuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()

        obs = robot.get_observation()
        assert isinstance(obs, MuJoCoSO101Observation)
        assert obs.joint_positions.shape == (12,)

    def test_send_action(self, mock_mujoco_bimanual: MagicMock) -> None:
        _ = mock_mujoco_bimanual
        robot = BiMuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()

        action = np.arange(12, dtype=np.float32)
        robot.send_action(action, goal_time=0.1)

    def test_send_action_wrong_shape(self, mock_mujoco_bimanual: MagicMock) -> None:
        _ = mock_mujoco_bimanual
        robot = BiMuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()

        with pytest.raises(ValueError, match="Expected action shape"):
            robot.send_action(np.array([1.0, 2.0, 3.0]))


class TestHttpCommands:
    def test_reset_command_calls_scene_reset(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        robot._scene_on_reset = MagicMock()  # noqa: SLF001
        robot._commands.put(ResetCommand())  # noqa: SLF001

        robot.get_observation()

        robot._scene_on_reset.assert_called_once_with(robot._model, robot._data, robot._rng)  # noqa: SLF001

    def test_reset_command_falls_back_to_randomize(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        robot._scene_on_reset = None  # noqa: SLF001
        robot._commands.put(ResetCommand())  # noqa: SLF001

        with patch.object(robot, "_randomize_blocks") as randomize:
            robot.get_observation()

        randomize.assert_called_once()

    def test_switch_scene_command(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        robot._commands.put(SwitchSceneCommand(scene_id="pick_lift"))  # noqa: SLF001

        with patch.object(robot, "_switch_to_scene") as switch:
            robot.get_observation()

        switch.assert_called_once_with("pick_lift")

    def test_unknown_switch_scene_command_is_handled(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        robot._commands.put(SwitchSceneCommand(scene_id="nope"))  # noqa: SLF001

        with patch.object(robot, "_switch_to_scene", side_effect=KeyError("nope")):
            robot.get_observation()

    def test_shutdown_command_sets_owner_event(self, mock_mujoco: MagicMock, monkeypatch: pytest.MonkeyPatch) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()

        event = threading.Event()
        fake_worker = types.ModuleType("physicalai.robot.transport._owner_worker")
        fake_worker.shutdown = event
        monkeypatch.setitem(sys.modules, "physicalai.robot.transport._owner_worker", fake_worker)

        robot._commands.put(ShutdownCommand())  # noqa: SLF001
        robot.get_observation()

        assert event.is_set()

    def test_shutdown_without_owner_event_does_not_raise(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        robot._commands.put(ShutdownCommand())  # noqa: SLF001
        robot.get_observation()

    def test_http_status_shape(self) -> None:
        robot = MuJoCoSO101(
            model_path="/fake/model.xml",
            cameras=[{"name": "overview", "device": None}],
        )
        status = robot._http_status()  # noqa: SLF001
        assert status["connected"] is False
        assert status["scene"] is None
        assert "single_pick_place" in status["scenes"]
        assert status["cameras"] == [
            {
                "name": "overview",
                "width": 640,
                "height": 480,
                "fps": 30,
                "device": None,
                "rendering": False,
            },
        ]


class TestHttpServerIntegration:
    @staticmethod
    def _free_port() -> int:
        import socket

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", 0))
            return int(sock.getsockname()[1])

    def test_connect_starts_server_disconnect_stops_it(self, mock_mujoco: MagicMock) -> None:
        import http.client

        _ = mock_mujoco
        port = self._free_port()
        robot = MuJoCoSO101(model_path="/fake/model.xml", http_port=port)
        robot.connect()
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        try:
            assert robot._http_server is not None  # noqa: SLF001
            conn.request("GET", "/health")
            assert conn.getresponse().status == 200
        finally:
            conn.close()
            robot.disconnect()
        assert robot._http_server is None  # noqa: SLF001
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
        with pytest.raises(ConnectionRefusedError):
            conn.request("GET", "/health")

    def test_connect_with_busy_port_continues_without_http(self, mock_mujoco: MagicMock) -> None:
        import socket

        _ = mock_mujoco
        port = self._free_port()
        blocker = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        blocker.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        blocker.bind(("127.0.0.1", port))
        blocker.listen(1)
        try:
            robot = MuJoCoSO101(model_path="/fake/model.xml", http_port=port)
            robot.connect()
            assert robot._http_server is None  # noqa: SLF001
            robot.disconnect()
        finally:
            blocker.close()

    def test_http_disabled_by_default_port_zero(self, mock_mujoco: MagicMock) -> None:
        _ = mock_mujoco
        robot = MuJoCoSO101(model_path="/fake/model.xml")
        robot.connect()
        try:
            assert robot._http_server is None  # noqa: SLF001
        finally:
            robot.disconnect()
