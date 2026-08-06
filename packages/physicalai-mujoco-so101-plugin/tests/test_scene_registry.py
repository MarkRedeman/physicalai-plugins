from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from physicalai_mujoco_so101_plugin.scene_registry import (
    get_reset_fn,
    get_scene,
    list_scenes,
)


class TestGarmentFoldScene:
    def test_scene_registered(self) -> None:
        scene = get_scene("garment_fold")
        assert scene.scene_id == "garment_fold"
        assert scene.scene_xml_relpath == "scenes/garment_fold/scene.xml"
        assert scene.free_joints == ()
        assert scene.target_bodies == ()

    def test_reset_fn_registered(self) -> None:
        assert get_reset_fn("garment_fold") is not None

    def test_list_scenes_contains_garment_fold(self) -> None:
        assert "garment_fold" in list_scenes()

    def test_unknown_scene_raises(self) -> None:
        with pytest.raises(KeyError, match="Unknown scene"):
            get_scene("nope")


class TestGarmentFoldReset:
    def test_reset_sets_home_and_flex(self) -> None:
        mock_mujoco = MagicMock()
        mock_mujoco.mj_name2id.side_effect = lambda _model, _obj_type, name: {
            "left_shoulder_pan": 0,
            "left_shoulder_lift": 1,
            "left_elbow_flex": 2,
            "left_wrist_flex": 3,
            "right_shoulder_pan": 4,
            "right_shoulder_lift": 5,
            "right_elbow_flex": 6,
            "right_wrist_flex": 7,
        }.get(name, -1)
        mock_mujoco.mjtObj = MagicMock()

        with patch.dict("sys.modules", {"mujoco": mock_mujoco}):
            nflexvert = 196
            model = MagicMock()
            model.nflex = 1
            model.nflexvert = nflexvert
            model.njnt = 8 + 3 * nflexvert
            model.jnt_bodyid = [100, 101, 102, 103, 104, 105, 106, 107, *([200] * (3 * nflexvert))]
            model.jnt_qposadr = [0, 1, 2, 3, 4, 5, 6, 7, *range(8, 8 + 3 * nflexvert)]
            model.jnt_dofadr = [0, 1, 2, 3, 4, 5, 6, 7, *range(8, 8 + 3 * nflexvert)]
            model.flex_vertbodyid = [200] * nflexvert
            model.flex_vert = np.tile(np.array([0.0, 0.02, 0.322]), (nflexvert, 1))

            qpos = np.zeros(8 + 3 * nflexvert)
            qvel = np.ones(8 + 3 * nflexvert)
            ctrl = np.zeros(8)
            data = MagicMock()
            data.qpos = qpos
            data.qvel = qvel
            data.ctrl = ctrl

            fn = get_reset_fn("garment_fold")
            assert fn is not None
            fn(model, data, np.random.default_rng(0))

            assert data.qpos[0] == pytest.approx(-1.1)
            assert data.qpos[3] == pytest.approx(0.3)
            assert data.qpos[4] == pytest.approx(1.1)
            assert data.qpos[7] == pytest.approx(0.3)
            np.testing.assert_allclose(data.qpos[8:], np.tile([0.0, 0.02, 0.322], nflexvert))
            assert np.all(data.qvel[8:] == 0.0)
            assert data.ctrl[0] == pytest.approx(-1.1)
            mock_mujoco.mj_forward.assert_called_once_with(model, data)

    def test_reset_without_flex_is_noop(self) -> None:
        mock_mujoco = MagicMock()
        mock_mujoco.mjtObj = MagicMock()
        mock_mujoco.mj_name2id.return_value = -1

        with patch.dict("sys.modules", {"mujoco": mock_mujoco}):
            model = MagicMock()
            model.nflex = 0
            data = MagicMock()
            fn = get_reset_fn("garment_fold")
            assert fn is not None
            fn(model, data, np.random.default_rng(0))
