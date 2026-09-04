from __future__ import annotations

import numpy as np
import pytest

from physicalai_stararm_plugin.motion import HoldPoseSource, JointLogger, SineWaveSource


class _Obs:
    def __init__(self, values: list[float]) -> None:
        self.joint_positions = np.asarray(values, dtype=np.float32)
        self.timestamp = 1.0


class _Event:
    def __init__(self, step: int, values: list[float]) -> None:
        self.step = step
        self.robot_state = _Obs(values)


def test_sine_wave_source_invalid_frequency() -> None:
    with pytest.raises(ValueError, match="frequency"):
        SineWaveSource(frequency=0.0)


def test_sine_wave_source_output_shape() -> None:
    source = SineWaveSource(amplitude=1.0, frequency=0.5)
    out = source.update(_Obs([0, 0, 0]), {}, 0)
    assert out.shape == (3,)


def test_hold_pose_source_echoes_observation() -> None:
    source = HoldPoseSource()
    values = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    out = source.update(_Obs(values.tolist()), {}, 0)
    np.testing.assert_allclose(out, values)


def test_joint_logger_rejects_nonpositive_throttle() -> None:
    with pytest.raises(ValueError, match="throttle_steps"):
        JointLogger(0)


def test_joint_logger_prints(capsys: pytest.CaptureFixture[str]) -> None:
    logger = JointLogger(throttle_steps=2)
    logger.on_tick(_Event(step=1, values=[1.0, 2.0]))
    assert capsys.readouterr().out == ""

    logger.on_tick(_Event(step=2, values=[1.0, 2.0]))
    assert "1.00" in capsys.readouterr().out
