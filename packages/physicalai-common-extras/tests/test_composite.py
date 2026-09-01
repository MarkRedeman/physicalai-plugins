"""Tests for physicalai_common_extras.CompositeSource / CompositeChannel."""

from __future__ import annotations

import numpy as np
import pytest
from physicalai.config import to_config

from physicalai_common_extras import CompositeChannel, CompositeSource, HoldPoseSource, SineWaveSource


class _FakeObservation:
    """Minimal observation with a joint_positions array."""

    def __init__(self, joint_positions: np.ndarray) -> None:
        self.joint_positions = joint_positions


class _FakeSource:
    """Minimal action source for CompositeSource tests."""

    def __init__(self, action: np.ndarray) -> None:
        self._action = np.asarray(action, dtype=np.float32)
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.last_update: tuple[object, object, int] | None = None

    def connect(self, *, bus: object, session_id: str) -> None:
        self.connect_calls += 1

    def update(self, robot_state: object, camera_frames: object, step: int) -> np.ndarray:
        self.last_update = (robot_state, camera_frames, step)
        return self._action

    def disconnect(self) -> None:
        self.disconnect_calls += 1


def _obs(n: int = 5) -> _FakeObservation:
    return _FakeObservation(np.zeros(n, dtype=np.float32))


class TestCompositeSource:
    def test_update_scatters_each_source_into_its_indices(self) -> None:
        a = _FakeSource(np.array([1.0, 2.0]))
        b = _FakeSource(np.array([3.0, 4.0, 5.0]))
        composite = CompositeSource(
            channels=[
                CompositeChannel(source=a, indices=[0, 3]),
                CompositeChannel(source=b, indices=[1, 2, 4]),
            ],
        )

        action = composite.update(_obs(), {"cam": object()}, 2)

        np.testing.assert_allclose(action, [1.0, 3.0, 4.0, 2.0, 5.0])

    def test_update_forwards_args_to_each_source(self) -> None:
        a = _FakeSource(np.zeros(2, dtype=np.float32))
        b = _FakeSource(np.zeros(3, dtype=np.float32))
        composite = CompositeSource(
            channels=[
                CompositeChannel(source=a, indices=[0, 1]),
                CompositeChannel(source=b, indices=[2, 3, 4]),
            ],
        )
        obs = _obs()
        frames = {"cam": object()}

        composite.update(obs, frames, 7)

        assert a.last_update == (obs, frames, 7)
        assert b.last_update == (obs, frames, 7)

    def test_disjoint_indices_are_validated_at_construction(self) -> None:
        a = _FakeSource(np.zeros(2, dtype=np.float32))
        b = _FakeSource(np.zeros(2, dtype=np.float32))
        with pytest.raises(ValueError, match="disjoint"):
            CompositeSource(
                channels=[
                    CompositeChannel(source=a, indices=[0, 1]),
                    CompositeChannel(source=b, indices=[1, 2]),
                ],
            )

    def test_missing_joint_coverage_raises(self) -> None:
        source = _FakeSource(np.zeros(3, dtype=np.float32))
        composite = CompositeSource(channels=[CompositeChannel(source=source, indices=[0, 1, 2])])
        with pytest.raises(ValueError, match="missing joints"):
            composite.update(_obs(n=4), {}, 0)

    def test_out_of_range_indices_raise(self) -> None:
        source = _FakeSource(np.zeros(2, dtype=np.float32))
        composite = CompositeSource(channels=[CompositeChannel(source=source, indices=[0, 9])])
        with pytest.raises(ValueError, match="out-of-range"):
            composite.update(_obs(n=4), {}, 0)

    def test_channel_output_length_mismatch_raises(self) -> None:
        source = _FakeSource(np.zeros(3, dtype=np.float32))
        composite = CompositeSource(channels=[CompositeChannel(source=source, indices=[0, 1])])
        with pytest.raises(ValueError, match="produced 3 values"):
            composite.update(_obs(n=2), {}, 0)

    def test_connect_forwards_to_every_source(self) -> None:
        a = _FakeSource(np.zeros(2, dtype=np.float32))
        b = _FakeSource(np.zeros(3, dtype=np.float32))
        composite = CompositeSource(
            channels=[
                CompositeChannel(source=a, indices=[0, 1]),
                CompositeChannel(source=b, indices=[2, 3, 4]),
            ],
        )

        composite.connect(bus=object(), session_id="sess")

        assert a.connect_calls == 1
        assert b.connect_calls == 1

    def test_disconnect_forwards_to_every_source(self) -> None:
        a = _FakeSource(np.zeros(2, dtype=np.float32))
        b = _FakeSource(np.zeros(3, dtype=np.float32))
        composite = CompositeSource(
            channels=[
                CompositeChannel(source=a, indices=[0, 1]),
                CompositeChannel(source=b, indices=[2, 3, 4]),
            ],
        )

        composite.disconnect()

        assert a.disconnect_calls == 1
        assert b.disconnect_calls == 1

    def test_export_config_roundtrip(self) -> None:
        composite = CompositeSource(
            channels=[
                CompositeChannel(source=SineWaveSource(frequency=0.5), indices=[0, 1]),
                CompositeChannel(source=HoldPoseSource(), indices=[2, 3]),
            ],
        )

        cfg = to_config(composite)

        assert cfg["class_path"] == "physicalai_common_extras.CompositeSource"
        channels = cfg["init_args"]["channels"]
        assert channels[0]["class_path"] == "physicalai_common_extras.CompositeChannel"
        assert channels[0]["init_args"]["indices"] == [0, 1]
        assert channels[0]["init_args"]["source"]["class_path"] == "physicalai_common_extras.SineWaveSource"
        assert channels[1]["init_args"]["source"]["class_path"] == "physicalai_common_extras.HoldPoseSource"
