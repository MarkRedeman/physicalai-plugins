from __future__ import annotations

import math

from physicalai_openarm_plugin.damiao import DamiaoSocketCAN


def test_mit_position_round_trip_and_state_decode() -> None:
    transport = DamiaoSocketCAN("can0", {"joint_1": (1, 17, "dm8009")})
    frame = transport._encode_mit("dm8009", 240.0, 5.0, 45.0)  # noqa: SLF001
    assert len(frame) == 8

    state = transport._decode_state(  # noqa: SLF001
        bytes((0, 0x80, 0x00, 0x80, 0x08, 0x00, 30, 31)),
        "dm8009",
    )
    assert math.isclose(state.position, 0.0, abs_tol=0.1)
    assert state.temp_mos == 30.0
    assert state.temp_rotor == 31.0
