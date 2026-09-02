from __future__ import annotations

import math
from unittest.mock import MagicMock

from physicalai_openarm_plugin.damiao import DamiaoSerial


def test_damiao_serial_registers_openarm_motors_and_sends_mit_targets() -> None:
    controller = MagicMock()
    motors = {name: MagicMock() for name in ("joint_1", "gripper")}
    controller.add_damiao_motor.side_effect = motors.values()
    for motor in motors.values():
        motor.get_state.return_value = MagicMock(pos=0.0, vel=0.0, torq=0.0, t_mos=30.0, t_rotor=31.0)
    factory = MagicMock(return_value=controller)
    transport = DamiaoSerial(
        "/dev/ttyACM0",
        {"joint_1": (1, 17, "dm8009"), "gripper": (8, 24, "dm4310")},
        _controller_factory=factory,
    )

    transport.connect()
    transport.send_positions({"joint_1": (240.0, 5.0, 45.0)})
    transport.disconnect(disable_torque=True)

    factory.assert_called_once_with(serial_port="/dev/ttyACM0", baud=921_600)
    assert controller.add_damiao_motor.call_count == 2
    controller.enable_all.assert_called_once()
    motors["joint_1"].send_mit.assert_called_once_with(math.radians(45.0), 0.0, 240.0, 5.0, 0.0)
    controller.disable_all.assert_called_once()
    controller.close.assert_called_once()
