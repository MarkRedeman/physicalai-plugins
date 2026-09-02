# ruff: noqa: DOC201, DOC501, D107, PLR2004

"""Minimal SocketCAN transport for OpenArm's documented Damiao MIT protocol."""

from __future__ import annotations

import contextlib
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

import can
from motorbridge import Controller, Mode

from physicalai_openarm_plugin.constants import (
    CAN_CMD_DISABLE,
    CAN_CMD_ENABLE,
    CAN_CMD_REFRESH,
    CAN_PARAM_ID,
    MOTOR_LIMITS,
)

if TYPE_CHECKING:
    from collections.abc import Mapping


@dataclass(frozen=True)
class MotorState:
    """Decoded Damiao motor state using degrees for position and velocity."""

    position: float
    velocity: float
    torque: float
    temp_mos: float
    temp_rotor: float


class DamiaoSocketCAN:
    """Own one SocketCAN interface and the fixed motors attached to it."""

    def __init__(
        self,
        port: str,
        motors: Mapping[str, tuple[int, int, str]],
        *,
        use_can_fd: bool = True,
        bitrate: int = 1_000_000,
        data_bitrate: int = 5_000_000,
        response_timeout: float = 0.02,
    ) -> None:
        self.port = port
        self.motors = dict(motors)
        self.use_can_fd = use_can_fd
        self.bitrate = bitrate
        self.data_bitrate = data_bitrate
        self.response_timeout = response_timeout
        self._bus: can.BusABC | None = None
        self._states: dict[str, MotorState] = {}

    @property
    def is_connected(self) -> bool:
        """Whether the SocketCAN bus is open."""
        return self._bus is not None

    def connect(self) -> None:
        """Open the bus and require one valid response from every configured motor."""
        if self.is_connected:
            return
        try:
            self._open_and_handshake()
        except Exception:
            self.disconnect(disable_torque=True)
            raise

    def _open_and_handshake(self) -> None:
        kwargs: dict[str, object] = {"interface": "socketcan", "channel": self.port, "bitrate": self.bitrate}
        if self.use_can_fd:
            kwargs.update(fd=True, data_bitrate=self.data_bitrate)
        self._bus = can.Bus(**kwargs)
        self._drain()
        self.enable_torque()
        self._states = self.read_states(require_all=True)

    def disconnect(self, *, disable_torque: bool) -> None:
        """Optionally disable torque and release the SocketCAN resource."""
        bus = self._bus
        if bus is None:
            return
        try:
            if disable_torque:
                with contextlib.suppress(Exception):
                    self.disable_torque()
        finally:
            self._bus = None
            bus.shutdown()

    def enable_torque(self) -> None:
        """Enable all configured motors and cache any immediate responses."""
        self._send_simple_command(CAN_CMD_ENABLE)

    def disable_torque(self) -> None:
        """Disable all configured motors and cache any immediate responses."""
        self._send_simple_command(CAN_CMD_DISABLE)

    def read_states(self, *, require_all: bool = True) -> dict[str, MotorState]:
        """Refresh all motors in one batch and decode their responses."""
        bus = self._require_bus()
        for send_id, _, _ in self.motors.values():
            bus.send(self._message(CAN_PARAM_ID, bytes((send_id, 0, CAN_CMD_REFRESH, 0, 0, 0, 0, 0))))
        states = self._receive_states()
        missing = set(self.motors) - set(states)
        if missing and require_all:
            msg = f"No state response from OpenArm motors: {', '.join(sorted(missing))} on {self.port}"
            raise ConnectionError(msg)
        self._states.update(states)
        return {name: self._states[name] for name in self.motors if name in self._states}

    def send_positions(self, commands: Mapping[str, tuple[float, float, float]]) -> None:
        """Send batch MIT commands of ``(kp, kd, target_degrees)`` to motors."""
        bus = self._require_bus()
        for name, (kp, kd, position_deg) in commands.items():
            send_id, _, motor_type = self.motors[name]
            bus.send(self._message(send_id, self._encode_mit(motor_type, kp, kd, position_deg)))
        self._states.update(self._receive_states())

    def _send_simple_command(self, command: int) -> None:
        bus = self._require_bus()
        for send_id, _, _ in self.motors.values():
            bus.send(self._message(send_id, bytes([0xFF] * 7 + [command])))
        self._states.update(self._receive_states())

    def _receive_states(self) -> dict[str, MotorState]:
        bus = self._require_bus()
        received: dict[str, MotorState] = {}
        recv_ids = {recv_id: (name, motor_type) for name, (_, recv_id, motor_type) in self.motors.items()}
        deadline = time.monotonic() + self.response_timeout
        while len(received) < len(self.motors):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            message = bus.recv(timeout=remaining)
            if message is None or message.arbitration_id not in recv_ids:
                continue
            name, motor_type = recv_ids[message.arbitration_id]
            received[name] = self._decode_state(bytes(message.data), motor_type)
        return received

    def _drain(self) -> None:
        bus = self._require_bus()
        while bus.recv(timeout=0.0) is not None:
            pass

    def _require_bus(self) -> can.BusABC:
        if self._bus is None:
            msg = "OpenArm is not connected. Call connect() first."
            raise ConnectionError(msg)
        return self._bus

    def _message(self, arbitration_id: int, data: bytes) -> can.Message:
        return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False, is_fd=self.use_can_fd)

    @staticmethod
    def _float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
        value = min(max(value, minimum), maximum)
        return int((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))

    @staticmethod
    def _uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
        return value * (maximum - minimum) / ((1 << bits) - 1) + minimum

    def _encode_mit(self, motor_type: str, kp: float, kd: float, position_deg: float) -> bytes:
        pmax, vmax, tmax = MOTOR_LIMITS[motor_type]
        q = self._float_to_uint(math.radians(position_deg), -pmax, pmax, 16)
        dq = self._float_to_uint(0.0, -vmax, vmax, 12)
        kp_value = self._float_to_uint(kp, 0.0, 500.0, 12)
        kd_value = self._float_to_uint(kd, 0.0, 5.0, 12)
        torque = self._float_to_uint(0.0, -tmax, tmax, 12)
        return bytes((
            q >> 8,
            q & 0xFF,
            dq >> 4,
            ((dq & 0xF) << 4) | (kp_value >> 8),
            kp_value & 0xFF,
            kd_value >> 4,
            ((kd_value & 0xF) << 4) | (torque >> 8),
            torque & 0xFF,
        ))

    def _decode_state(self, data: bytes, motor_type: str) -> MotorState:
        if len(data) < 8:
            msg = f"Invalid Damiao state frame length: {len(data)}"
            raise ValueError(msg)
        pmax, vmax, tmax = MOTOR_LIMITS[motor_type]
        position = self._uint_to_float((data[1] << 8) | data[2], -pmax, pmax, 16)
        velocity = self._uint_to_float((data[3] << 4) | (data[4] >> 4), -vmax, vmax, 12)
        torque = self._uint_to_float(((data[4] & 0xF) << 8) | data[5], -tmax, tmax, 12)
        return MotorState(math.degrees(position), math.degrees(velocity), torque, float(data[6]), float(data[7]))


class DamiaoSerial:
    """Experimental Damiao USB-CAN serial transport backed by ``motorbridge``."""

    def __init__(
        self,
        port: str,
        motors: Mapping[str, tuple[int, int, str]],
        *,
        baud: int = 921_600,
        _controller_factory: object | None = None,
    ) -> None:
        if baud <= 0:
            msg = "baud must be positive"
            raise ValueError(msg)
        self.port = port
        self.motors = dict(motors)
        self.baud = baud
        self._controller: Controller | None = None
        self._motors: dict[str, object] = {}
        self._controller_factory = _controller_factory or Controller.from_dm_serial

    @property
    def is_connected(self) -> bool:
        """Whether the Damiao USB-CAN serial controller is open."""
        return self._controller is not None

    def connect(self) -> None:
        """Open, register, validate, and enable all documented OpenArm motors."""
        if self.is_connected:
            return
        try:
            self._controller = self._controller_factory(serial_port=self.port, baud=self.baud)  # type: ignore[operator]
            self._motors = {
                name: self._controller.add_damiao_motor(send_id, recv_id, motor_type)
                for name, (send_id, recv_id, motor_type) in self.motors.items()
            }
            for motor in self._motors.values():
                motor.ensure_mode(Mode.MIT)
            self._controller.enable_all()
            self.read_states()
        except Exception:
            self.disconnect(disable_torque=True)
            raise

    def disconnect(self, *, disable_torque: bool) -> None:
        """Optionally disable torque, close all motor handles, and release the adapter."""
        controller = self._controller
        if controller is None:
            return
        try:
            if disable_torque:
                with contextlib.suppress(Exception):
                    controller.disable_all()
            for motor in self._motors.values():
                with contextlib.suppress(Exception):
                    motor.close()
        finally:
            self._controller = None
            self._motors = {}
            controller.close()

    def enable_torque(self) -> None:
        """Enable torque for all registered motors."""
        self._require_controller().enable_all()

    def disable_torque(self) -> None:
        """Disable torque for all registered motors."""
        self._require_controller().disable_all()

    def read_states(self) -> dict[str, MotorState]:
        """Poll and return states from every configured motor."""
        controller = self._require_controller()
        for motor in self._motors.values():
            motor.request_feedback()
        controller.poll_feedback_once()
        states: dict[str, MotorState] = {}
        for name, motor in self._motors.items():
            state = motor.get_state()
            if state is None:
                msg = f"No state response from OpenArm motor {name} on {self.port}"
                raise ConnectionError(msg)
            states[name] = MotorState(
                math.degrees(state.pos),
                math.degrees(state.vel),
                state.torq,
                state.t_mos,
                state.t_rotor,
            )
        return states

    def send_positions(self, commands: Mapping[str, tuple[float, float, float]]) -> None:
        """Send MIT position targets, converting the degree contract to radians."""
        for name, (kp, kd, position_deg) in commands.items():
            self._motors[name].send_mit(math.radians(position_deg), 0.0, kp, kd, 0.0)

    def _require_controller(self) -> Controller:
        if self._controller is None:
            msg = "OpenArm is not connected. Call connect() first."
            raise ConnectionError(msg)
        return self._controller
