from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import zmq
from loguru import logger

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation


@dataclass
class ZMQRobotObservation:
    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None

    @property
    def state(self) -> np.ndarray:
        return self.joint_positions


class ZMQRobot:
    """Robot client that proxies commands to a remote robot via ZMQ.

    Implements the ``physicalai.robot.Robot`` protocol using ZMQ REQ/REP
    sockets for synchronous request-response communication.

    The remote robot server is expected to support these commands:
        - ``features`` — returns available joint names.
        - ``read_state`` — returns current joint positions.
        - ``set_joints_state`` — sends target joint positions.
        - ``enable_torque`` / ``disable_torque`` — torque control.
        - ``ping`` — health check.
    """

    def __init__(
        self,
        zmq_endpoint: str,
        *,
        command_timeout: float = 5.0,
    ) -> None:
        self._endpoint = zmq_endpoint
        self._command_timeout = int(command_timeout * 1000)
        self._context: zmq.Context | None = None
        self._socket: zmq.Socket | None = None
        self._joint_names: list[str] = []

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def connect(self) -> None:
        if self.is_connected():
            return

        logger.info("Connecting to ZMQ robot at {}", self._endpoint)
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, self._command_timeout)
        self._socket.setsockopt(zmq.LINGER, 1000)
        self._socket.connect(self._endpoint)

        resp = self._command("features", {})
        self._joint_names = resp.get("features") or resp.get("payload", {}).get("features", [])
        if not self._joint_names:
            logger.warning("Remote robot returned no features; joint_names will be empty")

        logger.info("Connected to ZMQ robot at {} ({} joints)", self._endpoint, len(self._joint_names))

    def disconnect(self) -> None:
        if self._socket is None:
            return

        logger.info("Disconnecting from ZMQ robot at {}", self._endpoint)
        self._socket.close()
        self._context.term()
        self._socket = None
        self._context = None

    def is_connected(self) -> bool:
        return self._socket is not None

    def get_observation(self) -> RobotObservation:
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)

        resp = self._command("read_state", {})
        state = resp.get("state") or resp.get("payload", {}).get("state", {})

        positions = np.array([state.get(j, 0.0) for j in self._joint_names], dtype=np.float32)

        return ZMQRobotObservation(
            joint_positions=positions,
            timestamp=time.monotonic(),
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        if not self.is_connected():
            msg = "Robot is not connected. Call connect() first."
            raise ConnectionError(msg)

        joints = {name: float(action[i]) for i, name in enumerate(self._joint_names)}
        self._command("set_joints_state", {"joints": joints, "goal_time": goal_time})

    def enable_torque(self) -> None:
        self._command("enable_torque", {})

    def disable_torque(self) -> None:
        self._command("disable_torque", {})

    def _command(self, command: str, payload: dict) -> dict:
        if self._socket is None:
            msg = "Not connected"
            raise ConnectionError(msg)

        request = json.dumps({"command": command, "payload": payload})
        self._socket.send_string(request)

        try:
            response_str = self._socket.recv_string()
        except zmq.ZMQError as exc:
            msg = f"ZMQ error sending '{command}': {exc}"
            raise ConnectionError(msg) from exc

        return json.loads(response_str)
