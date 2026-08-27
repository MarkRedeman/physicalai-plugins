from __future__ import annotations

import argparse
import os
import signal
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from urllib.error import URLError

from physicalai_mujoco_so101_plugin import __main__ as cli
from physicalai_mujoco_so101_plugin.__main__ import _stop_owner_over_http
from physicalai_mujoco_so101_plugin.constants import (
    DEFAULT_BIMANUAL_MUJOCO_OWNER_NAME,
    DEFAULT_MUJOCO_OWNER_NAME,
)


class TestHttpOwnerName:
    @staticmethod
    def _response(body: bytes) -> MagicMock:
        response = MagicMock()
        response.__enter__.return_value = response
        response.read.return_value = body
        return response

    def test_returns_service_field(self) -> None:
        response = self._response(b'{"service": "mujoco-so101-bimanual"}')
        with patch("urllib.request.urlopen", return_value=response):
            assert cli._http_owner_name("127.0.0.1", 8080) == "mujoco-so101-bimanual"  # noqa: SLF001

    def test_unreachable_returns_none(self) -> None:
        with patch("urllib.request.urlopen", side_effect=URLError("refused")):
            assert cli._http_owner_name("127.0.0.1", 8080) is None  # noqa: SLF001

    def test_non_json_response_returns_none(self) -> None:
        response = self._response(b"not json")
        with patch("urllib.request.urlopen", return_value=response):
            assert cli._http_owner_name("127.0.0.1", 8080) is None  # noqa: SLF001


class TestPidOwnerName:
    @staticmethod
    def _ps_output(cmdline: str) -> SimpleNamespace:
        return SimpleNamespace(args=["ps"], returncode=0, stdout=f"{cmdline}\n", stderr="")

    def test_explicit_name_flag(self) -> None:
        cmdline = "physicalai-mujoco-so101 start --model x.xml --name my-sim --bimanual"
        with patch("subprocess.run", return_value=self._ps_output(cmdline)):
            assert cli._pid_owner_name(1234) == "my-sim"  # noqa: SLF001

    def test_bimanual_default_without_explicit_name(self) -> None:
        cmdline = "physicalai-mujoco-so101 start --model x.xml --bimanual"
        with patch("subprocess.run", return_value=self._ps_output(cmdline)):
            assert cli._pid_owner_name(1234) == DEFAULT_BIMANUAL_MUJOCO_OWNER_NAME  # noqa: SLF001

    def test_single_arm_default_without_explicit_name(self) -> None:
        cmdline = "physicalai-mujoco-so101 start --model x.xml"
        with patch("subprocess.run", return_value=self._ps_output(cmdline)):
            assert cli._pid_owner_name(1234) == DEFAULT_MUJOCO_OWNER_NAME  # noqa: SLF001

    def test_unreadable_command_line_returns_none(self) -> None:
        with patch("subprocess.run", side_effect=FileNotFoundError):
            assert cli._pid_owner_name(1234) is None  # noqa: SLF001


class TestStopOwnerOverHttp:
    def test_posts_shutdown(self) -> None:
        with patch("urllib.request.urlopen") as urlopen:
            _stop_owner_over_http("127.0.0.1", 8080)
        request = urlopen.call_args.args[0]
        assert request.full_url == "http://127.0.0.1:8080/shutdown"
        assert request.get_method() == "POST"
        assert urlopen.call_args.kwargs["timeout"] == 5

    def test_unreachable_is_silent(self) -> None:
        with patch("urllib.request.urlopen", side_effect=URLError("refused")):
            _stop_owner_over_http("127.0.0.1", 8080)

    def test_connection_error_is_silent(self) -> None:
        with patch("urllib.request.urlopen", side_effect=ConnectionError("refused")):
            _stop_owner_over_http("127.0.0.1", 8080)


class TestResolveOwnerName:
    def test_single_arm_default(self) -> None:
        args = argparse.Namespace(name=None, bimanual=False)
        assert cli._resolve_owner_name(args) == DEFAULT_MUJOCO_OWNER_NAME  # noqa: SLF001

    def test_bimanual_gets_its_own_default(self) -> None:
        args = argparse.Namespace(name=None, bimanual=True)
        assert cli._resolve_owner_name(args) == DEFAULT_BIMANUAL_MUJOCO_OWNER_NAME  # noqa: SLF001

    def test_explicit_name_wins(self) -> None:
        args = argparse.Namespace(name="my-sim", bimanual=True)
        assert cli._resolve_owner_name(args) == "my-sim"  # noqa: SLF001

    def test_defaults_differ(self) -> None:
        assert DEFAULT_MUJOCO_OWNER_NAME != DEFAULT_BIMANUAL_MUJOCO_OWNER_NAME


class TestMatchingPids:
    @staticmethod
    def _pgrep_output(stdout: str) -> SimpleNamespace:
        return SimpleNamespace(args=["pgrep"], returncode=0, stdout=stdout, stderr="")

    def test_own_process_and_parent_are_excluded(self) -> None:
        stdout = f"{os.getpid()}\n{os.getppid()}\n424242\n"
        with patch("subprocess.run", return_value=self._pgrep_output(stdout)):
            assert cli._matching_pids("physicalai-mujoco-so101 start") == [424242]  # noqa: SLF001

    def test_pattern_is_passed_to_pgrep(self) -> None:
        with patch("subprocess.run", return_value=self._pgrep_output("")) as run:
            cli._matching_pids("physicalai-mujoco-so101 start")  # noqa: SLF001

        assert run.call_args.args[0] == ["pgrep", "-f", "physicalai-mujoco-so101 start"]

    def test_non_numeric_output_is_ignored(self) -> None:
        with patch("subprocess.run", return_value=self._pgrep_output("nope\n7\n")):
            assert cli._matching_pids("x") == [7]  # noqa: SLF001


class TestStop:
    @staticmethod
    def _args(name: str = DEFAULT_MUJOCO_OWNER_NAME) -> argparse.Namespace:
        return argparse.Namespace(http_host="127.0.0.1", http_port=8080, name=name)

    def test_http_shutdown_used_when_no_local_owner(self) -> None:
        with (
            patch.object(cli, "_owner_pid", return_value=None),
            patch.object(cli, "_http_owner_name", return_value=DEFAULT_MUJOCO_OWNER_NAME),
            patch.object(cli, "_request_http_shutdown", return_value=True),
            patch.object(cli, "_matching_pids") as matching,
            patch("os.kill") as kill,
        ):
            cli._stop(self._args())  # noqa: SLF001

        matching.assert_not_called()
        kill.assert_not_called()

    def test_http_shutdown_skipped_when_owner_name_mismatches(self) -> None:
        """The HTTP port might belong to a different named owner; don't shut it down."""
        with (
            patch.object(cli, "_owner_pid", return_value=None),
            patch.object(cli, "_http_owner_name", return_value="some-other-owner"),
            patch.object(cli, "_request_http_shutdown") as http_shutdown,
            patch.object(cli, "_matching_pids", return_value=[]),
            patch("os.kill") as kill,
        ):
            cli._stop(self._args())  # noqa: SLF001

        http_shutdown.assert_not_called()
        kill.assert_not_called()

    def test_named_owner_is_preferred_over_http(self) -> None:
        """A local owner matching --name is stopped directly; HTTP is never tried."""
        with (
            patch.object(cli, "_owner_pid", return_value=4242),
            patch.object(cli, "_request_http_shutdown") as http_shutdown,
            patch.object(cli, "_matching_pids") as matching,
            patch("os.kill") as kill,
        ):
            cli._stop(self._args())  # noqa: SLF001

        http_shutdown.assert_not_called()
        matching.assert_not_called()
        kill.assert_called_once_with(4242, signal.SIGTERM)

    def test_named_owner_is_signalled(self) -> None:
        with (
            patch.object(cli, "_request_http_shutdown", return_value=False),
            patch.object(cli, "_owner_pid", return_value=4242),
            patch.object(cli, "_matching_pids", return_value=[]),
            patch("os.kill") as kill,
        ):
            cli._stop(self._args())  # noqa: SLF001

        kill.assert_called_once_with(4242, signal.SIGTERM)

    def test_pgrep_fallback_skips_mismatched_owner_names(self) -> None:
        """Two concurrent sims: stopping one by name must not kill the other."""
        with (
            patch.object(cli, "_owner_pid", return_value=None),
            patch.object(cli, "_http_owner_name", return_value=None),
            patch.object(cli, "_request_http_shutdown", return_value=False),
            patch.object(cli, "_matching_pids", return_value=[111, 222]),
            patch.object(cli, "_pid_owner_name", side_effect=lambda pid: "other-sim" if pid == 111 else "mujoco-so101"),
            patch("os.kill") as kill,
        ):
            cli._stop(self._args(name="mujoco-so101"))  # noqa: SLF001

        kill.assert_called_once_with(222, signal.SIGTERM)

    def test_only_start_invocations_are_matched(self) -> None:
        patterns = []

        def record(pattern: str) -> list[int]:
            patterns.append(pattern)
            return []

        with (
            patch.object(cli, "_request_http_shutdown", return_value=False),
            patch.object(cli, "_http_owner_name", return_value=None),
            patch.object(cli, "_owner_pid", return_value=None),
            patch.object(cli, "_matching_pids", side_effect=record),
            patch("os.kill") as kill,
        ):
            cli._stop(self._args())  # noqa: SLF001

        assert patterns == ["physicalai-mujoco-so101 start"]
        assert all("_owner_worker" not in pattern for pattern in patterns)
        kill.assert_not_called()

    def test_unsignalable_pid_is_reported_not_raised(self) -> None:
        with (
            patch.object(cli, "_request_http_shutdown", return_value=False),
            patch.object(cli, "_owner_pid", return_value=4242),
            patch.object(cli, "_matching_pids", return_value=[]),
            patch("os.kill", side_effect=ProcessLookupError),
        ):
            cli._stop(self._args())  # noqa: SLF001


class TestOwnerPid:
    def test_missing_lock_file_returns_none(self) -> None:
        assert cli._owner_pid("no-such-owner-name-xyz") is None  # noqa: SLF001
