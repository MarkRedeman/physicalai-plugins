from __future__ import annotations

from unittest.mock import patch
from urllib.error import URLError

from physicalai_mujoco_so101_plugin.__main__ import _stop_owner_over_http


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
