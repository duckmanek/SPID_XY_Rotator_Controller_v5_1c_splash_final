from __future__ import annotations

import socketserver
import threading
import time
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from spid_controller.pstrotator import PSTRotatorClient, PSTRotatorConfig


class _RecordingServer(socketserver.TCPServer):
    allow_reuse_address = True

    def __init__(self, server_address, RequestHandlerClass):
        super().__init__(server_address, RequestHandlerClass)
        self.messages: list[bytes] = []


class _RecordingHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:  # pragma: no cover - not critical for tests
        while True:
            data = self.request.recv(1024)
            if not data:
                break
            self.server.messages.append(data)  # type: ignore[attr-defined]


def _start_server() -> tuple[_RecordingServer, threading.Thread]:
    server = _RecordingServer(("127.0.0.1", 0), _RecordingHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    # Give the server a moment to start
    time.sleep(0.05)
    return server, thread


def _wait_for_fragment(
    server: _RecordingServer, fragment: bytes, timeout: float = 1.0
) -> None:
    deadline = time.monotonic() + timeout
    while fragment not in b"".join(server.messages) and time.monotonic() < deadline:
        time.sleep(0.01)


def test_enable_communication_presses_on_button() -> None:
    server, thread = _start_server()
    config = PSTRotatorConfig(host="127.0.0.1", port=server.server_address[1])
    client = PSTRotatorClient(config=config)

    try:
        client.enable_communication()
        assert client.is_enabled
        assert client._socket is not None  # noqa: SLF001 - testing internals

        client.disable_communication()
        assert not client.is_enabled
        assert client._socket is None  # noqa: SLF001 - ensure socket cleared

        _wait_for_fragment(server, b"PSTROTATOR ON\r\n")
        _wait_for_fragment(server, b"PSTROTATOR OFF\r\n")
        data = b"".join(server.messages)
        assert data.startswith(b"PSTROTATOR ON\r\n")
        assert data.endswith(b"PSTROTATOR OFF\r\n")

        with pytest.raises(RuntimeError):
            client.send_command(b"test")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


def test_disable_communication_sends_off_and_is_idempotent() -> None:
    server, thread = _start_server()
    config = PSTRotatorConfig(host="127.0.0.1", port=server.server_address[1])
    client = PSTRotatorClient(config=config)

    try:
        client.enable_communication()
        client.disable_communication()
        # second call should not raise
        client.disable_communication()
        assert not client.is_enabled

        _wait_for_fragment(server, b"PSTROTATOR OFF\r\n")
        data = b"".join(server.messages)
        assert data.count(b"PSTROTATOR OFF\r\n") == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)
