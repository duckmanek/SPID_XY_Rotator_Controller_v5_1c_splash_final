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


class _EchoHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:  # pragma: no cover - not critical for tests
        try:
            while self.request.recv(1024):
                pass
        except ConnectionResetError:
            pass


def _start_server() -> tuple[socketserver.TCPServer, threading.Thread]:
    server = socketserver.TCPServer(("127.0.0.1", 0), _EchoHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    # Give the server a moment to start
    time.sleep(0.05)
    return server, thread


def test_disable_communication_closes_socket() -> None:
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

        with pytest.raises(RuntimeError):
            client.send_command(b"test")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


def test_disable_communication_is_idempotent() -> None:
    server, thread = _start_server()
    config = PSTRotatorConfig(host="127.0.0.1", port=server.server_address[1])
    client = PSTRotatorClient(config=config)

    try:
        client.enable_communication()
        client.disable_communication()
        # second call should not raise
        client.disable_communication()
        assert not client.is_enabled
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)
