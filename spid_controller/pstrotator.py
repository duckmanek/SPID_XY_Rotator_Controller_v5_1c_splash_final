"""Utilities for communicating with the PSTrotator application."""

from __future__ import annotations

import socket
import threading
from contextlib import suppress
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PSTRotatorConfig:
    """Configuration required to communicate with PSTrotator."""

    host: str = "127.0.0.1"
    port: int = 12000
    timeout: float = 5.0


class PSTRotatorClient:
    """Client responsible for talking with the PSTrotator helper."""

    def __init__(self, config: Optional[PSTRotatorConfig] = None) -> None:
        self._config = config or PSTRotatorConfig()
        self._socket: Optional[socket.socket] = None
        self._enabled = False
        self._lock = threading.RLock()

    # ------------------------------------------------------------------
    # Communication lifecycle helpers
    # ------------------------------------------------------------------
    def enable_communication(self) -> None:
        """Open the TCP socket to PSTrotator if it is not already open."""

        with self._lock:
            if self._enabled:
                return

            self._socket = self._create_socket()
            self._enabled = True

    def disable_communication(self) -> None:
        """Terminate the PSTrotator connection and block further commands."""

        with self._lock:
            self._enabled = False
            if self._socket is not None:
                with suppress(OSError):
                    self._socket.shutdown(socket.SHUT_RDWR)
                with suppress(OSError):
                    self._socket.close()
                self._socket = None

    # ------------------------------------------------------------------
    # Information helpers
    # ------------------------------------------------------------------
    @property
    def config(self) -> PSTRotatorConfig:
        return self._config

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def send_command(self, payload: bytes) -> None:
        """Send a raw command to PSTrotator."""

        with self._lock:
            if not self._enabled or self._socket is None:
                raise RuntimeError(
                    "Communication with PSTrotator is disabled. Call"
                    " 'enable_communication' before sending commands."
                )

            self._socket.sendall(payload)

    def _create_socket(self) -> socket.socket:
        return socket.create_connection(
            (self._config.host, self._config.port),
            timeout=self._config.timeout,
        )

    # ------------------------------------------------------------------
    # Context manager interface
    # ------------------------------------------------------------------
    def __enter__(self) -> "PSTRotatorClient":
        self.enable_communication()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[override]
        self.disable_communication()
