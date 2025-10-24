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
    """Client responsible for talking with the PSTrotator helper.

    In addition to providing a low-level ``send_command`` helper the client
    can drive the virtual "PSTrotator ON/OFF" button exposed by the Windows
    application.  This makes it easy to mirror the GUI toggle from other
    interfaces such as a Tkinter button.
    """

    def __init__(self, config: Optional[PSTRotatorConfig] = None) -> None:
        self._config = config or PSTRotatorConfig()
        self._socket: Optional[socket.socket] = None
        self._enabled = False
        self._lock = threading.RLock()

    # ------------------------------------------------------------------
    # Communication lifecycle helpers
    # ------------------------------------------------------------------
    def enable_communication(self) -> None:
        """Open the TCP socket and press the "ON" button in PSTrotator."""

        with self._lock:
            if self._enabled:
                return

            sock = self._create_socket()
            try:
                self._socket = sock
                self._send_button_command("ON")
                self._enabled = True
            except Exception:
                with suppress(OSError):
                    sock.shutdown(socket.SHUT_RDWR)
                with suppress(OSError):
                    sock.close()
                self._socket = None
                raise

    def disable_communication(self) -> None:
        """Press the "OFF" button in PSTrotator and close the socket."""

        with self._lock:
            if not self._enabled and self._socket is None:
                return

            try:
                if self._socket is not None:
                    with suppress(OSError):
                        self._send_button_command("OFF")
            finally:
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

    def set_button_state(self, enabled: bool) -> None:
        """Explicitly set the PSTrotator ON/OFF button state."""

        if enabled:
            self.enable_communication()
        else:
            self.disable_communication()

    def _send_button_command(self, state: str) -> None:
        if self._socket is None:
            raise RuntimeError("No active socket to PSTrotator")

        message = f"PSTROTATOR {state}\r\n".encode("ascii")
        self._socket.sendall(message)

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
