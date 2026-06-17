import socket
import threading
import time
from typing import Optional


DELIM = b"\n"


class TcpClient:
    """
    Thread-safe TCP client for delimiter-terminated C4G messages.

    Each message is terminated by LF (\\n).
    """

    def __init__(
        self,
        host: str,
        port: int,
        connect_timeout_s: float = 3.0,
        io_timeout_s: float = 0.5,
        recv_chunk_size: int = 1024,
        max_message_bytes: int = 4096,
        max_send_bytes: int = 254,
        keepalive: bool = True,
        debug: bool = False,
    ) -> None:
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.io_timeout_s = io_timeout_s
        self.recv_chunk_size = recv_chunk_size
        self.max_message_bytes = max_message_bytes
        self.max_send_bytes = max_send_bytes
        self.keepalive = keepalive
        self.debug = debug

        self._sock: Optional[socket.socket] = None
        self._rx_buf = bytearray()
        self._lock = threading.Lock()

    def is_connected(self) -> bool:
        with self._lock:
            return self._sock is not None

    def close(self) -> None:
        with self._lock:
            sock = self._sock
            self._sock = None
            self._rx_buf.clear()

        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

            try:
                sock.close()
            except OSError:
                pass

    def _install_socket(self, sock: socket.socket) -> None:
        sock.settimeout(self.io_timeout_s)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        if self.keepalive:
            try:
                sock.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_KEEPALIVE,
                    1,
                )
            except OSError:
                pass

        with self._lock:
            self._sock = sock
            self._rx_buf.clear()

    def connect(self) -> None:
        self.close()

        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )
        sock.settimeout(self.connect_timeout_s)

        try:
            sock.connect((self.host, self.port))
        except OSError as exc:
            try:
                sock.close()
            except OSError:
                pass

            raise RuntimeError(
                f"TCP connect failed to "
                f"{self.host}:{self.port}: {exc}"
            ) from exc

        self._install_socket(sock)

    def ensure_connected(
        self,
        max_attempts: int = 10,
        backoff_s: float = 0.5,
    ) -> None:
        if self.is_connected():
            return

        last_error: Exception | None = None

        for attempt in range(max_attempts):
            try:
                self.connect()
                return

            except Exception as exc:
                last_error = exc

                if attempt < max_attempts - 1:
                    time.sleep(backoff_s * (attempt + 1))

        raise RuntimeError(
            f"Failed to connect after {max_attempts} attempts: "
            f"{last_error}"
        )

    def send_msg(self, message: str) -> None:
        """
        Send one ASCII message terminated by LF.
        """
        try:
            payload = message.encode(
                "ascii",
                errors="strict",
            )
        except UnicodeEncodeError as exc:
            raise ValueError(
                "C4G command must contain ASCII characters only"
            ) from exc

        if payload.endswith(DELIM):
            raw_message = payload[:-len(DELIM)]
        else:
            raw_message = payload

        if len(raw_message) > self.max_send_bytes:
            raise ValueError(
                f"Message is too long: {len(raw_message)} bytes; "
                f"maximum is {self.max_send_bytes}"
            )

        data = raw_message + DELIM

        if self.debug:
            print(
                "TX bytes:",
                data,
                "hex=",
                data.hex(),
                flush=True,
            )

        with self._lock:
            if self._sock is None:
                raise RuntimeError("TCP not connected")

            sock = self._sock

        try:
            sock.sendall(data)

        except OSError as exc:
            self.close()
            raise RuntimeError(
                f"TCP send failed: {exc}"
            ) from exc

    def recv_msg(self) -> str:
        """
        Receive exactly one delimiter-terminated message.

        TCP is a byte stream. Data is accumulated until LF is found.
        The function never treats an incomplete recv chunk as a complete
        message.
        """
        while True:
            with self._lock:
                if self._sock is None:
                    raise RuntimeError("TCP not connected")

                delimiter_index = self._rx_buf.find(DELIM)

                if delimiter_index != -1:
                    raw = bytes(
                        self._rx_buf[:delimiter_index]
                    )

                    del self._rx_buf[
                        :delimiter_index + len(DELIM)
                    ]

                    return raw.decode(
                        "ascii",
                        errors="replace",
                    ).strip("\x00\r\n\x08 ")

                sock = self._sock

            try:
                chunk = sock.recv(self.recv_chunk_size)

                if self.debug:
                    print(
                        "RX chunk:",
                        chunk,
                        "hex=",
                        chunk.hex(),
                        flush=True,
                    )

            except socket.timeout as exc:
                raise RuntimeError(
                    "TCP recv timeout"
                ) from exc

            except OSError as exc:
                self.close()
                raise RuntimeError(
                    f"TCP recv failed: {exc}"
                ) from exc

            if not chunk:
                self.close()
                raise RuntimeError(
                    "TCP connection closed by peer"
                )

            with self._lock:
                if self._sock is None:
                    raise RuntimeError("TCP not connected")

                self._rx_buf.extend(chunk)

                delimiter_index = self._rx_buf.find(DELIM)

                if delimiter_index != -1:
                    raw = bytes(
                        self._rx_buf[:delimiter_index]
                    )

                    del self._rx_buf[
                        :delimiter_index + len(DELIM)
                    ]

                    return raw.decode(
                        "ascii",
                        errors="replace",
                    ).strip("\x00\r\n\x08 ")

                if len(self._rx_buf) > self.max_message_bytes:
                    self.close()
                    raise RuntimeError(
                        "Received message exceeds "
                        f"{self.max_message_bytes} bytes"
                    )
                
                # C4G compatibility:
                # no delimiter was transmitted, but the current recv chunk
                # contains one complete controller message.
                raw = bytes(self._rx_buf)
                self._rx_buf.clear()

                return raw.decode(
                    "ascii",
                    errors="replace",
                ).strip("\x00\r\n\x08 ")


