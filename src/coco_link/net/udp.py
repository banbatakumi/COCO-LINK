"""スレッド型 UDP エンドポイント（Qt 非依存）.

受信は専用スレッドで行い、受信ごとにコールバック `on_message(envelope, addr)` を呼ぶ。
コールバックは受信スレッドから呼ばれるので、共有データの更新はロックで保護すること。
"""

from __future__ import annotations

import contextlib
import logging
import socket
import sys
import threading
from collections.abc import Callable

from ..protocol import Envelope, Message, ProtocolError, Sender, decode

log = logging.getLogger(__name__)

Address = tuple[str, int]
MessageHandler = Callable[[Envelope, Address], None]


class UdpEndpoint:
    """1つの UDP ソケット（送受信兼用）."""

    def __init__(self, src: str, bind_host: str = "0.0.0.0", bind_port: int = 0,
                 on_message: MessageHandler | None = None, name: str = "udp"):
        self.sender = Sender(src)
        self.on_message = on_message
        self.name = name
        self.rx_count = 0
        self.rx_errors = 0
        self.tx_count = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        if sys.platform == "win32":
            # Windows では送信先ポートが閉じていると recvfrom が WSAECONNRESET を投げ続ける。無効化する。
            with contextlib.suppress(AttributeError, OSError):
                self._sock.ioctl(socket.SIO_UDP_CONNRESET, False)  # type: ignore[attr-defined]
        self._sock.bind((bind_host, bind_port))
        self._sock.settimeout(0.2)
        self._thread: threading.Thread | None = None
        self._running = False

    @property
    def port(self) -> int:
        return self._sock.getsockname()[1]

    def start(self) -> UdpEndpoint:
        if self._thread is None:
            self._running = True
            self._thread = threading.Thread(target=self._loop, name=f"{self.name}-rx", daemon=True)
            self._thread.start()
        return self

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._sock.close()

    def send(self, msg: Message, addr: Address) -> bool:
        try:
            self._sock.sendto(self.sender.encode(msg), addr)
            self.tx_count += 1
            return True
        except (OSError, ProtocolError) as e:
            log.debug("send to %s failed: %s", addr, e)
            return False

    def _loop(self) -> None:
        while self._running:
            try:
                raw, addr = self._sock.recvfrom(65535)
            except TimeoutError:
                continue
            except ConnectionResetError:  # Windows の ICMP port unreachable
                continue
            except OSError:
                if not self._running:
                    break
                continue
            try:
                env = decode(raw)
            except ProtocolError as e:
                self.rx_errors += 1
                log.debug("drop packet from %s: %s", addr, e)
                continue
            self.rx_count += 1
            if self.on_message is not None:
                try:
                    self.on_message(env, addr)
                except Exception:  # コールバックの例外で受信スレッドを殺さない
                    log.exception("on_message handler failed")
