"""COCO-LINK 通信プロトコル v1 (docs/protocol.md)."""

from . import messages as _messages
from .codec import MAX_DATAGRAM, Envelope, ProtocolError, Sender, decode, encode
from .messages import *  # noqa: F403

__all__ = [*_messages.__all__, "MAX_DATAGRAM", "Envelope", "ProtocolError", "Sender", "decode", "encode"]
