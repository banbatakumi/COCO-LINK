"""world_state を operator へ送信する."""

from __future__ import annotations

from ..common.config import NetworkConfig
from ..net import UdpEndpoint
from ..protocol import messages as m


class VisionPublisher:
    def __init__(self, net: NetworkConfig):
        self.net = net
        self.endpoint = UdpEndpoint("vision", net.bind_host, 0, name="vision-tx")

    def send(self, ws: m.WorldState) -> bool:
        return self.endpoint.send(ws, (self.net.operator_host, self.net.operator_vision_port))

    def close(self) -> None:
        self.endpoint.close()
