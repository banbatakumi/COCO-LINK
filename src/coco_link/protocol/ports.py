"""既定ポート番号 (docs/protocol.md §2). 実際の値は config/network.yaml で上書きされる."""

OPERATOR_ROBOT_PORT = 50000
OPERATOR_VISION_PORT = 50001
ROBOT_PORT = 50100


def sim_robot_port(robot_id: int, base: int = ROBOT_PORT) -> int:
    """仮想ロボットの受信ポート."""
    return base + robot_id
