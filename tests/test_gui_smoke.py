"""GUI のスモークテスト（ヘッドレスで起動して落ちないこと）."""

import pytest

pytest.importorskip("PySide6.QtWidgets")

from PySide6.QtWidgets import QApplication  # noqa: E402

from coco_link.common.config import NetworkConfig  # noqa: E402
from coco_link.common.geometry import Pose2D  # noqa: E402
from coco_link.fleet.control_core import ControlCore  # noqa: E402
from coco_link.sim.engine import SimConfig, SimEngine  # noqa: E402
from coco_link.sim.runner import SimRunner  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    return QApplication.instance() or QApplication([])


def test_simulator_window(qapp):
    from coco_link.apps.simulator.main_window import SimulatorWindow
    e = SimEngine(SimConfig())
    e.add_robot(1, Pose2D(1, 1, 0))
    e.add_body("person", 2, 1)
    runner = SimRunner(e, network=False).start()
    w = SimulatorWindow(e, runner)
    w.show()
    for _ in range(10):
        w._refresh()
        qapp.processEvents()
    assert not w.grab().isNull()
    w.close()


def test_operator_window(qapp):
    from coco_link.apps.operator.__main__ import build_window
    core = ControlCore(NetworkConfig(bind_host="127.0.0.1", operator_robot_port=0, operator_vision_port=0))
    core.start(thread=False)
    try:
        w = build_window(core)
        w.show()
        for name in core.modes:
            w.mode_panel.combo.setCurrentText(name)    # 全モードのフォームが生成できる
            assert w.mode_panel.form is not None
            w.mode_panel.form.values()
        core.tick()
        w._refresh()
        qapp.processEvents()
        assert not w.grab().isNull()
        w.close()
    finally:
        core.close()
