import math
from dataclasses import fields

from coco_link.common.geometry import Pose2D
from coco_link.modes import MODES, discover
from coco_link.modes.follow_person import FollowPersonMode
from coco_link.modes.formation import FormationMode
from coco_link.sim.engine import SimConfig, SimEngine
from coco_link.sim.scenario import load_scenario

from .helpers import run_closed_loop


def test_discover_registers_modes():
    discover()
    assert {"待機", "フォーメーション", "人追従（隊列）"} <= set(MODES)


def test_all_params_have_labels_and_defaults():
    for cls in discover().values():
        for f in fields(cls.Params):
            assert f.metadata.get("label"), (cls.name, f.name)


def test_formation_converges_with_individual_differences():
    e = SimEngine(SimConfig(seed=1, param_spread=0.10))
    load_scenario(e, "scenarios/demo_formation.yaml")
    e.bodies.clear()   # 障害物なし
    mode = FormationMode(FormationMode.Params(shape="circle", spacing=0.3, center_x=1.5, center_y=1.0))
    run_closed_loop(e, mode, 25.0)
    for rid, target in mode.viz.targets.items():
        assert e.robots[rid].pose.distance_to(target) < 0.06, (rid, e.robots[rid].pose, target)
    assert "完成" in mode.status


def test_formation_shape_change_reassigns():
    e = SimEngine(SimConfig(seed=2, param_spread=0.05))
    for i, x in enumerate([0.5, 1.0, 1.5, 2.0]):
        e.add_robot(i + 1, Pose2D(x, 0.5, 0.0))
    mode = FormationMode(FormationMode.Params(shape="line", spacing=0.3))
    run_closed_loop(e, mode, 15.0)
    p = FormationMode.Params(shape="column", spacing=0.3)
    mode.update_params(p)
    run_closed_loop(e, mode, 20.0)
    ys = [e.robots[i].pose.y for i in range(1, 5)]
    assert max(ys) - min(ys) > 0.8  # 縦一列（heading 90°）になった


def test_follow_person_forms_a_queue():
    e = SimEngine(SimConfig(seed=3, param_spread=0.10))
    load_scenario(e, "scenarios/demo_follow.yaml")
    e.bodies = [b for b in e.bodies if b.kind == "person"]
    person = e.persons()[0]

    def walk(engine, t):   # 人が 0.15 m/s で右へ歩き、途中で上へ曲がる
        person.vx, person.vy = (0.15, 0.0) if t < 6 else (0.0, 0.1) if t < 10 else (0.0, 0.0)

    mode = FollowPersonMode()
    run_closed_loop(e, mode, 18.0, on_step=walk)
    assert mode.order == [1, 2, 3]
    prev = (person.x, person.y)
    for rid in mode.order:
        pos = e.robots[rid].pose
        d = math.hypot(pos.x - prev[0], pos.y - prev[1])
        assert 0.12 < d < 0.6, (rid, d)
        prev = (pos.x, pos.y)
