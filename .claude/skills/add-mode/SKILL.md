---
name: add-mode
description: COCO-LINK の操作GUIに新しい上位制御モード（例: 協調物資運搬、経路計画走行、ダンス）を追加するときに使う手順。「モードを追加」「新しい動作を作りたい」と言われたら読む。
---

# 新しいモードを追加する手順

詳細な設計指針は `docs/adding_a_mode.md` にある。必ず先に読むこと。

1. `src/coco_link/modes/<mode_name>.py` を作成する。既存の `formation.py` / `follow_person.py` を雛形にする。
   - `Mode` を継承し、クラスに `@register_mode` を付ける
   - `name`（GUI 表示名, 日本語可）と `Params`（`@dataclass`、各フィールドに既定値。`field(metadata={"label":..., "min":..., "max":..., "step":...})` で GUI 表示を制御）を定義
   - `on_start(world)`, `step(world, dt) -> dict[int, Command]`, `on_stop()` を実装
   - `step()` は **純粋に近い関数** にする（Qt・ソケット・time.sleep を使わない）。時刻は引数の `dt` と `world.t` を使う
2. 必要な制御則は `src/coco_link/control/` に関数として切り出す（モードは組み合わせるだけ）。
3. `tests/test_modes_<mode_name>.py` を作成し、`tests/helpers.py` のヘッドレスシミュレーション (`run_closed_loop`) で目標状態に収束することを確認する。
4. `docs/control.md` にアルゴリズム（数式）を追記する。
5. `ruff check . && pytest` が通ることを確認。
6. シミュレータ (`coco-sim`) + 操作GUI (`coco-operator`) で動作を目視確認。
7. `feature/mode-<name>` ブランチで `feat(modes): ...` としてコミット。
