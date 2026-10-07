# モードの追加ガイド

COCO-LINK の上位制御は **モード（プラグイン）** で拡張する。`pc/src/coco_link/modes/` に 1 ファイル追加するだけで、
操作GUIのモード一覧・パラメータ入力フォームに自動で現れる。GUI のコードを触る必要はない。

## 1. 最小の例

```python
# pc/src/coco_link/modes/spin.py
from dataclasses import dataclass

from .base import Mode, RobotCommand, param
from .registry import register_mode


@register_mode
class SpinMode(Mode):
    name = "その場回転"                       # GUI 表示名
    description = "全ロボットがその場で回転する"

    @dataclass
    class Params:                             # GUI のフォームはここから自動生成
        speed: float = param(1.0, "角速度 [rad/s]", min=-3.0, max=3.0, step=0.1)

    def step(self, world, dt):
        self.status = f"{len(world.usable_robots())} 台回転中"   # GUI に表示される
        return {rid: RobotCommand(vx=0.0, wz=self.params.speed) for rid in world.usable_robots()}
```

## 2. 使える情報（`World`）

| 属性 | 内容 |
|---|---|
| `world.t` | 時刻 [s] |
| `world.field_w`, `field_h` | フィールド寸法 [m] |
| `world.robots[id]` | `RobotState`: `pose` (Pose2D, 不明なら None), `vx`, `wz`, `telemetry`, `params`(同定済み物理パラメータ), `us_front`, `us_rear`, `state` |
| `world.usable_robots()` | 接続中・姿勢既知・走行可能なロボットだけ |
| `world.persons` / `obstacles` / `cargo` | ビジョンが見つけた人(緑)・障害物(赤)・物資(青) の `x, y, r` |

## 3. 出せる指令（`RobotCommand`）

| フィールド | 内容 |
|---|---|
| `vx`, `wz` | 機体速度 [m/s], [rad/s]（ロボット内で車輪 PI 制御される） |
| `wheel=(l, r)` | 開ループ duty（特殊用途） |
| `led=(r, g, b, pattern)` | LED。変化したときだけ送信される |
| `buzzer="ok"` or `440.0` | メロディ名 or 周波数（その周期に一度鳴らす） |
| `safety=False` | 超音波による自動減速を無効化（物資を押すときなど） |

返さなかったロボットには何も送られない → 300 ms 後にウォッチドッグで止まる。

## 4. フック

| メソッド | 呼ばれるとき |
|---|---|
| `on_start(world)` | 開始時 |
| `step(world, dt)` | 制御周期（既定 20 Hz）ごと |
| `on_stop(world)` | 停止時（既定: 全台停止） |
| `on_field_click(x, y)` | フィールドビューをクリックしたとき |
| `update_params(params)` | 実行中にパラメータが変更されたとき |

`self.viz`（`Visualization`）に目標姿勢・経路・点を入れるとフィールドビューに描画される。

## 5. 作法

- `step()` の中で `time.sleep`、ソケット、Qt を使わない。時間は `dt` と `world.t` で扱う。
- 制御則は `control/` に関数として切り出し、モードは「組み合わせ」に徹する（テストしやすい）。
- テストは `pc/tests/helpers.py` の `run_closed_loop(engine, mode, seconds)` で、通信抜きのシミュレーション閉ループを高速に回して収束を確認する（`pc/tests/test_modes.py` 参照）。

## 6. 今後追加したいモードの設計指針

### 協調物資運搬（Cooperative Transport）
- ビジョンの `cargo`（青）を対象に、2–3 台が物資を囲む「把持隊形」をとる（物資中心の仮想構造 + `formation.assign_slots`）
- 物資を目標地点へ押す: 隊形中心を物資位置、向きを目標方向にし、隊形全体を `kanayama_tracking` で目標へ動かす
- 押している間は `safety=False`（前方超音波が物資を検知して止まってしまうため）
- 評価: 物資の到達誤差・所要時間・隊形誤差（MCAP ログから算出）

### 経路計画走行（Path Planning）
- `world.obstacles` を占有格子地図にし、A* または RRT* で経路を計画（`control/planning.py` を新設）
- 経路を時間パラメータ付き軌道にして `kanayama_tracking` で追従
- 複数台の場合は優先度付き計画（先に計画したロボットの軌道を時空間障害物として扱う）

### パフォーマンス（ダンス）
- 時刻 → 隊形中心・形状・LED 色のタイムラインを YAML で定義し、音楽に合わせて再生する
- `buzzer` でメロディ、`led` で全台同期の色変化
