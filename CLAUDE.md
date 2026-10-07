# CLAUDE.md — COCO-LINK 開発ガイド（Claude Code 向け）

大学「ロボット創造実験」の **複数台協調パフォーマンス＆物資運搬ロボット** システム。
避難所・介護施設で、複数の2輪ロボットが隊列・フォーメーションで動いて人を楽しませ、物資も運ぶ。

## 構成（3プロセス + N台のESP32ロボット）

| プロセス | コマンド | 役割 |
|---|---|---|
| 操作GUI（上位制御） | `coco-operator` | モード実行・手動操作・接続一覧・システム同定・MCAPログ |
| シミュレータ | `coco-sim` | 仮想ESP32×N と仮想ビジョンを演じる（**実機と同じUDPプロトコル**） |
| ビジョン | `coco-vision` | 天井カメラで ArUco(ロボット) と色(緑=人/赤=障害物/青=物資) を認識 |

ROS は使わない。PC側は Python 3.11+ / PySide6、Mac と Windows 両対応。
ESP32 は `docs/firmware_spec.md` に仕様のみ（実装は `firmware/` に今後追加）。

## コマンド

```bash
# セットアップ（初回）
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[dev]"

# 起動（それぞれ別ターミナル）
coco-sim --scenario scenarios/demo_formation.yaml   # demo_follow.yaml / sysid.yaml もある
coco-operator                                        # --mode フォーメーション --record も可
coco-vision --synthetic                              # --camera 0 / --video file.mp4

# 品質チェック（コミット前に必ず。CI と同じ）
ruff check .            # --fix で自動修正
pytest                  # 全テスト約20秒。UDP結合テストは実時間で数秒かかる
pytest tests/test_modes.py -k formation

# GUI の変更を目で確認する（ヘッドレス環境でも可）
QT_QPA_PLATFORM=offscreen coco-sim --scenario scenarios/demo_formation.yaml --quit-after 15 --screenshot /tmp/sim.png &
QT_QPA_PLATFORM=offscreen coco-operator --mode フォーメーション --quit-after 12 --screenshot /tmp/op.png
```

Linux のヘッドレス環境で Qt が `libEGL.so.1` 等を要求したら `apt-get install -y libegl1 libgl1 libxkbcommon0 libfontconfig1 libdbus-1-3`。

## ディレクトリマップ

```
src/coco_link/
  protocol/   messages.py（★通信契約。docs/protocol.md と 1:1）, codec.py（JSON）
  net/        udp.py: UdpEndpoint（受信スレッド＋コールバック）
  common/     geometry.py（Pose2D, wrap_angle）, config.py（YAML→dataclass）, clock.py
  robot/      params.py（物理パラメータ）, kinematics.py, plant.py（真の物理）, firmware_model.py（★ESP32参照実装）
  sim/        engine.py（Qt非依存, 200Hz）, sensors.py, entities.py, virtual_robot_node.py, vision_emulator.py, runner.py
  fleet/      fleet_manager.py（発見・送信）, world_model.py（ビジョン+テレメトリ統合）, control_core.py（制御スレッド・調停）
  control/    pose_tracking.py, path_follow.py, formation.py, safety.py（純粋関数）
  modes/      base.py（Mode, RobotCommand, param）, registry.py（@register_mode, discover）, idle/formation/follow_person
  sysid/      experiments.py（試験定義）, estimators.py（推定）, runner.py（ControlCore のオーバーライドとして実行）, recording.py
  datalog/    mcap_logger.py（標準 logging と区別するため datalog という名前）
  vision/     calibration.py, aruco_tracker.py, color_detector.py, pipeline.py, synthetic.py（合成画像）, sources.py, worker.py
  apps/       GUI（operator / simulator / vision / common）。ロジックは置かない
config/       network.yaml, field.yaml, robots/default.yaml, robots/robot_XX.yaml（同定結果, コミットする）
scenarios/    シミュレータ初期配置
tools/        generate_aruco_markers.py（実寸PDF）, mcap_to_csv.py
tests/        helpers.py（run_closed_loop: 通信抜きの高速閉ループ）, test_*.py
```

## 設計文書（変更前に該当箇所を読むこと）

`docs/architecture.md`（全体） / `docs/protocol.md`（★契約） / `docs/firmware_spec.md`（ESP32） /
`docs/control.md`（制御則） / `docs/system_identification.md` / `docs/vision.md` / `docs/data_logging.md` /
`docs/adding_a_mode.md` / `docs/git_workflow.md`

## 守るべきルール

1. **実機とシミュレータで同じインターフェース**。operator 側に「シミュレータなら〜」という分岐を書かない。
   シミュ専用情報は `sim_truth` メッセージのみ（無くても動くこと。`RobotProxy.is_simulated` は表示専用）。
2. **プロトコル変更は 4 点セット**: `docs/protocol.md`・`protocol/messages.py`・`docs/firmware_spec.md`・`tests/test_protocol.py`
   （スキル `change-protocol`）。ファームのパラメータキーを足すときは `firmware_model.py` の `DEFAULT_PARAMS`/`PARAM_RANGES` も。
3. **`apps/` 以外で Qt を import しない**。ロジックは Qt 非依存にして pytest でテストする。
4. **ESP32 の挙動を変えたら `robot/firmware_model.py` も直す**（参照実装を仕様と一致させる）。
5. 新しいモードは `modes/` に追加するだけで GUI に出る設計を守る（スキル `add-mode`）。GUI 側にモード固有コードを書かない。
6. 単位は SI。角度は `wrap_angle()` で (-π, π]。座標系は `docs/protocol.md §4`（フィールド: x右・y上、ロボット: x前・y左）。
7. 新機能にはテストを付ける。制御系は `tests/helpers.py::run_closed_loop` で収束を、通信は UDP 結合テスト（`tests/test_fleet.py` の `system` fixture）で確認。
8. Windows 対応: パスは `pathlib`、ファイルは `encoding="utf-8"` 明示、UDP の WSAECONNRESET は `net/udp.py` で対処済み。
9. コメント・docstring・文書は日本語、識別子は英語。

## ハマりどころ（過去に踏んだもの）

- dataclass に `field` という名前のフィールドを作ると `dataclasses.field` を隠す → `from dataclasses import field as dc_field`（`protocol/messages.py`）
- 乱数で `set` を反復すると、ハッシュランダム化でプロセスごとに順序が変わり再現しない → タプル/リストを使う
- テレメトリの `t_ms` は「センサ取得時刻（ロボット時計）」。同定の微分・積分はこれを使う（受信時刻は Wi-Fi で揺らぐ）
- 同定中はテレメトリを 100 Hz（制御周期と同じ）にする。50 Hz だと τ を約 2 割過小評価する（docs/system_identification.md §5）
- OpenCV は `opencv-python-headless` を使う（通常版は Qt を同梱し PySide6 と衝突する）
- UDP ポートはテストでは 0（自動割当）を使い、固定ポート 50000 系と衝突させない

## Git

- ブランチ: `feature/*`, `fix/*`, `docs/*`, `chore/*`, `firmware/*`（`docs/git_workflow.md`）。機能ごとに切って `--no-ff` でマージ
- コミット: Conventional Commits（`feat(modes): 〜`, `fix(sim): 〜`）。1コミット1目的、`ruff check . && pytest` が通る状態で
- `logs/`・`output/`・`.venv/` はコミットしない。`config/robots/robot_XX.yaml`（同定結果）はコミットする
