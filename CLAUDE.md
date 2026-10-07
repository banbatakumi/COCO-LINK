# CLAUDE.md — COCO-LINK 開発ガイド（Claude Code 向け）

大学「ロボット創造実験」の **複数台協調パフォーマンス＆物資運搬ロボット** システム。
避難所・介護施設で、複数の2輪ロボットが隊列・フォーメーションで動いて人を楽しませ、物資も運ぶ。

## リポジトリ構成（モノレポ）

| ディレクトリ | 中身 | 詳しいガイド |
|---|---|---|
| `pc/` | PC 側ソフト（Python 3.11+ / PySide6）: 操作GUI `coco-operator`・シミュレータ `coco-sim`・ビジョン `coco-vision` と共通ライブラリ | `pc/CLAUDE.md` |
| `firmware/` | ESP32 ファームウェア（PlatformIO / Arduino）。**未着手** | `firmware/CLAUDE.md` |
| `docs/` | 共通の設計文書。`protocol.md` と `firmware_spec.md` が PC ⇄ ファームの契約 | — |
| `.github/` | CI（`pc/` の lint + test を Ubuntu/macOS/Windows で） | — |
| `.claude/skills/` | スキル: `add-mode`（モード追加）, `change-protocol`（プロトコル変更） | — |

ROS は使わない。Mac と Windows の両方で動くこと。

## 最低限のコマンド

```bash
cd pc
pip install -e ".[dev]"     # 初回（venv 推奨）
ruff check . && pytest      # コミット前に必ず（CI と同じ）
coco-sim --scenario scenarios/demo_formation.yaml   # 別ターミナルで
coco-operator
```

PC 側のコマンドは **すべて `pc/` で実行する**（`config/`・`scenarios/` は `pc/` からの相対パス）。

## 設計文書（変更前に該当箇所を読むこと）

`docs/architecture.md`（全体） / `docs/protocol.md`（★契約） / `docs/firmware_spec.md`（ESP32） /
`docs/control.md`（制御則） / `docs/system_identification.md` / `docs/vision.md` / `docs/data_logging.md` /
`docs/adding_a_mode.md` / `docs/git_workflow.md`

## PC とファームにまたがるルール

1. **実機とシミュレータで同じインターフェース**。シミュレータは実機 ESP32 と同じ UDP プロトコルを話す。
   operator 側に「シミュレータなら〜」という分岐を書かない（シミュ専用情報は `sim_truth` メッセージのみ）。
2. **プロトコル変更は PC とファームを同時に**: `docs/protocol.md`・`pc/src/coco_link/protocol/messages.py`・
   `docs/firmware_spec.md`・`pc/tests/test_protocol.py` をセットで更新する（スキル `change-protocol`）。
3. **ESP32 の挙動の正は `docs/firmware_spec.md` と、その Python 参照実装 `pc/src/coco_link/robot/firmware_model.py`**。
   どちらかを変えたら、もう片方も合わせる。
4. 単位は SI。座標系は `docs/protocol.md §4`（フィールド: x右・y上、ロボット: x前・y左）。
5. コメント・docstring・文書は日本語、識別子は英語。

## Git

- ブランチ: `feature/*`, `fix/*`, `docs/*`, `chore/*`, `firmware/*`（`docs/git_workflow.md`）。機能ごとに切って `--no-ff` でマージ
- コミット: Conventional Commits（`feat(modes): 〜`, `fix(sim): 〜`, `feat(firmware): 〜`）。1コミット1目的、テストが通る状態で
- `logs/`・`output/`・`.venv/`・`.pio/` はコミットしない。`pc/config/robots/robot_XX.yaml`（同定結果）はコミットする
