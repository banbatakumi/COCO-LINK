# CLAUDE.md — COCO-LINK 開発ガイド（Claude Code 向け）

大学「ロボット創造実験」の **複数台協調パフォーマンス＆物資運搬ロボット** システム。
避難所・介護施設で、複数の2輪ロボットが隊列・フォーメーションで動いて人を楽しませ、物資も運ぶ。

## 構成（3プロセス + N台のESP32ロボット）

| プロセス | コマンド | 役割 |
|---|---|---|
| 操作GUI（上位制御） | `coco-operator` | モード実行・テレオペ・接続一覧・システム同定・MCAPログ |
| シミュレータ | `coco-sim` | 仮想ESP32×N と仮想ビジョンを演じる（**実機と同じUDPプロトコル**） |
| ビジョン | `coco-vision` | 天井カメラで ArUco(ロボット) と色(緑=人/赤=障害物/青=物資) を認識 |

ROS は使わない。PC側は Python 3.11+ / PySide6、Mac と Windows 両対応。ESP32 は `docs/firmware_spec.md` に仕様のみ（実装は `firmware/` に今後追加）。

## コマンド

```bash
# セットアップ（初回）
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[dev]"

# 起動（それぞれ別ターミナル）
coco-sim --scenario scenarios/demo_formation.yaml
coco-operator
coco-vision --video path/to/video.mp4   # カメラなら --camera 0

# 品質チェック（コミット前に必ず）
ruff check .            # lint（--fix で自動修正）
pytest                  # 全テスト（GUI テストは QT_QPA_PLATFORM=offscreen で自動的にヘッドレス）
pytest tests/test_formation.py -k circle   # 一部だけ
```

## ディレクトリマップ

```
src/coco_link/
  protocol/   メッセージ定義（★通信契約。docs/protocol.md と一致させる）
  net/        UDP エンドポイント（受信スレッド）
  common/     Pose2D・角度正規化・設定読込
  robot/      物理パラメータ・差動2輪運動学・firmware_model（ESP32参照実装）
  sim/        物理エンジン・センサ・仮想ロボットノード・ビジョンエミュレータ
  fleet/      FleetManager（発見・送信）・WorldModel・ControlCore（制御スレッド）
  control/    姿勢追従・経路追従・隊形生成/割当・安全
  modes/      上位制御モード（プラグイン, @register_mode）
  sysid/      システム同定（試験・推定・実行）
  logging/    MCAP ロガー
  vision/     ArUco・色検出・キャリブレーション・送信
  apps/       GUI（operator / simulator / vision）。ロジックは置かない
config/       network.yaml, field.yaml, robots/*.yaml（同定結果）
scenarios/    シミュレータ初期配置
docs/         設計文書（下記）
tools/        マーカー生成・ログ変換
tests/        pytest
```

## 設計文書（変更前に該当箇所を読むこと）

- `docs/architecture.md` — 全体構成・スレッド・データフロー
- `docs/protocol.md` — UDP/JSON プロトコル（★契約）
- `docs/firmware_spec.md` — ESP32 ファーム仕様
- `docs/control.md` — フォーメーション・追従の制御則
- `docs/system_identification.md` — 同定の理論と手順
- `docs/vision.md` — マーカー・キャリブレーション
- `docs/adding_a_mode.md` — モード追加ガイド
- `docs/git_workflow.md` — ブランチ・コミット規約

## 守るべきルール

1. **実機とシミュレータで同じインターフェース**。operator 側に「シミュレータなら〜」という分岐を書かない。シミュ専用情報は `sim_truth` メッセージのみ（無くても動くこと）。
2. **プロトコル変更時は 4点セットで更新**: `docs/protocol.md`・`protocol/messages.py`・`docs/firmware_spec.md`・`tests/test_protocol.py`（スキル `change-protocol` 参照）。
3. **`apps/` 以外で Qt を import しない**。ロジックは Qt 非依存にして pytest でテストする。
4. **ESP32 の挙動を変えたら `robot/firmware_model.py` も直す**（参照実装を常に仕様と一致させる）。
5. 単位は SI（m, rad, s）。角度は `common/geometry.wrap_angle()` で (-π, π] に正規化。座標系は `docs/protocol.md §4`。
6. 新しいモードは `modes/` に追加するだけで GUI に出る設計を守る（スキル `add-mode` 参照）。GUI 側にモード固有のコードを書かない。
7. 新機能にはテストを付ける。制御系はヘッドレスシミュ (`tests/helpers.py`) で収束を確認する。
8. パスは `pathlib`、改行・文字コード（UTF-8 明示）に注意し Windows でも動くようにする。
9. コメント・docstring・文書は日本語、識別子は英語。

## Git

- ブランチ: `feature/*`, `fix/*`, `docs/*`, `chore/*`, `firmware/*`（`docs/git_workflow.md`）
- コミット: Conventional Commits（`feat(modes): 〜`, `fix(sim): 〜`）。1コミット1目的、テストが通る状態で
- `logs/` と `.venv/` はコミットしない。`config/robots/robot_XX.yaml`（同定結果）はコミットする
