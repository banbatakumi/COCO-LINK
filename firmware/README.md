# firmware/ — ESP32 ファームウェア（未着手）

ロボット（ESP32）のファームウェアをここに PlatformIO プロジェクトとして作る予定。

- 仕様: [../docs/firmware_spec.md](../docs/firmware_spec.md)
- 通信: [../docs/protocol.md](../docs/protocol.md)
- 参照実装（Python, 状態機械・制御則の正解）: [../pc/src/coco_link/robot/firmware_model.py](../pc/src/coco_link/robot/firmware_model.py)
- 開発ガイド（Claude Code 向け）: [CLAUDE.md](CLAUDE.md)

## 予定している構成

```
firmware/
├── platformio.ini       ボード設定（ESP32 の型番が決まったら）, lib_deps = ArduinoJson
├── include/config.h     ピン定義・定数（firmware_spec §1.3）
└── src/                 main.cpp と各モジュール（firmware_spec §2.2）
```

## 作り始めるとき

```bash
cd firmware
pio project init --board esp32dev --project-option "framework=arduino"
pio pkg install --library "bblanchon/ArduinoJson@^7"
```

ブランチは `firmware/<内容>`、コミットは `feat(firmware): ...`（docs/git_workflow.md）。
作ったら `.github/workflows/ci.yml` にビルドジョブを追加する。
完成したら firmware_spec.md §10 の受入試験チェックリストをすべて確認し、操作GUIのシステム同定が完走することを確かめる。
