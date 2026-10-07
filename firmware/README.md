# firmware/ — ESP32 ファームウェア

ここに PlatformIO プロジェクト（Arduino-ESP32）を置く予定。

- 仕様: [../docs/firmware_spec.md](../docs/firmware_spec.md)
- 通信: [../docs/protocol.md](../docs/protocol.md)
- 参照実装（Python, 状態機械・制御則の正解）: [../src/coco_link/robot/firmware_model.py](../src/coco_link/robot/firmware_model.py)

## 作り始めるとき

```bash
pio project init --board esp32dev --project-option "framework=arduino"
pio pkg install --library "bblanchon/ArduinoJson@^7"
```

ブランチは `firmware/<内容>`、コミットは `feat(firmware): ...` とする（docs/git_workflow.md）。
完成したら firmware_spec.md §10 の受入試験チェックリストをすべて確認し、操作GUIのシステム同定が完走することを確かめる。
