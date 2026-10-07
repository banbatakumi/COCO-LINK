---
name: change-protocol
description: COCO-LINK の UDP 通信プロトコル（メッセージ種別・フィールド・ポート・パラメータキー）を追加・変更するときに使う手順。ロボット・シミュレータ・ビジョン・操作GUI間でやりとりする内容を変えるときは必ず読む。
---

# プロトコル変更の手順

プロトコルは実機ファームウェア（別言語・別担当）との契約なので、**片側だけの変更は禁止**。

1. `docs/protocol.md` を先に更新する（フィールド名・型・単位・送信周期）。
   - 後方互換を壊す変更（既存フィールドの意味変更・削除）は `PROTOCOL_VERSION` を上げる
   - 追加だけなら版は据え置き（受信側は未知キーを無視する仕様）
2. `src/coco_link/protocol/messages.py` の dataclass を更新し、`MESSAGE_TYPES` 登録を確認。
3. 送信側・受信側を更新:
   - ロボット側の挙動: `src/coco_link/robot/firmware_model.py`（ファームの参照実装）
   - 操作GUI側: `src/coco_link/fleet/`
   - ビジョン: `src/coco_link/vision/pipeline.py`（実ビジョン）と `src/coco_link/sim/vision_emulator.py`（仮想ビジョン）の **両方**
4. `docs/firmware_spec.md` の該当箇所（§4〜§8, §11）を更新。
5. `tests/test_protocol.py` に往復（encode → decode）テストを追加。
6. 1400 byte 制限を超えないか確認（`tests/test_protocol.py::test_size_limits`）。
7. `ruff check . && pytest`。
8. コミットは `feat(protocol): ...` または `fix(protocol): ...`。
