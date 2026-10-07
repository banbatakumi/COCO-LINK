# CLAUDE.md — firmware/（ESP32 ファームウェア）

ロボット（ESP32, PlatformIO + Arduino）のファームウェア。**まだ未着手**。リポジトリ全体のルールは `../CLAUDE.md`。

## 作るときに守ること

1. **仕様書 `../docs/firmware_spec.md` が正**。状態機械・車輪速度制御・センサ処理・受入試験はそこに書いてある。
2. **迷ったら Python の参照実装 `../pc/src/coco_link/robot/firmware_model.py` と同じ挙動にする**。
   シミュレータの仮想ロボットはこれで動いているので、一致していれば操作GUI・システム同定がそのまま実機で動く。
3. 通信は `../docs/protocol.md` どおり（UDP + JSON, ArduinoJson）。メッセージ名・キー・ポート・パラメータ既定値を変えたくなったら、
   ファームだけで変えずにスキル `change-protocol` の手順で PC 側と同時に変える。
4. `telemetry` の `t_ms` は送信時刻ではなく「センサ値を取得した制御周期の時刻」（同定精度に効く, spec §7）。
5. 制御ループ（100 Hz）の中でブロッキング処理（`delay`, `pulseIn`, 大量の `Serial.print`）をしない。

## 確認方法

- ビルド: `pio run`（`firmware/` で）
- 実機確認: `../pc/` で `coco-operator` を起動 → ロボット一覧に出るか、手動操作で動くか、spec §10 のチェックリスト
