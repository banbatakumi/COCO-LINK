# COCO-LINK 通信プロトコル仕様 v1

本書は COCO-LINK の全プロセス（操作GUI・シミュレータ・ビジョン・ESP32 ファームウェア）が守る **唯一の通信契約** である。
実装は `pc/src/coco_link/protocol/messages.py`（Python）と `docs/firmware_spec.md`（ESP32）に反映されている。
**変更するときは本書・`messages.py`・`firmware_spec.md`・`pc/tests/test_protocol.py` を必ず同時に更新すること。**

---

## 1. 設計方針

| 項目 | 方針 | 理由 |
|---|---|---|
| トランスポート | UDP/IPv4 | 低遅延。制御指令は「最新値だけが意味を持つ」ので再送不要。ESP32 で軽量に実装可能 |
| エンコーディング | UTF-8 JSON、1データグラム = 1メッセージ | Wireshark / `nc -u` で人間が読める。ESP32 は ArduinoJson で扱える。`codec.py` を差し替えれば MsgPack にも移行可能 |
| 最大サイズ | 1400 byte 以下 | Ethernet/Wi-Fi の MTU 内に収め IP フラグメントを避ける |
| 信頼性 | 周期送信 + ウォッチドッグ | 指令は 10 Hz 以上で送り続ける。途絶したらロボットは自分で止まる（フェイルセーフ） |
| 同一性 | **実機とシミュレータは同じメッセージを話す** | 操作GUIは相手が実機か仮想ロボットか区別しない。これがシステム全体の要 |

## 2. ノードとポート

| ノード | 受信ポート | 送信先 |
|---|---|---|
| 操作GUI (operator) | **50000**: ロボット系 (hello / telemetry / params / pong / sim_truth)<br>**50001**: ビジョン系 (world_state) | ロボット: hello の送信元アドレス |
| ロボット (ESP32) | **50100** | operator:50000 |
| 仮想ロボット (simulator) | 127.0.0.1:**50100 + robot_id** | operator:50000 |
| ビジョン (vision / simulator のビジョンエミュレータ) | なし | operator:50001 |

ポート番号は `pc/config/network.yaml` で変更できる。

### 2.1 接続（ディスカバリ）手順

```
Robot                                     Operator
  |-- hello (1 Hz, 常時) ----------------->|  送信元 (IP, port) を記録し RobotProxy を生成
  |<------------------ cmd_vel / ping -----|  記録したアドレスへ送信
  |  (最後に受信した送信元を operator として latch)
  |-- telemetry (既定 50 Hz) ------------->|
```

1. ロボットは起動後、`hello` を **1 Hz で常に** 送信する。宛先は設定値 `operator_host`（既定: `255.255.255.255` ブロードキャスト。シミュレータは `127.0.0.1`）。
2. 操作GUIは `hello` の送信元 `(IP, port)` を、そのロボットへの送信先として記録する。
3. ロボットは、自分宛てに正しいメッセージを送ってきた最後の送信元を operator アドレスとして記憶し、以降 `telemetry` をそこへ unicast する。
4. operator 未確定の間、ロボットは `telemetry` を送らない（`hello` のみ）。
5. 操作GUIは 2 秒間何も受信しないロボットを「切断」として表示する。

## 3. 共通エンベロープ

すべてのメッセージは次のフィールドを持つ JSON オブジェクトである。

| キー | 型 | 説明 |
|---|---|---|
| `v` | int | プロトコルバージョン。本書は `1`。不一致のメッセージは破棄してよい |
| `type` | string | メッセージ種別（下表） |
| `src` | string | 送信者。`"op"`, `"robot:<id>"`, `"vision"`, `"sim"` |
| `seq` | uint32 | 送信者ごとの通し番号（欠落・順序逆転の検出用。2^32 で wrap） |
| `t_ms` | uint32 | 送信者のモノトニック時刻 [ms]（ESP32 は `millis()`）。`telemetry` ではセンサ値を取得した制御周期の時刻 |

例:
```json
{"v":1,"type":"cmd_vel","src":"op","seq":1024,"t_ms":53122,"vx":0.15,"wz":-0.3}
```

未知の `type`・未知のキーは **無視** する（前方互換）。

## 4. 座標系と単位

- 単位は SI（m, s, rad, m/s, rad/s, V）。duty は無次元 [-1, 1]。
- **フィールド座標系**: 原点はフィールド角（ArUco ID 40 の位置）。x 右向き、y 上向き（カメラから見て反時計回りが正）。θ は x 軸から反時計回り、範囲 (-π, π]。
- **ロボット座標系**: x 前方、y 左方、z 上方。`vx` は前進が正、`wz` は反時計回り（左旋回）が正。
- **車輪**: 左右とも「ロボットが前進する回転方向」を正とする（モータ・エンコーダの取り付け向きはファームのパラメータ `motor_dir_*` / `enc_dir_*` で吸収）。

## 5. メッセージ定義

### 5.1 operator → robot

| type | フィールド | 説明 |
|---|---|---|
| `cmd_vel` | `vx` [m/s], `wz` [rad/s] | 機体速度指令。ロボット内で車輪速度 PI 制御（閉ループ）。`max_v`, `max_w` でクランプ。状態を RUNNING へ |
| `cmd_wheel` | `left`, `right` [-1, 1] | 車輪 duty 直接指令（開ループ）。**システム同定用**。電池電圧補償なし |
| `stop` | – | 目標速度 0 にして IDLE へ（減速制限あり） |
| `estop` | – | 即座にモータ出力 0。`clear_estop` まで ESTOP 状態を保持（他の指令は無視） |
| `clear_estop` | – | ESTOP 解除 → IDLE |
| `set_led` | `r`,`g`,`b` [0–255], `pattern` (`"solid"`/`"blink"`/`"breath"`/`"rainbow"`), `period_ms` | フルカラーLED |
| `buzzer` | `freq_hz`, `dur_ms` **または** `melody` (`"startup"`/`"ok"`/`"error"`/`"chime"`) | `freq_hz=0` で停止 |
| `set_param` | `key` (string), `value` (number) | パラメータ書き込み（RAM と NVS）。応答として `params` を返す |
| `get_params` | – | 全パラメータ要求。応答は `params` |
| `ping` | `nonce` (uint32) | 応答は同じ `nonce` を持つ `pong`。RTT 計測用 |

### 5.2 robot → operator

| type | フィールド | 説明 |
|---|---|---|
| `hello` | `robot_id`, `fw` (string), `mac` (string), `caps` (string 配列) | 1 Hz 常時。`caps` 例: `["led_rgb","buzzer","us_front","us_rear","imu","enc"]` |
| `telemetry` | 下表 | 既定 50 Hz（`telemetry_hz`） |
| `params` | `values` (object: key → number) | `get_params` / `set_param` への応答 |
| `pong` | `nonce` | `ping` への応答 |
| `log` | `level` (`"info"`/`"warn"`/`"error"`), `msg` | 任意。デバッグ用 |
| `sim_truth` | `values` (object) | **シミュレータ専用拡張**。仮想ロボットの真の物理パラメータ（同定精度評価用）。実機は送らない |

`telemetry` のフィールド:

| キー | 型/単位 | 説明 |
|---|---|---|
| `robot_id` | int | |
| `state` | string | `BOOT` / `IDLE` / `RUNNING` / `ESTOP` / `LOW_BATT` / `FAULT` |
| `faults` | string 配列 | 例 `["imu","enc_l"]`。異常がなければ空 |
| `batt_v` | float [V] | 分圧抵抗から換算した電池電圧 |
| `us_front_m`, `us_rear_m` | float [m] または `null` | 超音波距離。エコーなし（範囲外）は `null` |
| `imu` | `{ax,ay,az}` [m/s²], `{gx,gy,gz}` [rad/s] | ロボット座標系。ジャイロバイアス未補正の生値 |
| `enc` | `{left_rad, right_rad, wl, wr}` | 車輪の累積回転角 [rad]（AS5600 の 12bit 角を unwrap）と推定角速度 [rad/s] |
| `applied` | `{left, right}` | 実際にモータへ出している duty [-1, 1] |
| `rssi` | int [dBm] | Wi-Fi 受信強度 |
| `loop_dt_us` | int [µs] | 制御ループ周期の実測値（リアルタイム性の監視） |
| `rx_count` | uint32 | 受信した有効メッセージ数（パケットロス評価） |

### 5.3 vision → operator

| type | フィールド | 説明 |
|---|---|---|
| `world_state` | 下表 | 既定 30 Hz |

| キー | 型 | 説明 |
|---|---|---|
| `frame` | uint32 | フレーム番号 |
| `stamp_ms` | uint32 | 撮像時刻（送信者のモノトニック時刻）|
| `latency_ms` | float | 撮像から送信までの推定遅延 |
| `field` | `{w, h}` [m] | フィールド寸法 |
| `robots` | `[{id, x, y, th, conf}]` | ArUco ID = ロボットID。`conf` は 0–1 |
| `persons` | `[{x, y, r}]` | 緑色領域（人）。`r` は等価半径 [m] |
| `obstacles` | `[{x, y, r}]` | 赤色領域（障害物） |
| `cargo` | `[{x, y, r}]` | 青色領域（物資） |

1400 byte を超えそうな場合、`conf` の低い順・面積の小さい順に要素を削る。

## 6. タイミングとフェイルセーフ

| 項目 | 値 | 備考 |
|---|---|---|
| 指令送信周期 (operator) | 20 Hz 以上推奨（最低 10 Hz） | 制御ループ周期で送る |
| ウォッチドッグ (robot) | 300 ms（`watchdog_ms`） | `cmd_vel`/`cmd_wheel` が途絶したらモータ停止し IDLE |
| telemetry 周期 | 50 Hz（`telemetry_hz`） | |
| hello 周期 | 1 Hz | |
| 切断判定 (operator) | 2 s 無受信 | |
| ESTOP | 全指令に優先 | `clear_estop` のみ受理 |

## 7. パラメータキー一覧（`set_param` / `params`）

| key | 既定値 | 単位 | 説明 |
|---|---|---|---|
| `robot_id` | 1 | – | 変更後は再起動で反映 |
| `wheel_radius` | 0.021 | m | 同定で更新 |
| `tread` | 0.090 | m | 左右車輪間距離（実効値）。同定で更新 |
| `max_v` | 0.40 | m/s | |
| `max_w` | 4.0 | rad/s | |
| `accel_limit` | 1.0 | m/s² | 車輪目標速度の変化率制限 |
| `kp` | 0.02 | duty/(rad/s) | 車輪速度 PI |
| `ki` | 0.4 | duty/rad | |
| `kff` | 0.04 | duty/(rad/s) | フィードフォワード（≒ 1/K） |
| `tau_ff` | 0.06 | s | 加速度フィードフォワードの時定数（≒ モータ時定数 τ） |
| `u_deadzone` | 0.10 | duty | 不感帯補償量 |
| `watchdog_ms` | 300 | ms | |
| `telemetry_hz` | 50 | Hz | |
| `batt_nominal_v` | 4.8 | V | 電池電圧補償の基準 |
| `batt_low_v` | 4.2 | V | これを 2 s 下回ると LOW_BATT |
| `batt_div_ratio` | 2.0 | – | (R1+R2)/R2 |
| `motor_dir_l`, `motor_dir_r` | 1 | ±1 | |
| `enc_dir_l`, `enc_dir_r` | 1 | ±1 | |

## 8. 動作確認方法

```bash
# ロボットからの受信を覗く（macOS/Linux）
nc -ul 50000
# 手でロボットに指令を送る
echo '{"v":1,"type":"buzzer","src":"op","seq":0,"t_ms":0,"melody":"ok"}' | nc -u -w0 192.168.0.101 50100
```

Windows では `pc/tools/` の Python スクリプトや Wireshark（フィルタ `udp.port==50000`）を使う。
