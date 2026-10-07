# データロギング（MCAP）

操作GUIのツールバー「● MCAP 記録」（または `coco-operator --record`）で、通信と制御の全データを
`pc/logs/coco_YYYYmmdd_HHMMSS.mcap` に記録する。実装: `pc/src/coco_link/datalog/mcap_logger.py`

## 記録されるトピック

| トピック | 内容 | 周期 |
|---|---|---|
| `/robot/<id>/telemetry` | テレメトリ（`robot_t_ms` = ロボット時計付き） | 50 Hz |
| `/robot/<id>/hello`, `params`, `pong`, `sim_truth` | ロボットからのその他メッセージ | 随時 |
| `/robot/<id>/cmd/<type>` | operator → ロボットの全指令（cmd_vel, set_led, ...） | 20 Hz |
| `/vision/world_state` | ビジョン | 30 Hz |
| `/core/state` | 制御周期ごとの推定姿勢・姿勢源・指令・目標・目標誤差・モード | 20 Hz |
| `/core/event` | モード開始/停止/パラメータ変更・E-STOP | 随時 |

エンコーディングは JSON（スキーマは JSON Schema。dataclass の型ヒントから自動生成）。

## 見る・解析する

- **Foxglove**（https://foxglove.dev, Mac/Windows/ブラウザ）で `.mcap` を開く
  - Plot パネル: `/robot/1/telemetry.batt_v` や `/core/state.robots[0].target_err` を時系列グラフに
  - Raw Messages パネル: メッセージの中身をそのまま見る
- **CSV に変換**して Excel / MATLAB / pandas で解析:
  ```bash
  cd pc
  python tools/mcap_to_csv.py logs/coco_20261007_120000.mcap
  ```
- **Python から直接**:
  ```python
  from coco_link.datalog import read_mcap
  for topic, t, data in read_mcap("logs/xxx.mcap", topics=["/core/state"]):
      ...
  ```

## 実験レポートでの使い方の例

- フォーメーションの収束時間・定常誤差: `/core/state` の `target_err` を全ロボットで集計
- 隊列追従の間隔: 隣り合うロボットの `x, y` から距離を計算
- 通信品質: `/robot/<id>/telemetry` の受信間隔のヒストグラム、`seq` の欠番からパケットロス率
- 電池の放電曲線: `batt_v` と走行時間
