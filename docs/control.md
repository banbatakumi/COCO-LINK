# 制御則の解説

本書は `pc/src/coco_link/control/` と `pc/src/coco_link/modes/` の制御アルゴリズムを数式で説明する。
階層は次のとおり。

```
上位制御 (PC, 20 Hz)                     下位制御 (ESP32, 100 Hz)
┌───────────────────────────────┐        ┌──────────────────────────┐
│ モード: 隊形生成/割当, 軌跡追従 │ (v,ω) │ 逆運動学 → 車輪速度 PI     │ duty
│ 姿勢制御: 極座標FB / Pure Pursuit├──────▶│ + FF + 不感帯/電圧補償    ├──▶ モータ
│ 安全: 超音波減速, 衝突回避       │  UDP   │ (docs/firmware_spec §5)   │
└──────────────▲────────────────┘        └──────────▲───────────────┘
               │ 姿勢 (x,y,θ) 30 Hz                   │ エンコーダ
           ビジョン                                    
```

## 1. ロボットのモデル

差動2輪ロボット（ユニサイクルモデル）:

$$
\dot x = v\cos\theta,\quad \dot y = v\sin\theta,\quad \dot\theta = \omega
$$

車輪角速度との関係（$r$: 車輪半径, $b$: トレッド）:

$$
v = \frac{r(\omega_R+\omega_L)}{2},\quad \omega = \frac{r(\omega_R-\omega_L)}{b}
$$

非ホロノミック拘束（横に動けない）があるため、「目標点へ向かう」制御は単純な比例制御では済まない。
これが以下の制御則を使う理由である。

## 2. 姿勢制御（`control/pose_tracking.py`）

### 2.1 極座標フィードバック（`go_to_pose`）

ロボットから目標姿勢 $(x^*, y^*, \theta^*)$ への誤差を極座標で表す:

$$
\rho = \sqrt{\Delta x^2+\Delta y^2},\quad
\alpha = \operatorname{atan2}(\Delta y, \Delta x) - \theta,\quad
\beta = \theta^* - \theta - \alpha
$$

制御則:

$$
v = k_\rho \rho,\quad \omega = k_\alpha \alpha + k_\beta \beta
$$

$k_\rho>0,\ k_\beta<0,\ k_\alpha-k_\rho>0$ のとき目標姿勢は局所漸近安定（Siegwart et al., *Introduction to Autonomous Mobile Robots*, §3.6）。
実装上の工夫:
- $|\alpha|>\pi/2$ なら後退で近づく（無駄な U ターンをしない）
- $v$ に $\cos^2\alpha$ を掛け、目標方向を向くまで前進を抑える
- $(v,\omega)$ を同じ比率で飽和させ、曲率を保つ（`saturate`）
- 距離 3 cm 以内に入ったらその場旋回で向きだけ合わせる

### 2.2 Kanayama 軌道追従（`kanayama_tracking`）

参照軌道が速度 $(v_r,\omega_r)$ で動くとき、ロボット座標系での誤差 $(e_x,e_y,e_\theta)$ に対し

$$
v = v_r\cos e_\theta + k_x e_x,\quad
\omega = \omega_r + v_r(k_y e_y + k_\theta \sin e_\theta)
$$

Lyapunov 関数で大域的安定性が示される（Kanayama et al., 1990）。将来の「経路計画して走行」モードで使う。

## 3. フォーメーション（`modes/formation.py`, `control/formation.py`）

### 3.1 仮想構造法

隊形全体を 1 つの剛体（仮想構造）とみなし、その中心姿勢 $C=(x_c,y_c,\theta_c)$ と、
隊形座標系での各スロット位置 $o_i$ から目標姿勢を作る:

$$
p_i^* = \begin{bmatrix}x_c\\y_c\end{bmatrix} + R(\theta_c)\, o_i,\quad \theta_i^* = \theta_c
$$

$C$ を動かせば隊形ごと移動・回転する（「回転速度」パラメータで $\theta_c$ を時間変化させると隊形が回るパフォーマンスになる）。
形状は `line / column / circle / v / grid / heart`。ハートは曲線を道のりで等分してスロットを配置している。

### 3.2 スロット割り当て（ハンガリアン法）

$n$ 台のロボットを $n$ 個のスロットにどう割り当てるかは組合せ最適化問題:

$$
\min_{\sigma}\sum_i \lVert p_i - p^*_{\sigma(i)}\rVert^2
$$

ハンガリアン法（`scipy.optimize.linear_sum_assignment`）で $O(n^3)$ で厳密に解ける。
**二乗距離の和を最小化すると直線経路同士が交差しない** という性質があり（交差していれば入れ替えると総和が減る）、衝突が減る。
割り当てはロボット集合や隊形が変わったときだけ計算し直す（毎周期変えると振動するため）。

### 3.3 衝突回避

各ロボットは自分のスロットへ 2.1 の制御で向かいつつ、前方 ±60° 以内・距離 25 cm 以内の他ロボットに対し、
距離に比例して減速し、相手と反対側へ旋回する（`control/safety.py::avoid_robots`、簡易ポテンシャル法）。

## 4. 人追従・隊列（`modes/follow_person.py`, `control/path_follow.py`）

### 4.1 軌跡（breadcrumb）追従

「前を行くもの（人 or 前のロボット）が通った点列」を記録し、それをたどる。
人が障害物を避けて歩けば、ロボットも同じ道を通って避けられる（カルガモの親子・車列）。

- 先頭ロボット: 人の軌跡を、道のり `follow_distance` 後ろで追う
- $i$ 番目: $(i-1)$ 番目のロボットの軌跡を、道のり `spacing` 後ろで追う

### 4.2 速度: 道のり誤差の P 制御

$L$ = 自分から前のものまでの **軌跡に沿った** 道のり、$d$ = 目標間隔として

$$
v = \operatorname{clip}(k_v (L - d),\ 0,\ v_{max})
$$

直線距離ではなく道のりを使うので、カーブで内側を近道しない。

### 4.3 操舵: Pure Pursuit

軌跡上で道のり $L_d$（注視距離）先の点を目標に、その点を通る円弧の曲率で旋回する:

$$
\kappa = \frac{2\sin\alpha}{L_d},\quad \omega = v\,\kappa
$$

$L_d$ が小さいと追従は正確だが振動しやすく、大きいと滑らかだが内側を回る（実験で調整する価値のあるパラメータ）。

## 5. 安全機能（`control/safety.py`, `fleet/control_core.py`）

| 層 | 機能 | 実装 |
|---|---|---|
| ESP32 | ウォッチドッグ（300 ms 指令途絶で停止） | firmware_spec §4 |
| ESP32 | E-STOP（ラッチ） | firmware_spec §4 |
| PC | 超音波減速: 進行方向の距離 $d$ に対し $v \leftarrow v\cdot\operatorname{clip}\left(\frac{d-0.08}{0.30-0.08},0,1\right)$ | `ultrasonic_speed_limit` |
| PC | ロボット間回避 | `avoid_robots` |
| PC | 優先度調停: E-STOP > 手動 > 同定 > モード | `ControlCore.tick` |

## 6. 姿勢推定（`fleet/world_model.py`）

ビジョンは約 50–100 ms 遅れて届く。遅延 $\tau$ の間にロボットは $v\tau$ 進んでいるので、
エンコーダから求めた $(v,\omega)$ で観測時刻から現在まで円弧積分して補償する。
ビジョンが 0.5 s 以内途切れた場合も同じ方法で推測航法し、それ以上は「姿勢不明」として制御対象から外す。

発展課題: カルマンフィルタ（EKF）でビジョン・エンコーダ・ジャイロを融合すると、ノイズと遅延に一層強くなる。

## 7. 参考文献

1. R. Siegwart, I. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed., MIT Press, 2011.
2. Y. Kanayama et al., "A stable tracking control method for an autonomous mobile robot," ICRA, 1990.
3. R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," CMU-RI-TR-92-01, 1992.
4. H. W. Kuhn, "The Hungarian method for the assignment problem," 1955.
5. M. A. Lewis, K.-H. Tan, "High precision formation control of mobile robots using virtual structures," 1997.
