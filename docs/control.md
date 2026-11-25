# 制御則の解説

本書は `pc/src/coco_link/control/` と `pc/src/coco_link/modes/` の制御アルゴリズムを数式で説明する。
実装と対応させたフィードバック制御式・ブロック線図を 1 か所にまとめた
[フィードバック制御則とブロック線図](feedback_control.md) も参照。
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

Lyapunov 関数で大域的安定性が示される（Kanayama et al., 1990）。協調物資運搬の押し出し（§7）で使う。

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
経路計画ナビゲーション（§8）は、同じ考え方を人・障害物・物資も含む円に広げた
`control/safety.py::avoid_circles` を、再計画の合間の反応的な回避に使う。

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

## 6. 包囲（`modes/encircle.py`, `control/encircle.py`）

クリックした人（緑）・物資（青）・障害物（赤）を、全ロボットで円形に取り囲む。

### 6.1 対象の選択と追従

クリック位置 $c$ に対し、候補物体 $i$（中心 $p_i$、半径 $r_i$）の表面までの距離

$$
d_i = \max\left(0,\ \lVert c-p_i\rVert-r_i\right)
$$

が最小で、$d_i \le d_{tol}$（クリック判定）のものを対象にする。中心距離ではなく表面距離で判定するので、
大きい障害物の端をクリックしても選べる。対象の種類（人・物資・障害物）と直前位置を覚えておき、
「対象を追従」が有効なら、同じ種類のうち直前位置に最も近いものをゲート距離内で追い続ける。

### 6.2 リングとスロット

対象中心 $p_t$、半径 $r_t$、機体半径 $r_b$、余白 $m$、台数 $n$ として、リング半径を

$$
R = \max\left(R_{min},\ r_t+r_b+m,\ \frac{n(2r_b+g)}{2\pi}\right)
$$

とする。第 3 項は台数が多いときにリング上で機体同士が重ならないための条件（$g$ は最小隙間）。
スロット $i$ は位相 $\phi_0$ を用いて

$$
p_i^* = p_t + R\begin{bmatrix}\cos\phi_i\\ \sin\phi_i\end{bmatrix},\quad
\phi_i = \phi_0 + \frac{2\pi i}{n},\quad
\theta_i^* = \phi_i+\pi
$$

と配置する（$\theta_i^*$ は中心を向く向き）。割り当てはフォーメーションと同じハンガリアン法（§3.2）で行い、
ロボット集合が変わったときだけ計算し直す。追従中は対象の移動に合わせてリング全体が動く。

### 6.3 走行制御

各ロボットは §2.1 の `go_to_pose` で自分のスロットへ向かい、§3.3 の `avoid_robots` で
近接する他ロボットを避ける。到着判定は位置 3 cm・向き 8°（`GoToPoseGains`）。

## 7. 協調物資運搬（`modes/cooperative_transport.py`, `control/transport.py`）

物資（青）を 1〜3 台で囲むように押し、目標地点まで運ぶ。状態機械は
`approach`（把持隊形へ移動）→ `push`（押し出し）→ `done`（完了）である。

### 7.1 把持隊形

物資中心を原点、目標方向を $+x$ とする物資座標系で、物資の後方 $\pi$ を中心に半角 $\sigma$ の円弧へ $n$ 台を等間隔に置く:

$$
\delta_i = -\sigma + \frac{2\sigma}{n-1}i \quad (i=0,\ldots,n-1),\qquad
o_i = R\begin{bmatrix}-\cos\delta_i\\-\sin\delta_i\end{bmatrix}
$$

$$
R = \max\left(r_{cargo}+r_{body}+m,\ \frac{2r_{body}+g}{2\sin(\sigma/(n-1))}\right)
$$

第 2 項は、隣り合うスロットの弦長が機体直径以上になる条件である。$m$ は物資との余白、$g$ は機体間の隙間。
目標姿勢は §3.1 と同じ仮想構造法で、物資姿勢 $C=(x_c,y_c,\theta_c)$ から

$$
p_i^* = \begin{bmatrix}x_c\\y_c\end{bmatrix}+R(\theta_c)\,o_i,\qquad \theta_i^*=\theta_c
$$

とする。スロット割り当ては §3.2 のハンガリアン法。搬送チームは物資に近い順に最大 `max_robots` 台を選ぶ。

### 7.2 押し出し目標（仮想物資）

実測の物資位置 $p_c$ から目標 $p_g$ への単位ベクトル $\hat u$ と距離 $d$ に対し、隊形中心を

$$
C = p_c + \hat u \min(\ell, d),\qquad \theta_c=\operatorname{atan2}(p_g-p_c)
$$

とする。$\ell$ は押し込み量（`lead`）。隊形中心が物資より $\ell$ だけ先行するため、ロボットは物資に少し食い込み、押す力が生まれる。$d \le$ `goal_tol` になったら完了とする。

### 7.3 追従と安全

- 把持隊形へは §2.1 の極座標フィードバックで移動する。
- 押し出し中は §2.2 の Kanayama 軌道追従で、各スロットを速度 $v_r=$ `push_speed` の参照軌道として追う。
- 押している間は `safety=False`。物資が前方超音波に映り、§5 の減速で止まってしまうため。
- 物資を見失ったら 0.5 s は最後の位置で継続し、それ以上で停止する。
- 評価は物資中心と目標地点の距離（到達誤差）で行う。

## 8. 全モード共通の経路計画（`control/navigation.py`, `control/planning.py`）

フォーメーション・人追従・包囲・協調物資運搬など、ロボットが移動するすべてのモードは
`Mode.run()` を通り、共通の経路計画ナビゲーションで障害物（赤）・人（緑）・物資（青）・
他のロボットを避ける。§2.1 の姿勢制御は「目標まで直線で見える」前提なので、間に障害物があると引っかかる。
モードは `Mode.navigation_targets()` でロボットごとの目標点を返し、ナビゲーションが
占有格子地図と A* で大域的な経路を作り、Pure Pursuit で追従する。

### 8.1 占有格子地図

フィールドを解像度 $\Delta = 5$ cm の格子に分け、ロボットの中心を置けるセルを「空き」とする。
物体 $i$（中心 $p_i$、半径 $r_i$）のまわり

$$
\lVert p - p_i \rVert < r_i + r_{body} + m
$$

のセルを占有にする（$r_{body}$: 機体半径、$m$: 安全余白、既定 8 cm）。
壁からも $r_{body}+m$ 以内のセルを占有にする。人・物資も同じ円で扱い、
他のロボットは現在位置に加えて、速度 $(v,\omega)$ から $\tau$ 秒先の位置も占有にする
（一緒に動いているロボットとの衝突を避けるため）。

### 8.2 A*

8 近傍の A*。移動コストは直進 1、斜め $\sqrt{2}$。ヒューリスティックは octile 距離

$$
h = (|dx|+|dy|) + (\sqrt{2}-2)\min(|dx|,|dy|)
$$

斜め移動は、両側の直交セルが空きのときだけ許可する（障害物の角をすり抜けない）。
計算量は $O(N\log N)$（$N$: 格子数）。始点・終点が占有セルなら、最も近い空きセルに置き換える。

### 8.3 平滑化と追従

A* の点列は格子に沿ってカクカクしているので、見通しのよい最も遠い点まで直線で結び直す
（greedy shortcut）。線分が占有セルを通らないかは半セル間隔でサンプリングして判定する。
追従は §4.3 の Pure Pursuit を使い、モードが `navigation_speeds()` で返した巡航速度
（省略時は既定 0.20 m/s）で走る。モードの瞬間的な速度指令は使わない。姿勢制御が特異姿勢に
近いときは指令速度がほぼ 0 になることがあるが、その値で経路を追うと極端に低速になって
目標へ着かないため、巡航速度を独立に与える。

人やロボットは動くので既定 0.5 秒ごとに再計画し、その間は §5 の `avoid_circles` で反応的に避ける。
他のロボットは現在位置に加えて、速度 $(v,\omega)$ から 0.5 秒先の位置も占有にする。

### 8.4 モードとの分担

- `step()`: モード固有の制御（目標姿勢へのフィードバック、隊形、押し出しなど）で速度指令を作る。
- `navigation_targets()`: 経路計画で向かう目標点を返す。返さないロボットにはナビゲーションは何もしない。
- `navigation_speeds()`: 経路追従中の巡航速度 [m/s] を返す（任意）。省略したロボットは既定 0.20 m/s。
  モードの瞬間的な `vx` ではなく、こちらを経路追従の速度に使う。
- `Mode.run()`: `step()` の結果のうち、目標点までの距離が `handover_distance`（既定 12 cm）より
  遠いロボットだけを経路追従指令に置き換える。目標が近いとき・巡航速度が 0 のとき・経路が
  見つからないときはモードの指令に任せるので、最後の姿勢合わせや物資押しを壊さない。

各モードが返す目標点と巡航速度は次のとおり。

| モード | ナビゲーションの目標点 | 巡航速度 |
|---|---|---|
| フォーメーション | 自分の隊形スロット | `max_speed` |
| 包囲 | 対象のまわりの自分のスロット | `max_speed` |
| 人追従（隊列） | 軌跡上で「leader の `gap` 後ろ」になる点 | 隊列誤差に比例する P 制御の速度 |
| 協調物資運搬 | 把持隊形へ向かう間はスロット。押し出し中は物資の搬送経路（下記）の先読み点 | `max_speed` |

協調物資運搬の押し出し中は、物資を押す経路そのものを占有格子と A* で計画する
（`modes/cooperative_transport.py`）。障害物・人・他のロボットを物資の半径で膨らませ、
経路上の先読み点へ向かって押す。これにより、物資を障害物にぶつけずに目標まで運べる。
同じ 1 点を複数台が目指す場合も、モード側がスロットを分散しているので最後に押し合わない
（§3.2 のハンガリアン法）。

## 9. 姿勢推定（`fleet/world_model.py`）

ビジョンは約 50–100 ms 遅れて届く。遅延 $\tau$ の間にロボットは $v\tau$ 進んでいるので、
エンコーダから求めた $(v,\omega)$ で観測時刻から現在まで円弧積分して補償する。
ビジョンが 0.5 s 以内途切れた場合も同じ方法で推測航法し、それ以上は「姿勢不明」として制御対象から外す。

発展課題: カルマンフィルタ（EKF）でビジョン・エンコーダ・ジャイロを融合すると、ノイズと遅延に一層強くなる。

## 10. 参考文献

1. R. Siegwart, I. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed., MIT Press, 2011.
2. Y. Kanayama et al., "A stable tracking control method for an autonomous mobile robot," ICRA, 1990.
3. R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," CMU-RI-TR-92-01, 1992.
4. H. W. Kuhn, "The Hungarian method for the assignment problem," 1955.
5. M. A. Lewis, K.-H. Tan, "High precision formation control of mobile robots using virtual structures," 1997.
6. P. E. Hart, N. J. Nilsson, B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," *IEEE Trans. Systems Science and Cybernetics*, 1968.
