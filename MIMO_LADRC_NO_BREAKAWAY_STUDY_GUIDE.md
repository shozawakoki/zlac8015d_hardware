# 学習用：BREAKAWAYなし MIMO-LADRC のソースコードを読む

**対象プログラム：`zlac8015d_hardware` / `experiment/mimo-ladrc-no-breakaway`**  
**確認したソースのコミット：`6375092965dd62762990edf37cfd5047b6465cc8`**  
**対象モード：`ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway`**  
**想定環境：ROS 2 Humble、ZLAC8015D、200 Hz制御周期**

> この資料は学習専用。**制御プログラム・ゲイン・安全機能は変更しない**。`cmd_vel`計測スクリプト、bringup用Python、他の制御モードの詳細説明は対象外とする。以下の説明は上記コミットの実装に基づく。数式の意味と実際の演算順序を区別すること。

## 学習の到達目標

1. なぜ機体速度 `[v, omega]` から左右のモータ電流 `[I_L, I_R]` を計算できるのか説明する。
2. ESOの `z1`、`z2`、`previous_current` の意味を、実際のC++行に対応付ける。
3. **静止状態で速度指令を受けた最初の周期**から、何が起こるか追跡する。
4. 電流上限・電流変化率・正常停止・2 m/s過速度ラッチの実行順序を説明する。
5. 現在の等帯域MIMO-LADRCが、数理的には左右独立制御へ変換できる理由を理解する。

## 0. まず読む3ファイル

| ファイル | 役割 | 最初に見る場所 |
|---|---|---|
| [`include/zlac8015d_hardware/mimo_control.hpp`](include/zlac8015d_hardware/mimo_control.hpp) | 行列・座標変換、`DifferentialDriveModel`、`Ladrc`、`wheel_overspeed` | `DifferentialDriveModel` と `Ladrc::step()` |
| [`include/zlac8015d_hardware/zlac8015d_system.hpp`](include/zlac8015d_hardware/zlac8015d_system.hpp) | 実機との接続状態、速度・電流・ESO状態、上限値 | `experimental_no_breakaway_`、`mimo_ladrc_` |
| [`src/zlac8015d_system.cpp`](src/zlac8015d_system.cpp) | モード設定、CANフィードバック、LADRC呼び出し、電流送信、停止 | `on_init()` → `read()` → `write()` |

**重要：** `mimo_control.hpp` には旧BREAKAWAY用ヘルパーとMIMO-PIDも残っている。また、`zlac8015d_system.cpp` の後半にも旧BREAKAWAY/TRACK処理が残る。しかし、**実験モードの正常な非ゼロ速度指令では、`write()` の専用分岐から電流を送って `return` するので、後続のBREAKAWAY処理には入らない**。

---

## 1. この制御器の入出力

### 1.1 全体を一行で書く

**左右の目標車輪速度 → エンコーダによる機体速度の測定 → MIMO-ESO → MIMO-LADRC → 左右の目標電流 → CANopen送信**。

プログラムが `hardware_interface` から受け取る入力は左右の目標車輪角速度 `hw_commands_` [rad/s]。MIMO演算に渡す直前に `[v_ref, omega_ref]` へ変換する。プログラムの出力は、ドライバ内蔵のProfile Torqueモードに送る**電流指令** [A]。

```text
hw_commands_[0], hw_commands_[1]  ← 目標車輪角速度
          ↓ wheel_to_body()
reference_body = [v_ref, omega_ref]

hw_velocities_[0], hw_velocities_[1] ← エンコーダ速度
          ↓ wheel_to_body()
measured_body  = [v, omega]
          ↓
mimo_ladrc_.step()  ← previous_current, dt, wc, wo
          ↓
raw = [I_L_raw, I_R_raw]
          ↓ 振幅制限・変化率制限
cmd = [I_L, I_R]
          ↓
RPDO0へ左右電流を書き込む
```

### 1.2 変数の順序を間違えない

```math
\boldsymbol q=
\begin{bmatrix}\omega_L\\\omega_R\end{bmatrix},
\qquad
\boldsymbol y=
\begin{bmatrix}v\\\omega\end{bmatrix},
\qquad
\boldsymbol u=
\begin{bmatrix}I_L\\I_R\end{bmatrix}.
```

このソースの車輪順序は**左→右**。角速度 `omega` は機体の鉛直軸周りの旋回速度であり、車輪角速度 `omega_L`、`omega_R` とは異なる。

## 2. `DifferentialDriveModel`：車輪モデルをMIMOに変換する

### 2.1 車輪から機体速度へ

車輪半径 `r`、車輪間隔 `ell` に対して

```math
v=\frac{r}{2}(\omega_L+\omega_R),
\qquad
\omega=\frac{r}{\ell}(\omega_R-\omega_L).
```

~~~cpp
// mimo_control.hpp / DifferentialDriveModel
T = {{{half_r, half_r}, {-yaw_scale, yaw_scale}}};
// half_r = wheel_radius * 0.5
// yaw_scale = wheel_radius / wheel_separation
~~~

よって

```math
\boldsymbol y=\boldsymbol T\boldsymbol q,
\qquad
\boldsymbol T=
\begin{bmatrix}
r/2&r/2\\
-r/\ell&r/\ell
\end{bmatrix}.
```

幾何パラメータが `r=0.064 m`、`ell=0.354 m` の場合、

```math
\boldsymbol T \approx
\begin{bmatrix}
0.032 & 0.032\\
-0.18079 & 0.18079
\end{bmatrix}.
```

- 左右同じ回転なら `omega=0`：直進。
- 左右が逆方向で同じ大きさなら `v=0`：超信地旋回。
- 左右回転速度が異なれば旋回速度が発生する。

### 2.2 電流入力から機体加速度へ

左右車輪の公称一次モデルを

```math
\dot{\boldsymbol q}
=\boldsymbol D_b\boldsymbol u+\boldsymbol f_q,
\qquad
\boldsymbol D_b=\mathrm{diag}(b_L,b_R)
```

とおく。機体速度を微分すると、

```math
\dot{\boldsymbol y}
=\boldsymbol T\dot{\boldsymbol q}
=\underbrace{\boldsymbol T\boldsymbol D_b}_{\boldsymbol B}
\boldsymbol u+\boldsymbol T\boldsymbol f_q.
```

したがって、

```math
\boxed{\dot{\boldsymbol y}=\boldsymbol B\boldsymbol u+\boldsymbol f},
\qquad
\boldsymbol B=
\begin{bmatrix}
rb_L/2&rb_R/2\\
-rb_L/\ell&rb_R/\ell
\end{bmatrix}.
```

~~~cpp
B = {{{T[0][0] * b_left, T[0][1] * b_right},
      {T[1][0] * b_left, T[1][1] * b_right}}};
B_inv = inverse(B);
~~~

**なぜ逆行列が必要か：** LADRCがまず求めるのは、機体の望ましい並進加速度と角加速度。それを実現する左右電流は `u=B^{-1} a_des` で計算するから。

公称値 `b_L=b_R=18` と上記幾何を使うと、

```math
\boldsymbol B\approx
\begin{bmatrix}
0.576&0.576\\
-3.25424&3.25424
\end{bmatrix}.
```

**学習上の注意：** `b0=18` は実験で用いる公称ゲイン。`on_init()` は外部hardwareパラメータ `ladrc_b0_left/right` を読み込み、未指定なら関数呼出し側のフォールバック `1.0` を使う。したがって **`mimo_model_` のヘッダ初期値18が常に実運用で使われるわけではない**。起動時の設定を確認する必要がある。

## 3. `Ladrc` クラス：ESOを理解する

### 3.1 `z1` と `z2`

```cpp
class Ladrc
{
public:
  Vec2 z1{{0.0, 0.0}};
  Vec2 z2{{0.0, 0.0}};
  bool initialized{false};
  ...
};
```

`z1` は機体速度の推定値。

```math
\boldsymbol z_1=
\begin{bmatrix}
\hat v\\\hat\omega
\end{bmatrix}.
```

`z2` は総外乱によって生じる機体加速度の推定値。

```math
\boldsymbol z_2=
\begin{bmatrix}
\hat f_v\\\hat f_\omega
\end{bmatrix},
\qquad
[\hat f_v]=\mathrm{m/s^2},
\quad
[\hat f_\omega]=\mathrm{rad/s^2}.
```

`z2` は**外乱トルク[N m]でも左右実電流[A]でもない**。公称一次モデルに対する等価加速度であり、摩擦・荷重・電流遅れ・モデル誤差などの効果を含み得る。

### 3.2 ESOの連続時間式

機体速度の測定誤差を

```math
\boldsymbol e_o=\boldsymbol y-\boldsymbol z_1
```

とする。ESOは

```math
\dot{\boldsymbol z}_1
=\boldsymbol z_2+\boldsymbol B\boldsymbol u_{\mathrm{prev}}
+2\boldsymbol\Omega_o\boldsymbol e_o,
```

```math
\dot{\boldsymbol z}_2
=\boldsymbol\Omega_o^2\boldsymbol e_o,
\qquad
\boldsymbol\Omega_o=
\mathrm{diag}(\omega_{o,v},\omega_{o,\omega}).
```

現行運用では観測器帯域を各軸20 rad/sに設定している。理想的な1軸誤差系の特性多項式は `(s+omega_o)^2`（重根）である。

### 3.3 C++の1行ずつの意味

**① 既知の入力加速度**

~~~cpp
const Vec2 known_acceleration =
  multiply(model.B, previous_current);
~~~

公称モデルから `B u_previous` を計算する。ここで `previous_current` は**前周期に送信した電流指令**（実電流ではない）。

**② 速度の観測誤差**

~~~cpp
const double error = measured[i] - z1[i];
~~~

実測機体速度とESO推定速度の差。

**③ ESOの状態微分**

~~~cpp
const double z1_dot =
  z2[i] + known_acceleration[i] + 2.0 * wo[i] * error;

const double z2_dot =
  wo[i] * wo[i] * error;
~~~

`i=0` が並進速度、`i=1` が角速度。両軸をループで更新するので、実装は2軸合計4状態。

**④ Euler法で状態更新**

~~~cpp
z1[i] += dt * z1_dot;
z2[i] += dt * z2_dot;
~~~

数学的には

```math
\boldsymbol z_1[k+1]=\boldsymbol z_1[k]
+\Delta t\left(
\boldsymbol z_2[k]+\boldsymbol B\boldsymbol u[k-1]
+2\boldsymbol\Omega_o(\boldsymbol y[k]-\boldsymbol z_1[k])
\right)
```

および

```math
\boldsymbol z_2[k+1]=\boldsymbol z_2[k]
+\Delta t\,\boldsymbol\Omega_o^2
(\boldsymbol y[k]-\boldsymbol z_1[k]).
```

**⑤ 重要：制御計算に用いるのは更新後の`z1,z2`。** `step()` は上記のループが終了した後に電流指令を計算する。連続時間式だけ見ると分かりにくいが、実際は離散時間のこの順序で動作する。

### 3.4 摩擦に対してESOは何を見ているか

静止状態で正の電流を指令すると、公称モデルは正方向の加速度を予測する。しかし実際は静止摩擦のため車輪が動かなければ、実測 `y` と推定 `z1` の差が生じる。この差を通じて `z2` が**運動を妨げる方向の等価外乱**として更新され、LADRCの補償項 `-z2` が電流増加に寄与し得る。

ただし、発進した瞬間から真の静止摩擦を推定できるという意味ではない。**観測誤差の蓄積・入力飽和・電流ループの遅れ**に左右されるため、静止摩擦の大きさによっては発進できない。

## 4. `Ladrc::step()`：MIMO制御則の1行ずつの意味

ESO更新が終わると、希望する加速度を計算する。

```math
\boldsymbol a_{\mathrm{des}}
=\boldsymbol a_{\mathrm{ff}}
+\boldsymbol\Omega_c(\boldsymbol y_{\mathrm{ref}}-\boldsymbol z_1)
-\boldsymbol z_2.
```

~~~cpp
desired_acceleration[i] =
  acceleration_feedforward[i]
  + wc[i] * (reference[i] - z1[i])
  - z2[i];
~~~

3項を区別する：

1. `acceleration_feedforward`：通常停止ランプの際に使う加速度先行項。通常の一定速度指令では0。
2. `wc[i] * (reference[i]-z1[i])`：目標と推定速度の誤差を縮めるフィードバック。
3. `-z2[i]`：ESOで推定された総外乱を打ち消そうとする補償。

左右電流への変換：

```math
\boxed{
\boldsymbol u_{\mathrm{raw}}
=\boldsymbol B^{-1}
\left[
\boldsymbol a_{\mathrm{ff}}+
\boldsymbol\Omega_c(\boldsymbol y_{\mathrm{ref}}-\boldsymbol z_1)
-\boldsymbol z_2
\right]
}
```

~~~cpp
return model.acceleration_to_current(desired_acceleration);
~~~

これは**まだ制限前**の電流要求。実際に送る値は次の `write()` でクリップされる。

## 5. `Zlac8015dSystemHardware` と実行の順序

### 5.1 `on_init()`：モードとパラメータ

~~~cpp
} else if (mode == "mimo_ladrc_no_breakaway") {
  control_mode_ = ControlMode::MIMO_LADRC_TORQUE;
  experimental_no_breakaway_ = true;
}
~~~

このモードは**MIMO-LADRCを使う**ことと、**BREAKAWAY分岐を迂回する**ことを、`experimental_no_breakaway_` フラグで区別している。

`on_init()` は `mimo_wc_`（制御帯域）、`mimo_wo_`（ESO帯域）、`b0`、車輪寸法、電流制限、フィードバックタイムアウトを読込み、`DifferentialDriveModel` を再構築する。

### 5.2 `on_activate()`：実機通信・初期状態

SocketCAN `can0` を開き、フィードバックPDO・電流指令RPDOを構成し、Profile Torqueモードを有効化する。MIMO-LADRCの推定値と状態は起動時にリセットする。

- `mimo_ladrc_.reset()`：`z1=z2=0`、未初期化。
- `mimo_was_ready_=false`：次の有効な速度指令で実測値による初期化が必要。
- `no_breakaway_overspeed_latched_=false`：過速度ラッチを解除。
- `breakaway_target=0`：実験モードでは起動時のBREAKAWAY目標を使用しない。

`on_activate()` でサーボを有効化しても、直ちにLADRCが非ゼロ電流を出すわけではない。`write()` が有効な目標速度を受けたときに初めて演算する。

### 5.3 `read()`：車輪速度・実電流・位置

~~~cpp
hw_velocities_[0] =
  -static_cast<double>(val_l) * RPM01_TO_RAD_PER_SEC;
hw_velocities_[1] =
   static_cast<double>(val_r) * RPM01_TO_RAD_PER_SEC;
~~~

実機で左モータのドライバ正方向がROSの前進正方向と逆なので、**左だけ符号反転**。`read()` はTPDO0の速度、TPDO1の実電流、TPDO2の位置を読み取る。

| フィードバック | CAN-ID (node=1) | 実装されている換算 |
|---|---|---|
| 左右実速度 `606Ch` | `0x181` | 0.1 rpm → rad/s、左のみ符号反転 |
| 左右実電流 `6077h` | `0x281` | 0.1 A → A、左のみ符号反転 |
| エンコーダ位置 `6064h` | `0x381` | count → rad、累積カウントと32bit折返し補正 |

`read()` で受け取った `hw_efforts_` は**実電流**。一方、後述の`previous_current`は**前回の目標電流**。この2つを同じ変数と考えない。

### 5.4 `write()`：優先順位が重要

`write()` の実行順（BREAKAWAYなし実験モード）：

```text
1. 過速度ラッチが成立済みか？
   YES → 両輪0 Aで終了
2. 車輪速度フィードバックは届いているか？
   NO  → 両輪0 Aで終了
3. 最新速度フィードバックが30 ms以内か？
   NO  → 両輪0 Aを試み、ERROR
4. 制御周期dtを取得（不正なら0.005秒、
   その後0.001～0.020秒に制限）
5. 指令がゼロか？ → 通常停止ランプ / ZERO HOLD
6. 有効な速度参照があるか？
   NO  → ESO reset、両輪0 Aで終了
7. 車輪速度を機体速度へ変換
8. ESOが未初期化なら実測機体速度から初期化
9. ESOを1周期更新 → MIMO-LADRC電流計算
10. 電流振幅と電流変化率の制限
11. 診断用のESO車輪座標値を更新
12. CANopenで左右目標電流を送信し、CSV記録
13. return（旧BREAKAWAY処理には進まない）
```

これが理解できれば、主要な実装経路は把握できる。

## 6. `write()` の実験専用分岐を読む

### 6.1 LADRCの演算開始条件

~~~cpp
if (experimental_no_breakaway_) {
  const bool active_reference =
    !zero_stop_hold_ &&
    (std::abs(control_ref_l) > 1.0e-6 ||
     std::abs(control_ref_r) > 1.0e-6);
~~~

**片輪でも非ゼロの目標速度があればLADRC演算へ進む。** ここに「目標方向へ車輪が実際に回り始めたか」「BREAKAWAY電流を何秒印加したか」というゲートはない。

### 6.2 ESO初期化

~~~cpp
if (!mimo_was_ready_) {
  mimo_ladrc_.reset_to_measurement(measured_body);
  mimo_was_ready_ = true;
}
~~~

```math
\boldsymbol z_1(0)=\boldsymbol y(0),
\qquad
\boldsymbol z_2(0)=
\begin{bmatrix}0\\0\end{bmatrix}.
```

静止から開始なら通常は `z1=[0,0]`。**外乱推定は0から始まる**。車輪が発進しなくても、このフラグは保持され、ESOは次周期以降も更新される。方向を反転しただけではESOをリセットしない。

### 6.3 前周期電流とESOへの入力

~~~cpp
const double current_l_measured =
  left_ladrc_.last_current_cmd;
const double current_r_measured =
  right_ladrc_.last_current_cmd;
...
const mimo::Vec2 previous_current =
  {{current_l_measured, current_r_measured}};
~~~

**名称に注意：** `current_l_measured` というローカル変数名だが、内容は `last_current_cmd`（前回送信した**指令値**）。実測した電流ではない。

ESOで `B u_previous` を使っており、**電流指令が実際には追従できない差**も等価外乱に入り得る。`I_actual` と `I_cmd` を比較する研究では特に重要。

### 6.4 LADRC呼び出し

~~~cpp
const auto raw = mimo_ladrc_.step(
  mimo_model_, reference_body, measured_body, previous_current,
  dt, mimo_wc_, mimo_wo_, body_stop_ff);
~~~

| 渡す変数 | 正確な意味 |
|---|---|
| `mimo_model_` | `T`、`B`、`B_inv` を含むモデル |
| `reference_body` | 目標機体速度 `[v_ref,omega_ref]` |
| `measured_body` | エンコーダ換算の機体速度 `[v,omega]` |
| `previous_current` | 左右**前周期指令電流** [A] |
| `dt` | 現周期長（秒） |
| `mimo_wc_` | 速度追従帯域 `[wc_v,wc_omega]` |
| `mimo_wo_` | ESO帯域 `[wo_v,wo_omega]` |
| `body_stop_ff` | 停止ランプ時の加速度フィードフォワード |

### 6.5 上限と変化率の処理

~~~cpp
const double imax = no_breakaway_current_limit_a_;
const double di = current_rate_limit_a_per_s_ * dt;
double cmd_l = clamp(
  clamp(raw[0], -imax, imax),
  current_l_measured - di,
  current_l_measured + di);
~~~

右輪も同じ演算。`imax=8 A`、`current_rate_limit=20 A/s`、`dt=0.005 s` なら、1周期で変化できる電流は最大 `0.1 A`。

```math
I_{\mathrm{cmd}}[k]=
\mathrm{clip}\left(
\mathrm{clip}(I_{\mathrm{raw}}[k],-8,8),
I_{\mathrm{cmd}}[k-1]-0.1,
I_{\mathrm{cmd}}[k-1]+0.1
\right).
```

最後に振幅制限を再適用する。これは**LADRCの外にあるアクチュエータ制限**。`raw` が8 Aを超えても、実際の目標電流が8 Aを超えることは許容しない。

### 6.6 左右モータへのCAN送信

~~~cpp
const int16_t left_ma = static_cast<int16_t>(
  std::lround(clamp(-left_a * 1000.0, -30000.0, 30000.0)));
const int16_t right_ma = static_cast<int16_t>(
  std::lround(clamp(right_a * 1000.0, -30000.0, 30000.0)));
experimental_frame.can_id = 0x200u + node_id_;
experimental_frame.can_dlc = 4;
~~~

左のみ符号反転。node_id=1なら**CAN-ID `0x201`**。前半2バイトが左電流、後半2バイトが右電流、いずれもmA単位の符号付き16bit整数。

送信後、`last_current_cmd` を更新するので、次周期のESOにはこの電流指令が渡される。

**注意：** 0 Aを送信する動作は機械ブレーキや安全認証済み非常停止ではない。

## 7. 静止からの発進を数値で追跡する

例として、静止状態 `[omega_L,omega_R]=[0,0]`、左右`b0=18`、`wc=5`、`wo=20`、`dt=0.005 s`、`v_ref=0.10 m/s`、`omega_ref=0`を考える。

### 7.1 最初の周期

目標車輪角速度は

```math
\omega_{L,\mathrm{ref}}=\omega_{R,\mathrm{ref}}
=\frac{0.10}{0.064}=1.5625\ \mathrm{rad/s}.
```

ESOは実測値で初期化され、初期`z1=z2=0`。前周期指令電流`u_prev=0`なので、最初のESO更新は変化しない。

LADRCが要求する生電流は左右とも

```math
I_{\mathrm{raw}}
=\frac{5(1.5625-0)-0}{18}
\approx0.4340\ \mathrm A.
```

しかし20 A/sの変化率制限により、

```math
I_{\mathrm{cmd}}[1]=
\min(0.4340,\;20\times0.005)
=\boxed{0.10\ \mathrm A}.
```

**発進用の固定0.65 Aを直接流すのではなく、LADRCが計算した電流を制限して送る。** これが本モードの特徴。

### 7.2 まだ動かなければ

次周期は `u_prev=[0.1,0.1]` となり、モデルは加速すると予測する。実際に速度が0のままならESO誤差が発生し、`z2` が反対方向の等価外乱を学習して、次の電流指令に反映する。

ただしこの数値例は**最初の1周期だけを計算したもの**。実電流が0.10 Aになったか、車輪がその後何秒で発進するかは、外部負荷と電流応答が分からない限り断定できない。

## 8. 停止処理と安全機能

### 8.1 通常の速度ゼロ指令

`write()` は目標車輪速度が左右とも0になったとき、実測車輪速度から減速用参照値を始める。

- 目標減速の大きさ：`1.0 m/s²`相当。車輪角減速度では `1.0/0.064=15.625 rad/s²`。
- `normal_stop_ramp_active_` の間は、LADRCの `body_stop_ff` に減速の加速度成分を加える。
- 左右の実測車輪速度がともに`0.50 rad/s`以下になると`zero_stop_hold_=true`となり、0 A指令を保持する。
- ZERO HOLDになった場合は`mimo_ladrc_.reset()`、`mimo_was_ready_=false`にするので、次回発進時のESOは再初期化される。

**通常停止は制御対象の減速度を厳密に固定する機能ではない**。実際の停止距離はモータ能力、路面、荷重で変化する。

### 8.2 車輪周速度の過速度ラッチ

`mimo_control.hpp::wheel_overspeed()`：

~~~cpp
const double limit_rad_s =
  max_peripheral_speed_m_s / wheel_radius_m;
return std::abs(measured_wheel_rad_s[0]) >= limit_rad_s ||
       std::abs(measured_wheel_rad_s[1]) >= limit_rad_s;
~~~

現行では車輪周速度2.0 m/s、半径0.064 mに対応し、

```math
\omega_{\mathrm{limit}}=\frac{2.0}{0.064}
=\boxed{31.25\ \mathrm{rad/s}}
\quad(\text{約 }298.4\ \mathrm{rpm}).
```

左右どちらか一輪がこの値以上なら、`read()` は `no_breakaway_overspeed_latched_=true` を設定する。その後`write()` が**LADRCより先に**`send_zero_command()`して終了する。

- 車輪が遅くなっても、0速度指令が来ても、ラッチは解除されない。`on_activate()` の再初期化が必要。
- 判定は**車輪周速度**であり、ロボット中心の実並進速度ではない。
- CAN速度フィードバックを受信できない場合は別の欠落監視で0 Aを指令する。
- 0 Aは回転を瞬時に止めるものではない。

### 8.3 センサ欠落

`velocity_feedback_received_==false` の間は0 A。最後の速度TPDOの受信から`feedback_timeout_s=0.030 s`より時間が経過したら、0 A送信を試みて`ERROR`を返す。これは過速度ラッチとは独立の処理。

### 8.4 現在**存在しない**保護

ユーザー指定に従って、実験用分岐には以下を入れていない：

- 片輪拘束時に機体速度誤差から他輪を連動制動する処理。
- 片輪静止かつ高電流で両輪を0 Aにするタイマ保護。

**結果：** 片輪拘束で車輪速度が低くても、LADRCは誤差を打ち消すため**最大8 Aの目標電流**を要求し得る。過速度2 m/s保護は、低速で拘束されたモータの過熱を防ぐ機能ではない。手で回転車輪を止めて確かめないこと。

## 9. MIMOなのに左右独立と等価になる条件

**本ソースでいうMIMOは機体座標の2入力2出力モデルを使うという実装構造**。左右輪間に必ず相互フィードバックが生じるとは限らない。

現在の帯域が

```math
\boldsymbol\Omega_c=
\mathrm{diag}(5,5)=5\boldsymbol I,\qquad
\boldsymbol\Omega_o=
\mathrm{diag}(20,20)=20\boldsymbol I
```

で、`B=T D_b` とすると、車輪座標に戻した制御ゲインは

```math
\boldsymbol T^{-1}\boldsymbol\Omega_c\boldsymbol T
=\boldsymbol T^{-1}(5\boldsymbol I)\boldsymbol T
=5\boldsymbol I.
```

ESO側も同様に等帯域なので、車輪座標の推定状態 `T^{-1}z1`、`T^{-1}z2` は左右で独立なESOとして記述できる。生電流指令を車輪座標で表すと

```math
I_L=\frac{5(\omega_{L,\mathrm{ref}}-\hat\omega_L)-\hat f_L}{b_L},
\qquad
I_R=\frac{5(\omega_{R,\mathrm{ref}}-\hat\omega_R)-\hat f_R}{b_R}.
```

つまり**理想的な公称モデルと同一帯域・初期化条件では、機体座標のMIMO-LADRC演算は2つの車輪別LADRCへ座標変換できる**。

左右に異なる外乱 `f_{q,L}, f_{q,R}` がある場合、

```math
f_\omega=\frac r\ell(f_{q,R}-f_{q,L})
```

なので、左右の速度追従誤差が残ると旋回偏差が生じる。ただし車輪別ESOでも外乱を十分に打ち消せれば直進可能である。

**例として**`wc_v=5, wc_omega=10`と異なる値にした場合、

```math
\boldsymbol T^{-1}
\begin{bmatrix}5&0\\0&10\end{bmatrix}
\boldsymbol T=
\begin{bmatrix}7.5&-2.5\\-2.5&7.5\end{bmatrix}.
```

非対角項により左右の誤差フィードバックが結合する。これは**考え方の例**であり、現在の実装パラメータを変更したわけではない。帯域差を大きくすれば必ず直進性能が改善するという保証もない。

## 10. ログをプログラムの理解に使う

`src/zlac8015d_system.cpp` では、BREAKAWAYなし実験モードに対して次の内部CSVを作る。

`/tmp/mimo_ladrc_no_breakaway_200hz.csv`

| 列名 | 制御器内での意味 |
|---|---|
| `ref_left_rad_s`、`ref_right_rad_s` | `hw_commands_`：入力された車輪目標角速度 |
| `vel_left_rad_s`、`vel_right_rad_s` | `hw_velocities_`：TPDOからの実測角速度 |
| `cmd_current_left_A`、`cmd_current_right_A` | 制限を通過して送信した指令電流 |
| `actual_current_left_A`、`actual_current_right_A` | `hw_efforts_`：TPDOからの実電流 |
| `hat_v_m_s`、`hat_omega_rad_s` | ESOの`z1`（機体座標） |
| `hat_d_v_m_s2`、`hat_d_omega_rad_s2` | ESOの`z2`（機体座標） |
| `z1_left/right`、`z2_left/right` | ESO推定状態を車輪座標に戻した診断値 |
| `wheel_overspeed_latched` | 過速度停止のラッチ状態（0/1） |

特に `cmd_current` と `actual_current` を比較すると、「ESOが正しい電流を前提にしていたか」というモデル上の問題を考察できる。

**CSVの注意点：** このCSVの `time_s` はログ開始を基準にしており、他のROS記録の時刻原点と一致するとは限らない。またハードウェアのログ周期は公称200 Hzだが、実際のサンプル間隔はタイムスタンプで確認する。

## 11. 学習用の理解確認問題

**Q1.** 初期`z2=0`なら、ADRCは静止摩擦をまったく扱えないのか？  
**確認点：** 発進後のESO観測誤差がどの式で`z2`を変化させるかを説明する。

**Q2.** `Ladrc::step()` へ渡す `previous_current` は、モータの実測電流なのか？  
**確認点：** `last_current_cmd` と `hw_efforts_` の代入元を確認する。

**Q3.** なぜ`v_ref=0.1 m/s`で発進する最初の周期は、LADRCの生電流約0.434 Aなのに実指令0.1 Aになるのか？  
**確認点：** 20 A/s × 0.005秒を計算する。

**Q4.** `wc_v=wc_omega` のとき、機体座標で計算すること自体が左右協調制御を実現するのか？  
**確認点：** `T^{-1} (wc I) T` を計算する。

**Q5.** 左車輪を固定し、右だけが回転した場合、過速度に至らなければ左右電流が自動で0になるか？  
**確認点：** 存在する保護と削除済み保護を区別する。

**Q6.** 通常停止`cmd_vel=0`、CANフィードバック欠落、過速度ラッチでは、どの時点でESOが更新されるか、電流指令が0になる条件は何か？  
**確認点：** `write()` の処理順を上から読む。

### 自分で確認できるオフラインテスト

以下の実行は**モデルの計算だけ**で、CANへ指令を出さない。

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware
g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude \
  tests/test_no_breakaway_ladrc.cpp -o /tmp/test_no_breakaway_ladrc
/tmp/test_no_breakaway_ladrc
~~~

`tests/test_no_breakaway_ladrc.cpp` は、静止からの指令符号、外乱推定を含む簡易シミュレーション、2 m/s判定を確認する。ただしテスト内の「静止摩擦の仮想試験」は**局所的に1.0 Aの制限を使う**。これは**実機用分岐の8.0 Aとは別のテスト条件**なので混同しない。テストが通ってもCANopenの実機挙動と機械的安全が保証されるわけではない。

---

## まとめ：学習で最も重要な対応関係

| 制御工学の概念 | ソース内の実装 |
|---|---|
| 車輪座標→機体座標 | `DifferentialDriveModel::wheel_to_body()` |
| 公称入力行列 `B` | `DifferentialDriveModel::B` |
| 速度推定 `z1` | `Ladrc::z1` |
| 総外乱推定 `z2` | `Ladrc::z2` |
| ESOの更新 | `Ladrc::step()` 前半 |
| LADRC制御則 | `Ladrc::step()` 後半 |
| BREAKAWAYなしの発進 | `Zlac8015dSystemHardware::write()` の `experimental_no_breakaway_` 分岐 |
| 電流振幅・変化率制限 | 同分岐の `clamp()` |
| モータ電流の送信 | `send_experimental()`（`write()`内lambda） |
| エンコーダ・電流の読み込み | `Zlac8015dSystemHardware::read()` |
| 通常停止 | `normal_stop_ramp_active_`、`zero_stop_hold_` |
| 過速度2 m/s停止 | `wheel_overspeed()` と `no_breakaway_overspeed_latched_` |

学習は、**`DifferentialDriveModel` → `Ladrc::step()` → `write()` の実験専用分岐 → `read()` → 停止・保護**の順で進めると、数学と実機プログラムの関係を追いやすい。
