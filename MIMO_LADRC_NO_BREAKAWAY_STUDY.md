# MIMO-LADRC（BREAKAWAYなし）ソースコード学習ガイド

> 対象：`zlac8015d_hardware` の `experiment/mimo-ladrc-no-breakaway` ブランチ。  
> 確認したコミット：`6375092965dd62762990edf37cfd5047b6465cc8`。  
> **対象はC++のMIMO-LADRC実装だけ。** 計測用Python・別制御モードの詳細は説明しない。  
> ブロック数式にはGitHubで表示できる `math` フェンスを使用。

## 1. このプログラムの目的

差動二輪ロボットの機体並進速度 `v` [m/s] と旋回角速度 `omega` [rad/s] をフィードバックし、左右の**目標モータ電流** `I_L,I_R` [A] を求める。

このモードの重要な特徴は、**静止状態で非ゼロ目標が来た最初の制御周期からMIMO-LADRCを適用する**点にある。従来のBREAKAWAY電流、READY待ち、TRACK電流下限、方向反転時のBREAKAWAYへの戻りは、専用分岐では実行しない。

対象のC++ソースは次の3ファイル：

1. [`include/zlac8015d_hardware/mimo_control.hpp`](include/zlac8015d_hardware/mimo_control.hpp)：`DifferentialDriveModel`、`Ladrc`、`wheel_overspeed()`。
2. [`include/zlac8015d_hardware/zlac8015d_system.hpp`](include/zlac8015d_hardware/zlac8015d_system.hpp)：ROS 2 hardware_interface宣言、状態、パラメータ。
3. [`src/zlac8015d_system.cpp`](src/zlac8015d_system.cpp)：`on_init()`、`on_activate()`、`read()`、`write()`、CAN電流送信。

全体の流れ：

```text
目標車輪角速度（左/右） ───────────────┐
                                      ↓
エンコーダ実測車輪角速度 ──→ 機体座標への変換 [v, omega]
                                      ↓
                        MIMO-ESO：z1（速度推定）、z2（総外乱推定）
                                      ↓
                        MIMO-LADRC：左右の生電流 I_raw
                                      ↓
                        ±8 Aの飽和・20 A/sの変化率制限
                                      ↓
                        CANopen RPDO0 → 左右モータ
                                      ↓
                        送信電流を次周期のESO入力へ保存
```

## 2. 車輪座標と機体座標：まず順番を覚える

プログラムの左右の順番は **`[left,right]`**。旋回は反時計回りを正とする。

```math
\boldsymbol q=\begin{bmatrix}\omega_L\\\omega_R\end{bmatrix},
\quad
\boldsymbol y=\begin{bmatrix}v\\\omega\end{bmatrix},
\quad
\boldsymbol u=\begin{bmatrix}I_L\\I_R\end{bmatrix}
```

車輪半径 `r=0.064 m`、車輪間隔 `ell=0.354 m`。車輪角速度から機体速度への変換は

```math
v=\frac r2(\omega_L+\omega_R),\qquad
\omega=\frac r\ell(\omega_R-\omega_L)
```

```math
\boldsymbol y=\boldsymbol T\boldsymbol q,\qquad
\boldsymbol T=
\begin{bmatrix}
r/2&r/2\\
-r/\ell&r/\ell
\end{bmatrix}
=
\begin{bmatrix}
0.032&0.032\\
-0.18079&0.18079
\end{bmatrix}
```

これが`mimo_control.hpp`の`DifferentialDriveModel`の`T`。逆行列`T_inv`は`body_to_wheel()`で使う。

```cpp
const double half_r = wheel_radius * 0.5;
const double yaw_scale = wheel_radius / wheel_separation;
T = {{{half_r, half_r}, {-yaw_scale, yaw_scale}}};
T_inv = inverse(T);
```

【理解の要点】`v,omega`はエンコーダ値から計算した機体速度である。外部位置センサで測った対地速度ではない。車輪がスリップすれば機体の真の運動とずれる。

## 3. 入力電流と加速度を結びつける `B` 行列

各輪を電流入力の一次モデルで近似する：

```math
\dot{\boldsymbol q}
=\boldsymbol D_b\boldsymbol u+\boldsymbol f_q,
\qquad
\boldsymbol D_b=
\begin{bmatrix}b_L&0\\0&b_R\end{bmatrix}
```

`b_L,b_R`は入力公称ゲインで、実機の設定では左右とも18 [(rad/s²)/A]。`f_q`は車輪方向の総外乱加速度。

上式を`T`で変換して、

```math
\dot{\boldsymbol y}=\boldsymbol B\boldsymbol u+\boldsymbol f,
\quad
\boldsymbol B=\boldsymbol T\boldsymbol D_b
=\begin{bmatrix}
rb_L/2&rb_R/2\\
-rb_L/\ell&rb_R/\ell
\end{bmatrix}
```

となる。実際の数値は

```math
\boldsymbol B\simeq
\begin{bmatrix}
0.576&0.576\\
-3.25424&3.25424
\end{bmatrix}.
```

`DifferentialDriveModel`内では次のように作る。

```cpp
B = {{{T[0][0] * b_left, T[0][1] * b_right},
      {T[1][0] * b_left, T[1][1] * b_right}}};
B_inv = inverse(B);
```

**`B_inv`が重要**。LADRCで求めた「必要な機体加速度」を、左右の「必要な電流指令」へ変換する。

注意：`b0=18`がどの運転状態にも正確な物理値という意味ではない。静止摩擦、無駄時間、指令と実電流の不一致などは`f`として扱われ得る。

## 4. ESOの役割と実際の更新式

`mimo_control.hpp`内の`Ladrc`クラスは、次の2つの状態ベクトルを持つ。

```cpp
Vec2 z1{{0.0, 0.0}};
Vec2 z2{{0.0, 0.0}};
```

| 状態 | `[0]`の意味 | `[1]`の意味 |
|---|---|---|
| `z1` | 並進速度推定 [m/s] | 角速度推定 [rad/s] |
| `z2` | 並進方向総外乱 [m/s²] | 旋回方向総外乱 [rad/s²] |

一次のLESOは、

```math
\begin{aligned}
\dot{\boldsymbol z}_1
&=\boldsymbol z_2+\boldsymbol B\boldsymbol u_{\mathrm{prev}}
+2\boldsymbol\Omega_o(\boldsymbol y-\boldsymbol z_1),\\
\dot{\boldsymbol z}_2
&=\boldsymbol\Omega_o^2(\boldsymbol y-\boldsymbol z_1),\\
\boldsymbol\Omega_o&=
\mathrm{diag}(\omega_{o,v},\omega_{o,\omega}).
\end{aligned}
```

現行値は`omega_o,v=20`、`omega_o,omega=20` [rad/s]。ESO誤差系の連続時間理想モデルを`(s+omega_o)^2`とする極配置に対応する。

`Ladrc::step()` の抜粋：

```cpp
const Vec2 known_acceleration =
  multiply(model.B, previous_current);

for (std::size_t i = 0; i < 2; ++i) {
  const double error = measured[i] - z1[i];
  const double z1_dot =
    z2[i] + known_acceleration[i] + 2.0 * wo[i] * error;
  const double z2_dot = wo[i] * wo[i] * error;
  z1[i] += dt * z1_dot;
  z2[i] += dt * z2_dot;
}
```

処理の読み方は「前回電流から予測加速度を出す → 速度推定誤差を計算 → 推定速度と総外乱を更新」。`z1+=dt*z1_dot`等は**陽的Euler離散化**である。

**ここでいう`previous_current`は実測電流ではない。** `write()`が保持している**前周期に送信した目標電流**`last_current_cmd`が使われる。実際の電流が目標値へ追従しなければ、その違いもESOが推定する総外乱の一部になり得る。

`z2`を「摩擦力そのもの」「実電流」「外乱トルク[Nm]」と読み替えないこと。

## 5. MIMO-LADRC制御則：ESOの推定値をどう使うか

ESOを更新した直後、同じ`Ladrc::step()`で左右の生電流を求める。

```math
\boldsymbol a_{\mathrm{des}}
=\boldsymbol a_{\mathrm{ff}}
+\boldsymbol\Omega_c(\boldsymbol y_{\mathrm{ref}}-\boldsymbol z_1)
-\boldsymbol z_2
```

```math
\boldsymbol u_{\mathrm{raw}}
=\boldsymbol B^{-1}\boldsymbol a_{\mathrm{des}},
\qquad
\boldsymbol\Omega_c=
\mathrm{diag}(\omega_{c,v},\omega_{c,\omega})
```

現行値は`omega_c,v=5`、`omega_c,omega=5` [rad/s]。

```cpp
desired_acceleration[i] =
  acceleration_feedforward[i]
  + wc[i] * (reference[i] - z1[i])
  - z2[i];
return model.acceleration_to_current(desired_acceleration);
```

それぞれの項の役割：

- `wc*(reference-z1)`：目標速度に追従するために要求する加速度。
- `-z2`：ESOによって推定された等価外乱の補償。
- `acceleration_feedforward`：通常停止時に与える目標減速度（通常の一定速度走行ではゼロ）。
- `B_inv`：機体方向の仮想加速度を左右の電流指令に変換。

制御則は機体座標で書かれているが、電流を送る最後の段階では`[左,右]`に戻る。

## 6. `on_init()`・`on_activate()`：何を初期化するか

`src/zlac8015d_system.cpp::on_init()`では起動時の制御モード文字列を判定する。

```cpp
} else if (mode == "mimo_ladrc_no_breakaway") {
  control_mode_ = ControlMode::MIMO_LADRC_TORQUE;
  experimental_no_breakaway_ = true;
}
```

**専用モードの識別は`experimental_no_breakaway_`フラグである**。内部enumは`MIMO_LADRC_TORQUE`だが、他のMIMOモードと同じ経路を実行するわけではない。

`ZLAC_CONTROL_MODE`環境変数があればURDF上の`control_mode`を上書きする。`on_init()`では`b0`、`wc`、`wo`、ホイール寸法、電流上限、タイムアウトを読み取って`mimo_model_`を構築する。実際の値はURDFで上書きされ得るため、ソース内の既定値だけで運転条件を断定しない。

`on_activate()`では、CANソケット・フィードバックPDO・電流出力RPDOを準備し、Profile Torqueモードでサーボを有効化する。ESOと停止状態を初期化する。

```cpp
mimo_ladrc_.reset();
mimo_was_ready_ = false;
no_breakaway_overspeed_latched_ = false;
```

ESOは実際の非ゼロ速度指令が入った最初の周期に`reset_to_measurement()`を呼び、**`z1=実測機体速度、z2=0`**から開始する。

## 7. `read()`：エンコーダ・電流を読み取る

`read()`はCANopenから受信したTPDOを処理する。

| CAN ID（node_id=1の場合） | 内容 | ソースの保存先 |
|---|---|---|
| `0x181` TPDO0 | 左右実角速度（0.1 rpm） | `hw_velocities_[0/1]` [rad/s] |
| `0x281` TPDO1 | 左右実電流（0.1 A） | `hw_efforts_[0/1]` [A] |
| `0x381` TPDO2 | 左右エンコーダ位置（counts） | `hw_positions_[0/1]` [rad] |

左モータではドライバ側の正方向がROS前進と逆のため、速度・電流・位置の左側値を符号反転している。速度換算は`0.1 rpm × (2*pi/60) = 0.0104719755 rad/s`。

`hw_efforts_`が実電流。前周期`last_current_cmd`と区別する。

## 8. `write()`専用分岐：発進からCAN送信まで

[`src/zlac8015d_system.cpp::write()`](src/zlac8015d_system.cpp)

### 8.1 起動中に車輪が止まっていてもLADRCを実行

```cpp
const bool active_reference =
  !zero_stop_hold_ &&
  (std::abs(control_ref_l) > 1.0e-6 ||
   std::abs(control_ref_r) > 1.0e-6);
```

**ここには実車輪速度が正であるという条件がない。** 少なくとも一方の車輪目標が非ゼロで、ゼロ保持でなければLADRCが実行される。

### 8.2 速度・目標・前回電流を機体座標に用意する

```cpp
const mimo::Vec2 measured_body =
  mimo_model_.wheel_to_body(
    {{hw_velocities_[0], hw_velocities_[1]}});
const mimo::Vec2 reference_body =
  mimo_model_.wheel_to_body(
    {{control_ref_l, control_ref_r}});
const mimo::Vec2 previous_current =
  {{current_l_measured, current_r_measured}};
```

`current_l_measured`という変数名に惑わされない。実際には直前に`left_ladrc_.last_current_cmd`を代入しているため、「前回電流**指令**」である。

### 8.3 最初のESO初期化とLADRC演算

```cpp
if (!mimo_was_ready_) {
  mimo_ladrc_.reset_to_measurement(measured_body);
  mimo_was_ready_ = true;
}

const auto raw = mimo_ladrc_.step(
  mimo_model_, reference_body, measured_body,
  previous_current, dt, mimo_wc_, mimo_wo_, body_stop_ff);
```

`raw[0]`は左輪、`raw[1]`は右輪の**制限前の目標電流**。

### 8.4 電流制限

```cpp
const double imax = no_breakaway_current_limit_a_;
const double di = current_rate_limit_a_per_s_ * dt;

double cmd_l = clamp(
  clamp(raw[0], -imax, imax),
  current_l_measured - di,
  current_l_measured + di);
```

右輪も同じ処理で、最後に±`imax`へ再度クリップする。通常設定は**電流上限8 A/輪、変化率20 A/s**。周期5 msでは**一周期に最大0.1 Aしか指令を変化させない**。

### 8.5 ログ用の状態変換

`z1,z2`は機体座標の推定値だが、CSVへ車輪座標の推定値も保存する。

```cpp
const auto estimated_wheels =
  mimo_model_.body_to_wheel(mimo_ladrc_.z1);
const auto disturbance_wheels =
  mimo_model_.body_to_wheel(mimo_ladrc_.z2);
```

`left_ladrc_.z1/z2`、`right_ladrc_.z1/z2`にこの値を格納している。**実験モードで左右独立ESOを別途動かしているわけではない**。

### 8.6 CANへ目標電流を送信

目標電流[A]をmAへ換算し、左だけ符号反転する。

```cpp
const int16_t left_ma = static_cast<int16_t>(
  std::lround(clamp(-left_a * 1000.0, -30000.0, 30000.0)));
const int16_t right_ma = static_cast<int16_t>(
  std::lround(clamp(right_a * 1000.0, -30000.0, 30000.0)));
```

`node_id=1`なら送信IDは`0x201`。左右の`int16`電流値を4バイトの同じRPDO0に格納する。送信した値を`last_current_cmd`に保存し、**次周期のESOのモデル入力にする**。

この分岐はCAN送信後に`return OK`するので、下に記述された従来のBREAKAWAY状態遷移処理へ進まない。

## 9. 発進の1周期目を実際に計算する

【例】静止している機体へ`v_ref=0.10 m/s, omega_ref=0`を指令し、`z1=z2=0`、前回指令電流0、`wc=5`とする。

まず

```math
\boldsymbol a_{\mathrm{des}}
=\begin{bmatrix}5(0.10-0)\\5(0-0)\end{bmatrix}
=\begin{bmatrix}0.50\\0\end{bmatrix}.
```

`B`から左右同じ生電流が必要となる。

```math
0.50=0.576I_L+0.576I_R
\quad\Longrightarrow\quad
I_{L,\mathrm{raw}}=I_{R,\mathrm{raw}}
=\frac{0.50}{1.152}\simeq0.4340\ \mathrm{A}.
```

しかし前回の電流指令が0、`dt=0.005 s`なら、変化率制限が先に効く。

```math
\Delta I_{\max}=20\times0.005=0.10\ \mathrm{A},
\qquad
I_{L,\mathrm{cmd}}=I_{R,\mathrm{cmd}}=0.10\ \mathrm{A}.
```

この例では**BREAKAWAYの0.65 Aをいきなり与えない**。最初のモータ電流指令はLADRC出力と電流変化率制限から決まる。

ESOは、モータが動かず速度予測と測定に差が出ると外乱推定を変化させるが、**静止摩擦を必ず突破できるわけではない**。

## 10. 通常停止（NORMAL STOPとZERO HOLD）

`hw_commands_`が左右とも0になると、`write()`は`normal_stop_ramp_active_`を使って内部の目標角速度を減らす。

- 停止開始時の実車輪速度からランプを開始。
- 車輪角加速度相当の上限は`1.0/0.064=15.625 rad/s²`。
- `body_stop_ff`でLADRCに減速加速度をフィードフォワードする。
- 両輪実速度が`0.50 rad/s`以下になると`zero_stop_hold_=true`。
- ZERO HOLDでは0 Aを指令し、ESOをリセットする。
- 次の非ゼロ目標でZERO HOLDを解除し、ESOを実測速度で再初期化する。

**ZERO HOLDは車輪を固定する機械ブレーキではない。** 単に目標電流0 Aを保持する。

## 11. 過速度・フィードバック異常

`mimo_control.hpp::wheel_overspeed()`は左・右の実測角速度の絶対値を調べる。半径0.064 m、車輪周速度2.0 m/sに相当する閾値は

```math
\omega_{\mathrm{limit}}=\frac{2.0}{0.064}
=31.25\ \mathrm{rad/s}.
```

`read()`でいずれか一輪がこれ以上なら`no_breakaway_overspeed_latched_=true`とする。以降`write()`は**左右0 A指令**を優先する。速度が下がっても`cmd_vel=0`でも解除されず、制御器の再初期化が必要。

速度フィードバックが未受信なら0 Aを送り、最終速度TPDOから**30 ms**超の欠落でも0 Aを送り`ERROR`を返す（既定設定）。

**このモードに片輪拘束時の両輪連動制動はない。** 拘束状態でもLADRCが最大8 Aを要求する場合がある。0 A指令も物理非常停止ではないため、手で回転輪を止める試験は行わない。

## 12. なぜ現在は左右独立LADRCと数理的に等価なのか

現状の機体座標の帯域幅は

```math
\boldsymbol\Omega_c=5\boldsymbol I,\qquad
\boldsymbol\Omega_o=20\boldsymbol I.
```

左右車輪座標へ変換すると、

```math
\boldsymbol T^{-1}\boldsymbol\Omega_c\boldsymbol T
=\boldsymbol T^{-1}(5\boldsymbol I)\boldsymbol T
=5\boldsymbol I.
```

ESO側も`20I`なので同様。理想的なモデル・座標変換・初期化条件では、生電流制御則は各車輪独立のLADRCへ変換できる。

```math
I_L=\frac{5(\omega_{L,\mathrm{ref}}-\hat\omega_L)-\hat f_L}{b_L},
\qquad
I_R=\frac{5(\omega_{R,\mathrm{ref}}-\hat\omega_R)-\hat f_R}{b_R}.
```

左右で異なる外乱が与えられると、機体座標の旋回方向外乱は

```math
f_\omega=\frac r\ell(f_{q,R}-f_{q,L})
```

となる。各車輪のESOが正確に外乱を補償できれば独立制御でも直進は可能だが、実際には推定遅れ・飽和・静止摩擦の差で車輪速度偏差が残り得る。

例えば【**設計例であって現行設定ではない**】`wc_v=5, wc_omega=10`とすれば、

```math
\boldsymbol T^{-1}
\begin{bmatrix}5&0\\0&10\end{bmatrix}
\boldsymbol T
=\begin{bmatrix}7.5&-2.5\\-2.5&7.5\end{bmatrix}.
```

非対角成分ができ、片輪の速度誤差が他輪の指令にも影響する。現行ブランチのゲインは`5,5`と`20,20`のままである。

## 13. 研究時に見る診断データ

内部CSV：`/tmp/mimo_ladrc_no_breakaway_200hz.csv`

| 列 | 意味 |
|---|---|
| `ref_left_rad_s,ref_right_rad_s` | 左右目標角速度 |
| `vel_left_rad_s,vel_right_rad_s` | 左右実測角速度 |
| `cmd_current_left_A,cmd_current_right_A` | CANへ送った**指令**電流 |
| `actual_current_left_A,actual_current_right_A` | ドライバから返った**実**電流 |
| `hat_v_m_s,hat_omega_rad_s` | ESOの`z1` |
| `hat_d_v_m_s2,hat_d_omega_rad_s2` | ESOの`z2` |
| `z1_left,z1_right` | `z1`を車輪座標へ逆変換した値 |
| `z2_left,z2_right` | `z2`を車輪座標へ逆変換した値 |
| `wheel_overspeed_latched` | 過速度ラッチの0/1 |

`ref_v_m_s`等は元の`hw_commands_`から算出され、通常停止中に`write()`内部で作った減速ランプ参照と一致しない場合がある。

`z2`は**総外乱の加速度**であり、実電流でも摩擦トルクでもない。モデル入力を「実電流」ではなく「指令電流」にしている点も考慮して解析する。

## 14. 理解確認（コードを読みながら答える）

1. ESOの`z1`と`z2`はそれぞれ何を表し、どの単位か。
2. なぜ車輪が0 rad/sでもこのモードはLADRCを開始できるか。
3. ESOの`previous_current`は実測電流と指令電流のどちらか。
4. 生電流が4 A、直前0 A、`dt=0.005 s`のとき、送信される最初の電流指令は何Aか。
5. `omega_c,v=omega_c,omega`の場合、なぜ各輪独立の演算に還元できるか。
6. 左輪の速度だけが低下した場合、機体の旋回角速度はどちらに偏るか。
7. ZERO HOLDと、車輪周速度2 m/sの過速度ラッチはどう違うか。

<details>
<summary>解答</summary>

1. `z1`は機体並進速度[m/s]・角速度[rad/s]の推定、`z2`はそれぞれの総外乱加速度[m/s²]・[rad/s²]。
2. `active_reference`の判定に車輪実速度条件がなく、非ゼロ目標でLADRC専用分岐へ入るため。
3. `last_current_cmd`という前周期の**指令**電流。
4. `20×0.005=0.1 A`。変化率で制限される。
5. `T^{-1}(wc I)T=wc I`のため。
6. `omega=(r/ell)(omega_R-omega_L)`。左輪だけ遅くなると**正方向**に偏る。
7. ZERO HOLDは通常停止終了後の0 A保持で新しい非ゼロ指令で解除。過速度ラッチは制御器を再初期化するまで解除しない。

</details>

---

## 15. 推奨する学習順序

1. `mimo_control.hpp::DifferentialDriveModel`の行列`T,B,B_inv`を紙に書き、数値代入をする。
2. `mimo_control.hpp::Ladrc::step()`のESO更新式と制御則を各1行ずつ式に対応付ける。
3. `zlac8015d_system.hpp`の`mimo_ladrc_`と`mimo_wc_,mimo_wo_`を確認する。
4. `zlac8015d_system.cpp::on_init()`で専用モードがどう有効になるか読む。
5. `read()`の`hw_velocities_`と`hw_efforts_`の違いを確認する。
6. `write()`の`active_reference`→`reset_to_measurement`→`step`→電流制限→RPDO0送信を追う。
7. `tests/test_no_breakaway_ladrc.cpp`で数値計算と安全閾値のチェックを確認する。

**到達目標：** 「目標速度→ESOによる速度・外乱推定→MIMO-LADRC→左右電流指令→CANopen送信」を、ソースの変数名と式の両方で説明できること。
