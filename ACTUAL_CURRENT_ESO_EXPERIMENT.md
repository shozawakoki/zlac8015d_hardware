# MIMO-LADRC：ESO実電流入力版（BREAKAWAYなし）

対象ブランチ: `experiment/mimo-ladrc-no-breakaway-actual-current-eso`

## 目的
従来のMIMO-LADRC（ESOに前周期の指令電流を使用）と、CANopen TPDO1で受信した実電流をESOに使用する方式を比較する。今回、**実電流入力版のみ、通常停止用の減速ランプ・停止加速度フィードフォワード・ZERO HOLDを通らず、目標速度がゼロでもMIMO-LADRCを連続実行する**。制御ゲイン・公称ゲイン・電流上限や変化率は変更しない。既存の`main`は変更していない。

| 起動モード | ESO入力（車輪順序 [L,R]） |
| --- | --- |
| `mimo_ladrc_no_breakaway` | 前回送信した指令電流 `[left_ladrc_.last_current_cmd, right_ladrc_.last_current_cmd]`（通常停止時は従来どおり減速ランプとZERO HOLD） |
| `mimo_ladrc_no_breakaway_actual_current` | 最新の実電流 `[hw_efforts_[0], hw_efforts_[1]]`（CANopen TPDO1、0x280+node_id、6077h:01/02）。**停止もLADRC制御** |

両方式とも、機体座標`y=[v,omega]^T`の2軸LESOと同一の制御則を使用する。

```math
\begin{aligned}
\dot{\boldsymbol z}_1 &= \boldsymbol z_2 + \boldsymbol B_0\boldsymbol u_{\mathrm{ESO}}
  + 2\boldsymbol\Omega_o(\boldsymbol y-\boldsymbol z_1),\\
\dot{\boldsymbol z}_2 &= \boldsymbol\Omega_o^2(\boldsymbol y-\boldsymbol z_1),\\
\boldsymbol u_{\mathrm{raw}} &= \boldsymbol B_0^{-1}
\bigl[\boldsymbol\Omega_c(\boldsymbol y_{\mathrm{ref}}-\boldsymbol z_1)
-\boldsymbol z_2+\boldsymbol a_{\mathrm{ff}}\bigr].
\end{aligned}
```

実電流版では `u_ESO` に**最新受信実電流**を使用し、従来版では**前回指令電流**を使用する。LESOのオイラー更新、`wc`、`wo`、`b0`、`B0`、電流上限・変化率、過速度保護は同じ。ただし**通常停止処理はモード間で異なる**（以下参照）。

## 連続MIMO-LADRCによる停止（実電流入力版のみ）

`/cmd_vel=0` では上位のdiff_drive_controllerが車輪目標速度をゼロに変換する。このモードでは、減速ランプによる目標値の書き換えを行わず、`[v_ref, omega_ref]=[0,0]` を直接MIMO-LADRCへ入力する。

```math
\boldsymbol u_{\mathrm{raw}}=\boldsymbol B_0^{-1}
\left[-\boldsymbol\Omega_c\boldsymbol z_1-\boldsymbol z_2\right]
\quad(\boldsymbol y_{\mathrm{ref}}=\boldsymbol 0,\ \boldsymbol a_{\mathrm{ff}}=\boldsymbol 0).
```

- 停止開始時にも`mimo_ladrc_.reset()`と`send_zero_command()`による通常停止への切り替えは実施しない。ESOの`z1,z2`を継続更新する。
- 走行停止後も制御則は継続する。速度ゼロで自動的に0 Aへ固定する機構は**ない**。ESOの推定外乱・電流誤差によっては、ゼロ速度付近でも電流が流れ得る。
- 既存の電流制限（8 A/輪、20 A/s）、速度・電流フィードバック監視（30 ms）、2.0 m/sの過速度0 Aラッチは残る。ただし過速度0 Aは機械ブレーキではない。
- `mimo_ladrc_no_breakaway`（指令電流入力版）と他のモードでは、既存の通常停止減速ランプおよびZERO HOLDを**変更していない**。そのためESO入力方式だけの厳密な比較にはならず、停止時の応答の比較には制御方式の差も含まれる。

**重要：** これまでの1.0 m/s²の減速参照生成上限はこのモードに適用されない。停止時の減速度や姿勢変化は実機条件に依存し、従来以上に急制動・車体浮き上がり・逆転・近傍振動が生じ得る。独立した非常停止と人員を近づけない試験環境を用意し、実機の停止試験前に固定した車輪の低速動作から確認すること。

## 実電流の取得経路

- `read()`: TPDO1 `6077h:01` (左) / `6077h:02` (右) を符号付き16 bitで受信。単位は0.1 A/count。
- ROS前進正方向に一致させるため左輪を符号反転：`hw_efforts_[0]=-val_l*0.1`, `hw_efforts_[1]=val_r*0.1`。
- `write()`: 最新の `hw_efforts_` をそのまま`mimo_ladrc_.step()`のESO入力に使用。実電流の更新はTPDO1であり、200 Hz制御の各周期で新しい電流が届くとは限らない。その場合は最新の受信値を保持する。
- 現在のTPDO1イベントタイマ設定は10 ms、制御器設定は200 Hz（5 ms）。CANフレーム到着時刻と速度測定時刻は完全同期していない。追加の位相補正・フィルタ・ゲイン変更は行っていない。
- 実電流未受信時は0 Aを送信し、LESO更新を開始しない。実電流フィードバックの最終受信から**既存の`feedback_timeout_ms=30` ms**を超えた場合も0 Aを送信しエラーを返す。新しいパラメータは追加していない。
- 指令電流の変化率制限は、**従来どおり前周期の指令電流**を基準にする（実電流を変化率制限の基準に置き換えてはいない）。

## 未変更のパラメータ

`ladrc_b0_left/right=18`、`mimo_wc_v/mimo_wc_omega=5` rad/s、`mimo_wo_v/mimo_wo_omega=20` rad/s、車輪半径0.064 m、トレッド0.354 m、制御周期200 Hz、指令電流上限8 A/輪、電流変化率20 A/s、速度フィードバックタイムアウト30 ms。URDF、bringup、CANopen PDO設定は変更していない。

## チェックアウト・オフライン試験

ロボットの制御プロセスを停止した状態で以下を実行する。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch experiment/mimo-ladrc-no-breakaway-actual-current-eso
git pull --ff-only origin experiment/mimo-ladrc-no-breakaway-actual-current-eso

# ROS/CAN不要の単体テスト
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include \
  tests/test_no_breakaway_ladrc.cpp -o /tmp/test_no_breakaway_ladrc
/tmp/test_no_breakaway_ladrc

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

## 起動モードとログ（安全確認後のみ）

```bash
# ESO：前回指令電流（比較基準）
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway ros2 launch zlac_can_bringup zlac_can_bringup.launch.py

# ESO：CANopen TPDO1実電流（今回の実験）
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway_actual_current ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

起動時のログ文字列`MIMO_LADRC_ZERO_REFERENCE_CLOSED_LOOP_20261125_R1`および`MIMO_LADRC_ESO_TPDO1_ACTUAL_CURRENT_20261125_R1`を確認する。比較用ログは以下に分離する。

- 指令電流版：`/tmp/mimo_ladrc_no_breakaway_200hz.csv`
- 実電流版：`/tmp/mimo_ladrc_no_breakaway_actual_current_200hz.csv`

実電流版CSVには`eso_input_current_left_A`および`eso_input_current_right_A`列を追加する（既存の指令電流・実電流・ESO状態の列も維持）。

## 実験上の注意

1. 実電流はドライバの計測値（0.1 A刻み）。測定遅延・量子化・ノイズの影響がESOへ直接入るので、総外乱推定値の改善を保証しない。
2. 最初はロボットを適切に固定して車輪を浮かせ、非常停止の動作確認と低速の無負荷応答だけを評価する。コマンド発生源が複数ないか確認する。
3. 実電流フィードバック途絶では0 Aへ移行するが、機械的な非常停止ではない。過速度の2 m/sラッチ、30 msの速度フィードバック監視等は従来どおり。
4. 片輪拘束時の連動停止はこのBREAKAWAYなし実験では従来同様に無効。**手で車輪を止めないこと。**
5. この変更は未実機検証。比較する場合は同じ`/cmd_vel`プロファイル・路面・負荷・実験回数でRMSE、電流追従、`z2`を比較する。特に停止時は制御方式が異なるため、ESO入力の効果だけとは解釈しない。

## 変更したソース

- `src/zlac8015d_system.cpp`: 追加モード選択、実電流TPDO1受信時刻、フィードバック監視、LESO入力切替、ゼロ目標での連続LADRC、別CSV。
- `include/zlac8015d_hardware/zlac8015d_system.hpp`: 実験モードと電流受信時刻の状態追加。
- `include/zlac8015d_hardware/mimo_control.hpp`: 入力変数名とコメントを中立化（演算式は不変）。
- `tests/test_no_breakaway_ladrc.cpp`: 指令電流と実電流が異なるとLESOの予測加速度が変化すること、ゼロ速度目標でLADRCが制動電流を計算できることを検証。
