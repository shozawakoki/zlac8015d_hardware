# BREAKAWAYなしMIMO-LADRC：8 A指令と車輪周速度2.0 m/sの過速度遮断（検証版）

## バージョンと用途

- 検証用リポジトリ／ブランチ：`zlac8015d_hardware` / `experiment/mimo-ladrc-no-breakaway`
- 採用済みの安定版：`release/mimo-ladrc-stable` は変更しない
- ROS 2 Humble / Ubuntu 22.04 / ZLAC8015D CANopen、制御周期200 Hz
- 起動モード：`ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway`
- 起動識別子：`MIMO_LADRC_NO_BREAKAWAY_EXP_20261115_R5`

## 変更したモータ制御

発進直後からMIMO-LADRC／LESOを適用する。BREAKAWAY電流、段階的な始動電流増加、TRACK電流下限、方向反転時のBREAKAWAY再突入は使用しない。並進速度vと角速度omegaをエンコーダから算出してフィードバックする。

- 通常のMIMO-LADRC電流指令の最大値：**±8 A/輪**（`no_breakaway_current_limit_a`、許容設定範囲 `(0,8] A`、全体電流上限を超えない）
- 指令電流の変化率：**20 A/s**（既存パラメータ）
- `ladrc_b0_left/right=18`、`mimo_wc_v=5`、`mimo_wc_omega=5`、`mimo_wo_v=20`、`mimo_wo_omega=20` は現状のまま
- 通信速度フィードバックタイムアウト・CANopen送信エラー処理・通常の停止指令とZERO HOLDは維持

### 廃止した拘束保護

以下はいずれも**実験モードから削除済み**。

1. 並進速度偏差・角速度偏差・片輪拘束を同時に検出して、他輪へ最大1 Aの制動電流を指令する連動停止
2. 低速かつ高電流（2 A以上0.4秒）による左右電流0 Aラッチ

したがって片輪拘束時には、LADRCが拘束輪への電流を最大8 Aまで増加させる場合がある。拘束輪・駆動系の発熱、事故防止のため、**手で車輪を止める試験はしない**。別の独立したハードウェア非常停止・電流／温度保護が必要。

### 新たな過速度停止

ホイールの実測角速度をROS正方向の符号に変換した後、**受信したTPDO0の各フレームごと**に判定する。車輪半径r=0.064 mで、

```text
wheel_limit_m_s = 2.0 m/s
wheel_limit_rad_s = 2.0 / 0.064 = 31.25 rad/s
wheel_limit_rpm = wheel_limit_rad_s * 60 / (2*pi) = 約298.4 rpm
trigger = abs(omega_left) >= wheel_limit_rad_s OR
          abs(omega_right) >= wheel_limit_rad_s
```

正転／逆転のどちらにも適用し、左右いずれか一輪でも閾値に達すれば**左右両輪の電流指令を0 A**にする。閾値は機体重心の並進速度ではなく車輪周速度であるため、その場旋回でも検出できる。

停止状態 `no_breakaway_overspeed_latched_` をラッチし、速度低下や `/cmd_vel=0` では解除しない。**制御プロセスを停止し、安全な状態と原因を確認した後のハードウェア再起動（on_activate）でのみ解除**される。

過速度検出は通常のMIMO-LADRCの指令計算に優先する。速度の新しいCANパケットが来なくても、既存の速度フィードバック監視は0 Aを指令する。0 Aは駆動電流を指令しないだけであり、機械的なブレーキ・CiA402 Quick Stop・安全認証された緊急停止ではない。惰性回転や運動エネルギーは残る。

検証中のログには `MIMO_WHEEL_OVERSPEED_STOP` が一度だけ記録される。実験モードの `/tmp/mimo_ladrc_no_breakaway_200hz.csv` に `wheel_overspeed_latched` 列を追加した。旧 `coordinated_stall_stop` などの列は削除した。

**適用範囲：** この過速度0 Aラッチは `mimo_ladrc_no_breakaway` のみに適用。一般の `mimo_ladrc_torque` や `velocity` モード、および別ブランチの安定版には影響しない。

## テストとビルド（モータを起動しない）

制御ノードとCANの別オーナーを停止した状態で実行する。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch experiment/mimo-ladrc-no-breakaway
git pull --ff-only origin experiment/mimo-ladrc-no-breakaway

g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude \
  tests/test_no_breakaway_ladrc.cpp -o /tmp/test_no_breakaway_ladrc
/tmp/test_no_breakaway_ladrc

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
LIB="$(ros2 pkg prefix zlac8015d_hardware)/lib/libzlac8015d_hardware.so"
strings "$LIB" | grep -F MIMO_LADRC_NO_BREAKAWAY_EXP_20261115_R5
```

## 起動（上記に問題がない場合のみ）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway \
  ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

最初は車輪を浮かせて保護装置・防護柵を使用し、低速の走行と正常停止だけを確認する。**過速度閾値まで意図的に加速する実機テストは行わない。** 閾値動作はオフラインC++テストで確認し、実機ではログと通常動作を確認する。

CSV保存例（停止後にlaunchを終了してから）：

```bash
cp /tmp/mimo_ladrc_no_breakaway_200hz.csv \
 "$HOME/ros2_ws/no_breakaway_$(date -u +%Y%m%dT%H%M%SZ).csv"
```

## 研究上の注意：等ゲインのMIMO-LADRC

`y=T q`（`q=[omega_L,omega_R]`）で、`B=T diag(b_L,b_R)` としたとき、
`Omega_c = wc*I`、`Omega_o = wo*I` なら、座標変換後のLADRCフィードバックとESOは車輪座標で独立な2つの制御則へ変換できる。
左右の外乱が異なる場合でも、各車輪のESOは別々の外乱を補償できるため、必ず直進できないわけではない。ただし推定誤差、飽和、摩擦差による車輪速度偏差から機体ヨーレート誤差が生じる。角速度帯域を並進速度と異なる値にすると左右輪へのフィードバックが結合するが、地上走行の直進精度は別途評価が必要。

## 安定版への復帰

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
git switch release/mimo-ladrc-stable
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware --symlink-install
source install/setup.bash
```

実機でのコントローラ・CAN制御中にはブランチ切替やビルドを行わない。
