# MIMO-LADRCによる静止状態からの直接発進（BREAKAWAYなし）

## 研究目的

従来の制御は静止摩擦対策として、MIMO-LADRCと別のBREAKAWAY電流を起動時に指令していた。本実験はBREAKAWAYを廃止し、静止状態で非ゼロ速度指令を受けた最初の200 Hz制御周期から、MIMO-LADRCとESOを使用する。
実機で急加速改善を確認した安定版 `release/mimo-ladrc-stable` は**変更しない**。この実験はハードウェアリポジトリの別ブランチ `experiment/mimo-ladrc-no-breakaway` のみで有効。bringupには安定版 `release/mimo-ladrc-stable` を使う。

## モードと制御仕様

- **安定版** `ZLAC_CONTROL_MODE=mimo_ladrc_torque`：これまでどおりBREAKAWAY→TRACK状態遷移を利用。
- **実験版** `ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway`：BREAKAWAY、READY待ち、TRACK電流下限補正、方向反転時のBREAKAWAY再突入、ストールからBREAKAWAYへの戻りを**一切使用しない**。
- 実験モードのMIMO-LADRCとLESOは目標車輪角速度が1輪だけ非ゼロでも作動。制御開始時のESOはエンコーダの実測速度で初期化する。
- 通信フィードバック監視、20 A/sの電流変化率制限、通常停止の減速ランプとZERO HOLD、CAN送信検査は維持。
- 実験モードの電流上限は既定で**1.0 A/輪**。URDFに任意のhardware param `no_breakaway_current_limit_a` を加えれば変更可能だが、許可範囲は `(0,2.0] A` かつ `current_limit_a` 以下。まず既定値で試験する。
- いずれかの車輪が `stall_speed_rad_s` 未満で、その車輪自身の電流指令が上限の95%以上に2秒間張り付くと、再発進を試みず**電流0のラッチ状態**に入り、速度指令を明示的に0へ戻すまで解除しない。
- 既存の全体Imax=8 Aとは異なり実験上限が優先する。従来の安定モードでは新しい1 A制限は無効。
- 診断識別子 `MIMO_LADRC_NO_BREAKAWAY_EXP_20261115_R1`。内部CSVは `/tmp/mimo_ladrc_no_breakaway_200hz.csv` に記録。`tracking_left/right=1` はLADRC計算中（BREAKAWAY不使用）を表す。

## 理論上の注意

静止摩擦力は非線形・速度依存・履歴依存であり、摩擦力そのものを停止直後に直接観測するセンサはない。ESOは予測された速度変化と実速度の不一致から等価外乱を推定できるため、停止中に電流を指令し続けて動かない場合には逆向きの外乱推定が蓄積することがある。一方、同定係数b0の不一致、電流飽和、通信遅延、時間変動する静止摩擦閾値によっては、LADRCだけでは発進できない場合がある。最初から静止摩擦を完全に推定できるわけではない。

## 実行コマンド（機体停止・車輪浮上・非常停止を確認）

端末1・事前作業：

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
strings "$LIB" | grep -F MIMO_LADRC_NO_BREAKAWAY_EXP_20261115_R1
```

端末1・起動：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway \
  ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

端末2・確認：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
```

両コントローラのactive状態と起動ログの識別子を確認後、**車輪を浮かせた状態**で既存の操作コントローラから短時間の低速直進・旋回を試す。静止摩擦を評価する地上試験は、その後に安全が確認できた場合のみ行う。`/cmd_vel` は他の発行ノードとの競合を避ける。

試験後は外部コントローラで0指令を送って車輪停止を確認し、端末1でCtrl+Cする。CSVを保存する：

```bash
cp /tmp/mimo_ladrc_no_breakaway_200hz.csv \
  "$HOME/ros2_ws/no_breakaway_$(date -u +%Y%m%dT%H%M%SZ).csv"
```

## 比較する主な指標

1. 速度指令から車輪の始動までの遅れ（左右それぞれ）。
2. 速度が0の間のMIMO-LADRC電流指令・実電流・ESO外乱推定値。
3. 始動可否、最大電流、電流飽和時間と2秒ストール保護の発生有無。
4. 立ち上がり時間、オーバーシュート、定常追従誤差、直進時の角速度偏差。
5. 安定版のBREAKAWAYありと**同一指令・同一地面・同一負荷**で比較。設定した1 Aの制限が違う場合は、制御方式以外の条件差であることを明記する。

## 元の安定版に戻す

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
git switch release/mimo-ladrc-stable
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware --symlink-install
source install/setup.bash
ZLAC_CONTROL_MODE=mimo_ladrc_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

制御プロセスが動いている間のブランチ切替・ビルドは行わない。
