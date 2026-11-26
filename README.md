# 実験版：MIMO-LADRC / MIMO-PIDの発進から停止まで連続制御

## 追加：比較用 MIMO-PID（BREAKAWAYなし・停止まで一貫制御）

`ZLAC_CONTROL_MODE=mimo_pid_no_breakaway` を指定すると、既存の機体座標MIMO-PIDで発進・巡行・旋回・ゼロ速度への停止を一貫して制御します。停止用減速ランプ、停止用加速度フィードフォワード、ZERO HOLDを通さず、PIDの積分状態・微分フィルタ状態を保ったまま`/cmd_vel=0`にも追従します。

比較対象の`mimo_ladrc_no_breakaway_actual_current`も同じゼロ速度目標への連続制御です。両者は電流指令の上限8 A/輪、変化率20 A/s、速度・電流フィードバック監視30 ms、車輪周速度2.0 m/sで0 Aラッチなどを共有し、既存のゲインとパラメータは変更しません。

MIMO-PIDのゲインは`Kp_v=Kp_omega=0.7`、`Ki_v=Ki_omega=0.5`、`Kd_v=Kd_omega=0.03`、微分フィルタ`tau=0.05s`（URDFで定義した既定値）。**停止時の従来1.0 m/s²の減速参照制限がなくなるため、独立した非常停止を準備し、車輪浮上・低速で安全を検証してください。**

比較条件・ビルド・オフラインテスト・計測CSVについては [MIMO_PID_NO_BREAKAWAY_COMPARISON.md](MIMO_PID_NO_BREAKAWAY_COMPARISON.md) を参照してください。

---

## 追加：ESOに実電流を入力する比較実験

このブランチ `experiment/mimo-ladrc-no-breakaway-actual-current-eso` は、BREAKAWAYなしMIMO-LADRCで**ESOの入力電流だけ**を切り替えられる比較実験用です。`main` は未変更です。

- `ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway`：従来方式（前周期の指令電流をESOへ入力）
- `ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway_actual_current`：新方式（CANopen TPDO1の実電流をESOへ入力）

実電流入力版は**発進・走行・旋回・通常停止・ゼロ速度目標への保持まで、MIMO-LADRCを連続実行**します。通常停止専用の減速ランプ・停止用フィードフォワード・ZERO HOLDはこのモードでバイパスします。`/cmd_vel=0`でもESOをリセットせず、ゼロ速度目標に対する電流を計算します。従来の指令電流入力版とその他のモードの停止処理はそのまま残します。

ゲイン、`b0`、電流上限8 A/輪、変化率20 A/s、制御周期200 Hz、2.0 m/s過速度停止は変更しません。新方式は実電流未受信時に0 Aを出力し、フィードバックが既存の30 msタイムアウトを超えた場合も0 Aにしてエラーを返します。

**注意：** 実電流版では停止時の目標減速制限（従来の1.0 m/s²）が適用されません。急制動・逆転・車体浮き上がりの可能性があるため、非常停止を準備して低速・車輪浮上状態から評価してください。

詳細・実験前チェック・オフライン試験・ログは [ACTUAL_CURRENT_ESO_EXPERIMENT.md](ACTUAL_CURRENT_ESO_EXPERIMENT.md) を参照してください。新方式のログは `/tmp/mimo_ladrc_no_breakaway_actual_current_200hz.csv` です。

---

このブランチ `experiment/mimo-ladrc-no-breakaway` は検証用です。採用済みの `release/mimo-ladrc-stable` は変更していません。

起動時に `ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway` を指定すると、静止状態からMIMO-LADRCを使用し、始動用BREAKAWAY制御を通しません。
実験モードの通常電流指令上限は8 A/輪、変化率制限は20 A/sです。片輪拘束による両輪連動の制動・低速高電流による連動停止は削除しました。**いずれかの車輪周速度が2.0 m/s（r=0.064 mなら31.25 rad/s、約298.4 rpm）以上になった場合のみ、両輪へ0 Aを指令し、制御プログラム再起動まで停止状態を保持します。** 0 Aは非常停止や機械ブレーキではありません。**手でタイヤを止める試験は行わないでください。**

**MIMO-LADRCの制御コードを学習する場合：** [MIMO_LADRC_NO_BREAKAWAY_STUDY_GUIDE.md](MIMO_LADRC_NO_BREAKAWAY_STUDY_GUIDE.md) に、ESO・制御則・C++関数の対応と、静止状態からの演算順をまとめています。

検証方法、過速度ラッチ、200Hz CSV、安定版への復帰手順は [NO_BREAKAWAY_EXPERIMENT.md](NO_BREAKAWAY_EXPERIMENT.md) を参照してください。

---
# zlac8015d_hardware — MIMO-LADRC 完成版

ROS 2 Humble・ZLAC8015D CANopen hardware_interface。完成版ブランチ: `release/mimo-ladrc-stable`。

## 実機制御

実機試験で急加速の改善を確認したMIMO-LADRCをそのまま採用。制御ソースとゲイン・制限値の計算は検証ブランチから変更していません。
状態遷移の実装は、MIMOモードで車輪が目標と逆方向に回転していても、一定以上の速度を連続サンプルで検出すればTRACKに移行できる方式です。
ビルド識別子：`MIMO_TRANSITION_FIX_20261113_R2`。

## 残したファイル

- `src/zlac8015d_system.cpp` : CAN通信、電流制御、状態遷移、安全監視
- `include/zlac8015d_hardware/mimo_control.hpp` : MIMO-LADRC制御則
- `include/zlac8015d_hardware/zlac8015d_system.hpp` : hardware_interface宣言
- `zlac8015d_hardware.xml` : プラグイン登録
- `CMakeLists.txt`・`package.xml` : ビルド

## 起動

同じ `release/mimo-ladrc-stable` ブランチの `zlac8015d_can_bringup` と一緒に配置する。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
ZLAC_CONTROL_MODE=mimo_ladrc_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

機体は既存の操作コントローラが `/cmd_vel` に送信する速度指令で動く。CAN独占、非常停止、許容電流・速度を確認し、安全な条件で運転する。

試験・測定用Pythonスクリプト、C++テスト、解析プログラムは検証用 `feat/mimo-ladrc-pid-straight-tracking` ブランチに残しており、完成版からのみ除外しています。
