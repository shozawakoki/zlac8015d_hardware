# 実験版：静止状態からMIMO-LADRCで直接発進（BREAKAWAYなし）

このブランチ `experiment/mimo-ladrc-no-breakaway` は検証用です。採用済みの `release/mimo-ladrc-stable` は変更していません。

起動時に `ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway` を指定すると、静止状態からMIMO-LADRCを使用し、始動用BREAKAWAY制御を通しません。
安全のため実験モードのみ既定1 Aの電流上限（最大設定2 A）、20 A/sの変化率制限、2秒の飽和ストールラッチを備えています。

検証方法、状態の切り分け、200Hz CSV、安定版への復帰手順は [NO_BREAKAWAY_EXPERIMENT.md](NO_BREAKAWAY_EXPERIMENT.md) を参照してください。

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
