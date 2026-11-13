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
