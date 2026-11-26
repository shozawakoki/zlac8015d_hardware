# MIMO-PID（BREAKAWAYなし・停止まで連続制御）比較実験

対象: `zlac8015d_hardware` / `experiment/mimo-ladrc-no-breakaway-actual-current-eso`

## 目的・制御モード

今回の`mimo_pid_no_breakaway`は、実電流入力MIMO-LADRCと比較するための機体座標系MIMO-PIDである。**走り出し、巡行、旋回、減速、`/cmd_vel=0`後の速度ゼロ保持までPIDを継続実行**する。旧BREAKAWAY、TRACK切り替え、停止用減速ランプ、停止加速度フィードフォワード、ZERO HOLDは使わない。既存モードは変更しない。

| 起動環境変数 `ZLAC_CONTROL_MODE` | 制御器 | `/cmd_vel=0`時 | 出力CSV |
| --- | --- | --- | --- |
| `mimo_ladrc_no_breakaway_actual_current` | MIMO-LADRC（LESOにTPDO1実電流） | LADRCでゼロ速度追従 | `/tmp/mimo_ladrc_no_breakaway_actual_current_200hz.csv` |
| `mimo_pid_no_breakaway` | MIMO-PID（LESOなし） | PIDでゼロ速度追従 | `/tmp/mimo_pid_no_breakaway_200hz.csv` |
| `mimo_ladrc_no_breakaway` | MIMO-LADRC（LESOに前周期指令電流） | **旧**減速ランプ・ZERO HOLD | `/tmp/mimo_ladrc_no_breakaway_200hz.csv` |
| `mimo_pid_torque` | **旧**MIMO-PID | **旧**状態遷移・減速処理 | `/tmp/mimo_pid_track_200hz.csv` |

## MIMO-PID制御則と実装

入力は目標機体速度`y_ref=[v_ref,omega_ref]^T`、フィードバックはエンコーダから変換した`y=[v,omega]^T`、出力は左右指令電流`[I_L,I_R]^T`。

```math
\begin{aligned}
\boldsymbol e &= \boldsymbol y_{\mathrm{ref}}-\boldsymbol y,\\
\boldsymbol a_{\mathrm{PID}} &= b_{\mathrm{nom}}
\left[\boldsymbol K_P\boldsymbol e
+\boldsymbol K_I\int\boldsymbol e\,dt
-\boldsymbol K_D\dot{\boldsymbol y}_{\mathrm{filtered}}\right],\\
\boldsymbol I_{\mathrm{raw}} &= \boldsymbol B_0^{-1}\boldsymbol a_{\mathrm{PID}}.
\end{aligned}
```

`B_0=T diag(b_left,b_right)`, `b_nom=(b_left+b_right)/2`。車輪順序`[L,R]`、機体順序`[v,omega]`。`mimo::Pid::step()`を使用し、新しいPID制御アルゴリズムは追加していない。P/Iは制御周期`dt`で更新、Dは新しいTPDO0速度フレームが到着したとき、その実測到着時間差で更新（`tau=0.05s`の一次微分フィルタ）。機体速度への微分作用は負符号なので、目標速度ステップの微分キックを回避する。

既存クラスの**条件付きアンチワインドアップ**は、電流の目標計算値がいずれか一輪で8Aを超える場合に積分更新を停止する。ただし電流の変化率制限による一時的な飽和は内部PIDの積分抑制条件に含まれていないため、実機のオーバーシュートには注意する。

### 既存パラメータ（変更なし）

- `mimo_pid_kp_v = mimo_pid_kp_omega = 0.7`
- `mimo_pid_ki_v = mimo_pid_ki_omega = 0.5`
- `mimo_pid_kd_v = mimo_pid_kd_omega = 0.03`
- `pid_derivative_filter_tau_s=0.05` s
- `ladrc_b0_left=ladrc_b0_right=18` (PID変換行列に使用)
- 車輪半径0.064 m、トレッド0.354 m
- controller_manager 200 Hz、各輪指令電流上限**±8 A**、変化率**20 A/s**
- 速度・電流TPDO監視**30 ms**、車輪周速度**2.0 m/s**以上で0 Aラッチ

MIMO-LADRCも同じアクチュエータ制限、速度指令、エンコーダフィードバック、通常停止時のゼロ目標を使用する。**ただしPIDゲインの数値とLADRC帯域設定には直接的な等価性はない**。比較には両制御器の調整条件とその根拠を別途記録すること。

### 停止動作と残る安全保護

`/cmd_vel=0`で`hw_commands_=[0,0]`になっても、PIDをリセットせず、P/I/Dで左右電流を計算し続ける。制御器の積分状態が残るので、**停止後にもゼロでない電流が流れる場合がある**。0 A固定（ZERO HOLD）は用いない。

一方、安全監視や故障時の0 A指令は必ず優先する。具体的には速度・実電流未受信/タイムアウト、左右いずれかの車輪の周速度2.0 m/s以上、非有限な制御演算値、CAN送信エラーなど。過速度ラッチはコントローラ再起動まで解除されない。0 A指令は機械ブレーキではなく非常停止ではない。片輪拘束時の連動停止もないので、手でタイヤを止めないこと。

**危険性:** 停止時の従来減速指令上限1.0 m/s²は適用されない。転倒・車体浮き上がり・逆転・振動を想定して、車輪を浮かせて防護・非常停止確認後の低速検証から始める。

## 変更したファイル

- `src/zlac8015d_system.cpp`: 制御モード追加、既存`mimo::Pid`の直結分岐、ゼロ速度時継続PID、MIMO-LADRCと同じ電流送信・保護、専用CSV、起動識別子。
- `include/zlac8015d_hardware/zlac8015d_system.hpp`: モードフラグ追加。
- `tests/test_no_breakaway_pid.cpp`: 発進、停止、旋回、速度サンプル微分、既存アンチワインドアップのオフライン検証。
- `CMakeLists.txt`: テスト登録。

PID用CSVには`ref_v_m_s`,`ref_omega_rad_s`,`v_m_s`,`omega_rad_s`,`cmd_current_left_A`,`cmd_current_right_A`,`actual_current_left_A`,`actual_current_right_A`,`mimo_pid_integral_v`,`mimo_pid_integral_omega`,`mimo_pid_d_v_m_s2`,`mimo_pid_d_omega_rad_s2`,`mimo_pid_d_updated`,`mimo_pid_d_dt_s`が含まれる。PIDログの`hat_d_v_m_s2`と`hat_d_omega_rad_s2`はESO推定値ではない（常に0として出力）。**PIDにESOはない。**

## チェックアウト・オフラインテスト・ビルド

先に走行指令を止め、制御プロセスを停止し、他のプロセスがCANを占有していないことを確認する。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch experiment/mimo-ladrc-no-breakaway-actual-current-eso
git pull --ff-only origin experiment/mimo-ladrc-no-breakaway-actual-current-eso

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include \
  tests/test_no_breakaway_pid.cpp -o /tmp/test_no_breakaway_pid
/tmp/test_no_breakaway_pid

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include \
  tests/test_no_breakaway_ladrc.cpp -o /tmp/test_no_breakaway_ladrc
/tmp/test_no_breakaway_ladrc

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

## 起動（試験環境の安全確認後のみ）

```bash
# MIMO-PID 発進から停止まで PID 連続制御
ZLAC_CONTROL_MODE=mimo_pid_no_breakaway \
  ros2 launch zlac_can_bringup zlac_can_bringup.launch.py

# 比較用 MIMO-LADRC 実電流入力（別起動）
ZLAC_CONTROL_MODE=mimo_ladrc_no_breakaway_actual_current \
  ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

起動ログで`Configured mode=mimo_pid_no_breakaway`および`MIMO_PID_NO_BREAKAWAY_ZERO_REFERENCE_20261126_R1`を確認する。起動モードの切り替えは必ずプロセス停止・再起動により行い、同時起動を避ける。

## 比較評価

同一の`/cmd_vel`指令、同一条件で、直進・その場旋回・両軸同時指令・停止の応答を収集する。特に停止時には速度ゼロへの整定時間、逆転ピーク、停止後の電流RMS、移動距離・角度、振動を比較する。速度追従誤差は同じ評価時間区間・測定源でRMSE/IAEを算出する。制御モードを変える前に試行ごとにCSVを別名で保存すること。

**現在はGitHub上のコード変更のみ。ROS2ビルド・実機検証結果ではない。**
