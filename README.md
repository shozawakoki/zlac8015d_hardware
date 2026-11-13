## TRACK移行・再始動判定の修正（fix/track-handoff-stall-gate）

このブランチは `devel/startup-stall-rework` から分岐しています。
`main`、`devel/pid`、`devel/startup-stall-rework` は変更していません。

- **TRACKへの移行**: 始動時の大きな電流下限をTRACK区間まで
  引き継がず、0.20 Aの通常下限に戻します。電流指令の変化は従来の
  `current_rate_limit_a_per_s` によるレート制限で行います。
- **失速判定**: 正方向に低速であること、TRACK移行後の150 msが
  経過していること、一定期間駆動電流を要求していることに加え、
  駆動要求と実際の指令が少なくとも
  `max(stall_min_drive_current_a, breakaway_start_a)` [A]
  以上であることを必要とします。従来の `0.20 A` 指令では
  再始動しません。
- **繰り返し再始動の抑制**: 非ゼロ速度の指令期間中に自動再始動は
  1回までとし、再び失速が判定された場合は左右輪の電流指令を
  ゼロに固定します。`stall_lockout_left/right` が内部CSVに記録されます。
  このラッチはゼロ速度指令を受けて解除されます。
- **安全監視は維持**: CAN通信、速度フィードバック、電流上限などの
  既存監視は削除しません。追加の起動可否チェックやテスト用ノードは
  増やしていません。

この変更により **P制御だけで安定な速度追従を保証するものではありません**。
例えばKp=0.2、目標速度0.10 m/s（車輪目標1.5625 rad/s）では、
停止近傍の比例指令は約0.313 Aです。静止摩擦を突破できない場合、
不要な再始動が抑えられても、速度が低いまま残る可能性があります。
PI/LADRCの本来の追従性能は、始動補償と区別して評価してください。

### 最小限の確認方法

bringup側は変更不要です。すでに用意した
`zlac_can_bringup/devel/staircase-evaluation` と
`~/pid_startup_test.yaml` をそのまま利用できます。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch fix/track-handoff-stall-gate
git pull --ff-only

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware --symlink-install
source install/setup.bash

# 1条件の設定を確認。モータは動かない。
ros2 run zlac_can_bringup run_gain_sweep.py --config ~/pid_startup_test.yaml
```

初回実機試験は車輪を浮かせ、独立した非常停止装置と通信断時の
停止機能を確認したうえで**PIDの1条件だけ**実行してください。
`--execute --i-have-e-stop` は実機の速度指令を送信します。
ソフトウェアで指令電流をゼロにしても、惰性やCAN通信障害時の停止は
保証されません。

確認する値は、①最大車輪角速度、②速度指令中の
`TRACK→BREAKAWAY` 回数、③ `stall_lockout_left/right`、
④試験が安全に停止したか、の4点のみです。

ビルド・実機試験はまだ未検証です。

---

## 始動・失速復帰の再設計（devel/startup-stall-rework）

この作業ブランチは `devel/pid` から分岐し、PID/LADRC 共通の
BREAKAWAY と TRACK への遷移条件を変更します。元の `devel/pid`
および `main` は変更していません。

- 始動電流の増分は既定で **0.10 A / 20制御サンプル**
  （公称200 Hzなら0.10秒ごと、以前の0.50 A/10サンプルより緩やか）。
- 既定の始動電流上限を **2.0 A** とし、`current_limit_a` より高い
  BREAKAWAY設定を拒否します。接地条件では不足する場合があります。
  **確認できないまま上限を増加させないでください。**
- 車輪が始動速度に達したら電流増加を止め、6サンプル以上連続して
  始動速度を確認するとTRACKへ移行します。
- TRACK中の失速復帰は、0.10 rad/s未満が80サンプル継続するだけでなく、
  **制御器が正方向の駆動電流を要求し、最終指令も同方向であること**
  を必要とします。減速・制動電流による一時停止ではBREAKAWAYへ戻しません。
- 超信地旋回の協調BREAKAWAY待機電流も設定された始動電流上限に制限します。

設定値（すべてROS2ハードウェアのURDF `<hardware><param>` から読み取り）：

| パラメータ | 初期値 | 説明 |
| --- | ---: | --- |
| `breakaway_start_a` | 0.65 | 始動指令電流 [A] |
| `breakaway_step_a` | 0.10 | 段階的増分 [A] |
| `breakaway_max_a` | 2.0 | 始動中の上限 [A] |
| `breakaway_step_samples` | 20 | 増分間の制御サンプル数 |
| `breakaway_start_speed_rad_s` | 0.30 | TRACK移行速度の閾値 |
| `breakaway_start_samples` | 6 | TRACK移行に必要な連続サンプル数 |
| `stall_speed_rad_s` | 0.10 | 失速速度の閾値 |
| `stall_samples` | 80 | 失速継続サンプル数 |
| `stall_min_drive_current_a` | 0.20 | 再始動を許す最小駆動電流 [A] |

各値は**机上の初期設定**であり、車輪を浮かせた実機試験・接地試験で
安定性を検証した値ではありません。サンプル数に基づく時間は、実際の
制御周期によって変わります。

対になる bringup 作業ブランチは
[devel/staircase-evaluation](https://github.com/shozawakoki/zlac8015d_can_bringup/tree/devel/staircase-evaluation)
です。通常の `velocity` モードは本変更のBREAKAWAY制御を利用しません。

ゲイン調整は車輪を浮かせた状態で完結させず、接地状態の負荷を考慮した
同定結果を基に再設定してください。過去に速度の大きな行き過ぎと
左右輪の非同期な加速が観察されているため、**新実装でも地面に置いた
一括ゲイン試験をすぐに再開しないでください。** まず車輪を浮かせ、
低速で1条件の始動・停止・速度上限を検証してください。

# zlac8015d_hardware

ZLAC8015D モータドライバを、Linux SocketCAN / CANopen 経由で ROS 2 の `ros2_control` から操作するための **C++ ハードウェアインターフェース**です。対向二輪型移動ロボットの左右駆動輪に対して、速度指令とエンコーダ位置・実測角速度・モータ電流の取得を行います。

**3種類の制御モード**を実装しています。`devel/pid` ブランチでは、既存のLADRCとモータドライバ内蔵速度制御を維持し、左右車輪の独立したPID電流制御を追加しています。

| モード | 内容 | ドライバへの指令 |
| --- | --- | --- |
| `velocity` | ZLAC8015D 内蔵の速度制御を使用 | 車輪角速度 |
| `ladrc_torque` | 左右車輪の一次 LADRC（線形拡張状態オブザーバ：LESO）で速度追従制御 | モータ電流 |
| `pid_torque` | 左右車輪の PID による速度追従制御（電流指令） | モータ電流 |

ROS 2 側の**コマンドインターフェースは、3モードとも車輪角速度 `velocity` [rad/s]** です。`ladrc_torque` と `pid_torque` では、プラグイン内部で電流指令に変換します。`velocity` はドライバの内蔵速度制御を使用します。

この README は `main` の `src/zlac8015d_system.cpp`、ヘッダー、ビルド設定および [`zlac8015d_can_bringup`](https://github.com/shozawakoki/zlac8015d_can_bringup) の現行 URDF を基準にしています。**本実装に含まれない同定用プログラムやスクリプトの実行手順は掲載していません。**

## 動作環境

- Ubuntu 22.04 / ROS 2 Humble
- Linux SocketCAN、ZLAC8015D モータドライバ
- C++17 / `ament_cmake`
- `rclcpp`、`hardware_interface`、`pluginlib`
- 起動・差動二輪コントローラの設定に [`zlac8015d_can_bringup`](https://github.com/shozawakoki/zlac8015d_can_bringup) を使用

```bash
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-robot-state-publisher python3-colcon-common-extensions can-utils
```

## 現在のファイル構成

```text
zlac8015d_hardware/
├── CMakeLists.txt
├── package.xml
├── README.md
├── zlac8015d_hardware.xml
├── include/
│   └── zlac8015d_hardware/
│       ├── zlac8015d_system.hpp
│       └── zlac8015d_system_before_breakaway.hpp
└── src/
    └── zlac8015d_system.cpp
```

| ファイル | 役割 |
| --- | --- |
| `src/zlac8015d_system.cpp` | SocketCAN、CANopen 通信、`ros2_control`、LADRC、起動・停止処理 |
| `include/zlac8015d_hardware/zlac8015d_system.hpp` | 現行ハードウェアクラスの宣言 |
| `include/zlac8015d_hardware/zlac8015d_system_before_breakaway.hpp` | 旧構造のヘッダー。現行 `.cpp` がインクルードするファイルではない |
| `zlac8015d_hardware.xml` | `pluginlib` の登録情報 |

`CMakeLists.txt` でライブラリ `zlac8015d_hardware` を生成します。**この `main` ブランチには独立したモデル同定実行ファイル、Python 解析スクリプト、`scripts/` ディレクトリはありません。** `ros2 run zlac8015d_hardware first_order_identification` 等は、このリポジトリの現在のビルドでは利用できません。

## インストールとビルド

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/shozawakoki/zlac8015d_hardware.git
git clone https://github.com/shozawakoki/zlac8015d_can_bringup.git
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

クローン済みの場合は重複して取得せず、既存リポジトリの `main` を更新してからビルドしてください。プラグイン名は次のとおりです。

```xml
<plugin>zlac8015d_hardware/Zlac8015dSystemHardware</plugin>
```

## CAN の確認と起動

```bash
ip -details link show can0
candump can0
```

`candump` は `Ctrl+C` で終了します。CAN ビットレートと機器の接続条件は環境依存のため、ここでは固定値を指定しません。本パッケージは CAN インターフェースの作成や故障した物理バスの復旧を行いません。

bringup パッケージから起動します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

別ターミナルで以下を確認します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /odom
```

最初の試験では車輪を浮かせ、非常停止手段を確保してください。`ros2_control` と同じ CANopen ノードを操作する別プログラムを同時に起動しないでください。

## 制御モードの切替（devel/pid）

最初に本リポジトリの `devel/pid` ブランチをビルドしてください。bringup 側は既存の `main` をそのまま使用できます。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_hardware
git fetch origin
git switch devel/pid
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware zlac_can_bringup --symlink-install
source install/setup.bash
```

### 起動時の切替（推奨）

モードを環境変数 `ZLAC_CONTROL_MODE` で指定します。これは URDF の `control_mode` を一時的に上書きします。新しいターミナルで、次のうち **1つだけ** を実行してください。

```bash
# PID 電流制御（Kp=0.7, Ki=0.5, Kd=0.03）
ZLAC_CONTROL_MODE=pid_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py

# 従来の LADRC 電流制御
ZLAC_CONTROL_MODE=ladrc_torque ros2 launch zlac_can_bringup zlac_can_bringup.launch.py

# モータドライバ内蔵の速度制御
ZLAC_CONTROL_MODE=velocity ros2 launch zlac_can_bringup zlac_can_bringup.launch.py
```

`ZLAC_CONTROL_MODE` を省略すると、bringup の URDF に設定されている `control_mode` を使用します（現行 bringup では `ladrc_torque`）。モード変更は**起動・再起動時のみ**で、運転中の切替には対応しません。必ず既存の launch を停止してから別モードで再起動してください。

### URDF を書き換えて切替

`zlac8015d_can_bringup/urdf/orange_go_2025.urdf` 内で次のように設定することもできます。

```xml
<param name="control_mode">pid_torque</param>
<param name="pid_kp">0.7</param>
<param name="pid_ki">0.5</param>
<param name="pid_kd">0.03</param>
<param name="pid_derivative_filter_tau_s">0.05</param>
```

`pid_kp`、`pid_ki`、`pid_kd`、`pid_derivative_filter_tau_s` は **ハードウェア側に上記の既定値があります**。URDF に記載しなくてもこの値が適用されます。URDF を変更した場合は bringup を再ビルドしてから再起動してください。環境変数が設定されている場合は環境変数のモードが優先されます。

### PID の制御則

各輪の目標角速度を `r`、実測角速度を `omega` とすると、速度偏差は `e = r - omega` です。TRACK 状態のPID制御は以下を用います。

```text
I_cmd = Kp * e + integral(Ki * e dt) - Kd * Df(omega)
```

`Df(omega)` は実測角速度の時間微分を時定数 `0.05 s` の一次ローパスで平滑化したものです。目標値の急変で微分キックが生じるのを避けるため、微分項は偏差ではなく実測値から計算します。

| PID パラメータ | 初期値 | 意味 |
| --- | ---: | --- |
| `pid_kp` | `0.7` | 比例ゲイン [A/(rad/s)] |
| `pid_ki` | `0.5` | 積分ゲイン [A/rad] |
| `pid_kd` | `0.03` | 微分ゲイン [A·s/rad] |
| `pid_derivative_filter_tau_s` | `0.05` | 微分の一次フィルタ時定数 [s] |

これらは指定された**初期ゲイン**であり、実機での安定性・追従性能を確認して調整する必要があります。制御出力はモータ電流 [A] であり、数値をそのままドライバ内蔵 PID のゲインとして使用するものではありません。

PID と LADRC は既存の BREAKAWAY、TRACK_HOLD、失速検知、その場旋回時の協調始動、直進同期補正、通常停止の減速ランプを共用します。PID の積分状態は起動時、方向切替時、停止時、BREAKAWAY／失速復帰時に初期化されます。電流出力は既存の `current_limit_a` と TRACK 時の変化率制限を経由します。PID の積分には電流上限に対する条件付き積分（アンチワインドアップ）を適用します。

**比較実験上の注意：** 本実装の比較には共通の補助制御が含まれます。また、PIDの通常減速ランプには公称入力ゲイン `b0` を使ったフィードフォワードも含みます。制御則そのものの比較には TRACK 中の区間を主評価とし、起動・停止区間は別に評価してください。

## ROS 2 とドライバのインターフェース

| ジョイント | コマンド | 状態 |
| --- | --- | --- |
| `left_wheel_joint` | `velocity` [rad/s] | `position` [rad]、`velocity` [rad/s]、`effort` |
| `right_wheel_joint` | `velocity` [rad/s] | `position` [rad]、`velocity` [rad/s]、`effort` |

本実装の `effort` は、慣例的なトルク [N·m] ではなく、**実測モータ電流 [A]** を格納します。右車輪はドライバと ROS 2 で正方向が一致しますが、左車輪では角速度・電流・エンコーダ位置・指令の符号を反転して、ROS 2 の前進方向に合わせています。

エンコーダ換算は実装上 **1024 line × 4 = 4096 count/rev** です。位置は最初に受信したエンコーダ値を原点として積算し、32-bit カウンタの折返しを考慮しています。ハードウェアの再起動で原点が変わるため、オドメトリの継続性には注意してください。

## 現在の設定パラメータ

制御モードとゲインは bringup リポジトリの `urdf/orange_go_2025.urdf` 内、`<ros2_control><hardware>` で設定します。

### 対応する bringup の車輪・オドメトリ設定

最新の [`zlac8015d_can_bringup/config/controllers.yaml`](https://github.com/shozawakoki/zlac8015d_can_bringup/blob/main/config/controllers.yaml) では、実機で確認した有効車輪間距離 `wheel_separation: 0.354` [m]、車輪半径 `wheel_radius: 0.064` [m] を使用します。`orange_go_2025.urdf` の左右車輪ジョイントの Y 座標はそれぞれ `+0.177` / `-0.177` [m] です。

車輪寸法と速度指令の変換・車輪オドメトリ計算は bringup 側の `diff_drive_controller` が担当します。本ハードウェアプラグインが受け取る指令は車輪角速度 [rad/s] であり、上記 `wheel_separation` を直接読み込みません。

実機では左モータの符号調整が完了しています。また GPS・IMU を利用した外部の位置推定系でロボット位置を扱っていますが、本プラグインのエンコーダ位置は起動ごとに相対原点へ初期化されます。外部位置推定系自体はこれら2つのリポジトリに含まれません。

| パラメータ | 現行 URDF 値 | 内容 |
| --- | --- | --- |
| `can_interface` | `can0` | SocketCAN デバイス |
| `node_id` | `1` | CANopen ノード ID（1～127） |
| `control_mode` | `ladrc_torque` | `velocity` / `ladrc_torque` |
| `ladrc_b0_left` | `18.0` | 左車輪の公称入力ゲイン [(rad/s²)/A] |
| `ladrc_b0_right` | `18.0` | 右車輪の公称入力ゲイン [(rad/s²)/A] |
| `ladrc_wc` | `5.0` | 制御帯域幅 [rad/s] |
| `ladrc_wo` | `20.0` | LESO 帯域幅 [rad/s] |
| `current_limit_a` | `8.0` | TRACK 電流指令の絶対値上限 [A] |
| `current_rate_limit_a_per_s` | `20.0` | TRACK 電流の変化率上限 [A/s] |
| `feedback_timeout_ms` | `30.0` | 速度受信のタイムアウト閾値 [ms] |

`b0` は機体・運転条件に合わせた同定結果を適用してください。PIDモードでも通常停止フィードフォワードに使用します。`18.0` は現在の URDF に記載された公称値です。**静止摩擦対策に用いる BREAKAWAY 電流ランプは C++ ソースに定数として定義されています。** また、一部の後段処理では電流変化率制限より優先して電流を変更するため、すべてのフェーズで設定された `current_rate_limit_a_per_s` が適用されるわけではありません。

## LADRC による電流指令

この章は `ladrc_torque` モードの説明です。`pid_torque` では LESO を動かさず、前述の PID によって TRACK 中の電流指令を計算します。

各車輪の制御対象を、運転中の近似として

```text
omega_dot = f + b0 * I_cmd
```

で扱います。`omega` は車輪角速度、`I_cmd` は電流指令、`f` はモデル化されないダイナミクスや外乱をまとめた総外乱です。TRACK 中、LESO は

```text
e      = omega_measured - z1
z1_dot = z2 + b0 * I_previous_cmd + 2 * wo * e
z2_dot = wo^2 * e
```

を陽的オイラー法で更新し、電流指令を

```text
I_raw = (wc * (omega_reference - z1) + a_ff - z2) / b0
```

から計算します。`a_ff` は通常走行では 0、通常停止時の減速ランプでは減速度フィードフォワードです。`I_previous_cmd` は実測電流ではなく、**前制御周期に指令した電流**です。実測電流は別途 CAN から取得して状態・CSV に記録します。

### 起動時の静止摩擦対策（BREAKAWAY → TRACK）

停止状態から電流を増やして回転開始を確認し、その後に通常の LADRC に切り替えます。

- BREAKAWAY：`0.65 A` から開始し、内部の制御カウントが進むと `0.50 A` ずつ増加。実装上の上限は `min(8.0 A, current_limit_a)`（最終指令でも電流上限を適用）。
- 単輪の TRACK 移行条件：指令方向の実測角速度が `0.30 rad/s` 以上の状態で `20` 制御サンプル継続。
- TRACK_HOLD：移行直後 `30` サンプルを用いて起動電流から通常の電流下限 `0.20 A` へ移行。
- STALL 検出：指令方向の角速度が `0.10 rad/s` 未満の状態で `20` サンプル継続すると BREAKAWAY に戻ります。

上記のサンプル数は制御ループ **200 Hz** を前提とした実装上のカウント値です。例えば `20` サンプルは公称 `100 ms` です。実際の経過時間はスケジューリング等の影響を受けます。

その場旋回（左右車輪が逆方向でほぼ同速）の場合は、左右がともに始動条件を満たすまで TRACK 移行を協調させます。直進時は、両輪が TRACK と TRACK_HOLD を終えた後に、左右速度差を使った同期補正を段階的に加えます。この補正は旋回や停止中には適用しません。

### 通常停止

`ladrc_torque` / `pid_torque` モードで左右の目標車輪角速度がゼロになると、`write()` 内部で**急停止ではなく通常減速ランプ**を開始します。

- 車体並進減速度の設定値：`1.0 m/s²`（ソース内定数）。
- 車輪半径 `0.064 m` に対応する車輪角減速度：`15.625 rad/s²`。
- 両車輪の実測角速度が `0.50 rad/s` 以下になったら、電流ゼロを保持する ZERO HOLD に移行。
- 次の非ゼロ速度指令が入ると ZERO HOLD を解除。

CiA402 Quick Stop 用のパラメータ設定は初期化コードに存在しますが、**通常のゼロ速度指令では Quick Stop を発行しません**。異常時の安全停止がハードウェアで保証されているという意味ではありません。

## CANopen PDO と更新設定

CANopen ノード ID が `1` の場合：

| CAN ID | 方向 | データ |
| --- | --- | --- |
| `0x181` | ZLAC8015D → PC | 左右実測角速度（各 `int32`、0.1 rpm 単位） |
| `0x281` | ZLAC8015D → PC | 左右実測電流（各 `int16`、0.1 A 単位） |
| `0x381` | ZLAC8015D → PC | 左右エンコーダ位置（各 `int32`、count） |
| `0x301` | PC → ZLAC8015D | 目標速度（`velocity` モード） |
| `0x201` | PC → ZLAC8015D | 左右目標電流（`ladrc_torque` / `pid_torque`、各 `int16`、mA 単位） |
| `0x601` | PC → ZLAC8015D | SDO 設定コマンド |

起動時に NMT / SDO を使って PDO マッピング、動作モード（Profile Velocity = `3` または Profile Torque = `4`）、CiA402 の初期設定を送信します。

現在の実装が TPDO イベントタイマへ**書き込む値**は、実測角速度と電流がともに `10 ms`、位置が `40 ms` です。実際の受信周期はドライバの応答・設定状態・CAN バスの状況に依存するため、`candump` で確認してください。なお、C++ のログメッセージやコメントには `5 ms / 20 ms` と記載された箇所があり、**イベントタイマへ送信する値と不一致**です。ソースを変更せず、この相違を明記しています。

## フィードバック監視とログ

`ladrc_torque` / `pid_torque` では、未受信の速度フィードバックがある場合はゼロ電流指令を試みます。最終受信から `feedback_timeout_ms` を超えると、ゼロ電流指令の送信を試みて `ros2_control` にエラーを返します。**CAN 断線時はゼロ電流フレーム自体が届かない可能性があるため、安全停止を保証するものではありません。** `velocity` モードには、この電流制御モードのフィードバック監視は適用されません。

制御モード別の実行ログ出力先：

```text
/tmp/ladrc_track_200hz.csv  # LADRC モード
/tmp/pid_track_200hz.csv    # PID モード
```

CSV は実装上、目標・実測角速度、位置、指令電流・実測電流、`z1` / `z2` 診断列、TRACK フラグ、BREAKAWAY 目標電流を収録します。LADRC モードの `z1` / `z2` は LESO 推定値ですが、PID モードでは `z1` は実測角速度、`z2` は 0 で、LESO 推定を意味しません。PIDログには積分電流成分とフィルタ後の角加速度も追加しています。通常は `200` サンプルごとにフラッシュされます。`velocity` モードではいずれの CSV も生成しません。ログファイル名の `200hz` は公称制御レートに基づきます。

ログ確認：

```bash
# 選択した制御モードに応じたファイルを確認
head -n 5 /tmp/pid_track_200hz.csv
tail -n 5 /tmp/pid_track_200hz.csv
```

## 安全上の注意

- **モータを動かす前に非常停止手段を確保**し、初回は車輪を浮かせて回転方向・停止挙動を確認してください。
- `current_limit_a` は TRACK と BREAKAWAY の最終電流指令に適用します。すべての状態遷移で電流変化率が一律に守られるわけではありません。ドライバ側の電流制限・許容値も必ず確認してください。
- CANopen ノード、モータ配線、符号、エンコーダ分解能が異なる環境では、実装を検証してから使用してください。
- 同定用などの独立した CAN 操作プログラムと bringup を同時実行しないでください。
- CAN 断線、ドライバ故障、PC 停止時の安全はこのソフトウェアだけでは保証できません。非常停止回路、ドライバの保護機能を別途用意してください。
- `can_launch_watchdog.py` はこのリポジトリにはありません。別途用いる場合は bringup の二重起動を避け、再起動に伴うオドメトリ原点の変化を考慮してください。

## ライセンス

現行 `package.xml` のライセンス欄は `TODO` です。公開・再利用条件は正式なライセンス表記が設定されるまで未確定です。


## Experimental MIMO-LADRC / MIMO-PID modes

The branch `feat/mimo-ladrc-pid-straight-tracking` adds two **opt-in** current-command modes: `mimo_ladrc_torque` and `mimo_pid_torque`. The existing `velocity`, `ladrc_torque`, and `pid_torque` modes remain available.

The matrix model and controller implementation live in [`include/zlac8015d_hardware/mimo_control.hpp`](include/zlac8015d_hardware/mimo_control.hpp). The C++ hardware plugin performs the same BREAKAWAY, TRACK, stall protection, current limits, normal-stop ramp, and CAN packing as the legacy implementation; it enables coupled body-coordinate control only after both wheels are in TRACK. The transformed feedback is encoder-derived `[v, omega]`. Neither an IMU feedback loop nor a heading controller has been added.

The initial MIMO LADRC bandwidths are `wc_v=wc_omega=5` and `wo_v=wo_omega=20` rad/s. The MIMO PID gains are seeded from wheel PID `Kp=0.7, Ki=0.5, Kd=0.03` with derivative filter `tau=0.05 s`. Independent body-axis parameters are set in the matching bringup URDF (wheel radius 0.064 m, separation 0.354 m). Equal v/yaw gains reproduce legacy control under symmetric, unsaturated plant assumptions; MIMO by itself does **not** guarantee straighter actual motion.

For build, mode selection, logging, and experimental/safety constraints, see the [companion bringup instructions](https://github.com/shozawakoki/zlac8015d_can_bringup/blob/feat/mimo-ladrc-pid-straight-tracking/MIMO_CONTROL.md). The standard-library-only test can be run without ROS:

```bash
g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude tests/test_mimo_control.cpp -o /tmp/test_mimo_control
/tmp/test_mimo_control
```

**Unverified:** the new mode has not been compiled against ROS 2 Humble or tested on a physical robot in this workspace; the matrix test does not constitute hardware validation.
