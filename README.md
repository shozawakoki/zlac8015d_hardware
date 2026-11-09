# zlac8015d_hardware

ZLAC8015D モータドライバを、Linux SocketCAN / CANopen 経由で ROS 2 の `ros2_control` から操作するための **C++ ハードウェアインターフェース**です。対向二輪型移動ロボットの左右駆動輪に対して、速度指令とエンコーダ位置・実測角速度・モータ電流の取得を行います。

2種類の制御モードを実装しています。

| モード | 内容 | ドライバへの指令 |
| --- | --- | --- |
| `velocity` | ZLAC8015D 内蔵の速度制御を使用 | 車輪角速度 |
| `ladrc_torque` | 左右車輪の一次 LADRC（線形拡張状態オブザーバ：LESO）で速度追従制御 | モータ電流 |

ROS 2 側の**コマンドインターフェースは、両モードとも車輪角速度 `velocity` [rad/s]** です。`ladrc_torque` モードのみ、プラグイン内部で電流指令に変換します。

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

## ROS 2 とドライバのインターフェース

| ジョイント | コマンド | 状態 |
| --- | --- | --- |
| `left_wheel_joint` | `velocity` [rad/s] | `position` [rad]、`velocity` [rad/s]、`effort` |
| `right_wheel_joint` | `velocity` [rad/s] | `position` [rad]、`velocity` [rad/s]、`effort` |

本実装の `effort` は、慣例的なトルク [N·m] ではなく、**実測モータ電流 [A]** を格納します。右車輪はドライバと ROS 2 で正方向が一致しますが、左車輪では角速度・電流・エンコーダ位置・指令の符号を反転して、ROS 2 の前進方向に合わせています。

エンコーダ換算は実装上 **1024 line × 4 = 4096 count/rev** です。位置は最初に受信したエンコーダ値を原点として積算し、32-bit カウンタの折返しを考慮しています。ハードウェアの再起動で原点が変わるため、オドメトリの継続性には注意してください。

## 現在の設定パラメータ

制御モードとゲインは bringup リポジトリの `urdf/orange_go_2025.urdf` 内、`<ros2_control><hardware>` で設定します。

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

`b0` は機体・運転条件に合わせた同定結果を適用してください。`18.0` は現在の URDF に記載された公称値です。**静止摩擦対策に用いる BREAKAWAY 電流ランプは C++ ソースに定数として定義されています。** また、一部の後段処理では電流変化率制限より優先して電流を変更するため、すべてのフェーズで設定された `current_rate_limit_a_per_s` が適用されるわけではありません。

## LADRC による電流指令

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

- BREAKAWAY：`0.65 A` から開始し、内部の制御カウントが進むと `0.50 A` ずつ増加。実装上の上限 `8.0 A`。
- 単輪の TRACK 移行条件：指令方向の実測角速度が `0.30 rad/s` 以上の状態で `20` 制御サンプル継続。
- TRACK_HOLD：移行直後 `30` サンプルを用いて起動電流から通常の電流下限 `0.20 A` へ移行。
- STALL 検出：指令方向の角速度が `0.10 rad/s` 未満の状態で `20` サンプル継続すると BREAKAWAY に戻ります。

上記のサンプル数は制御ループ **200 Hz** を前提とした実装上のカウント値です。例えば `20` サンプルは公称 `100 ms` です。実際の経過時間はスケジューリング等の影響を受けます。

その場旋回（左右車輪が逆方向でほぼ同速）の場合は、左右がともに始動条件を満たすまで TRACK 移行を協調させます。直進時は、両輪が TRACK と TRACK_HOLD を終えた後に、左右速度差を使った同期補正を段階的に加えます。この補正は旋回や停止中には適用しません。

### 通常停止

`ladrc_torque` モードで左右の目標車輪角速度がゼロになると、`write()` 内部で**急停止ではなく通常減速ランプ**を開始します。

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
| `0x201` | PC → ZLAC8015D | 左右目標電流（`ladrc_torque`、各 `int16`、mA 単位） |
| `0x601` | PC → ZLAC8015D | SDO 設定コマンド |

起動時に NMT / SDO を使って PDO マッピング、動作モード（Profile Velocity = `3` または Profile Torque = `4`）、CiA402 の初期設定を送信します。

現在の実装が TPDO イベントタイマへ**書き込む値**は、実測角速度と電流がともに `10 ms`、位置が `40 ms` です。実際の受信周期はドライバの応答・設定状態・CAN バスの状況に依存するため、`candump` で確認してください。なお、C++ のログメッセージやコメントには `5 ms / 20 ms` と記載された箇所があり、**イベントタイマへ送信する値と不一致**です。ソースを変更せず、この相違を明記しています。

## フィードバック監視とログ

`ladrc_torque` では、未受信の速度フィードバックがある場合はゼロ電流指令を試みます。最終受信から `feedback_timeout_ms` を超えると、ゼロ電流指令の送信を試みて `ros2_control` にエラーを返します。**CAN 断線時はゼロ電流フレーム自体が届かない可能性があるため、安全停止を保証するものではありません。** `velocity` モードには、この LADRC 固有のフィードバック監視は適用されません。

LADRC 実行ログの出力先：

```text
/tmp/ladrc_track_200hz.csv
```

CSV は実装上、目標・実測角速度、位置、指令電流・実測電流、LESO 推定 `z1` / `z2`、TRACK フラグ、BREAKAWAY 目標電流を収録します。通常は `200` サンプルごとにフラッシュされます。`velocity` モードではこの CSV は生成しません。ログファイル名の `200hz` は公称制御レートに基づきます。

ログ確認：

```bash
head -n 5 /tmp/ladrc_track_200hz.csv
tail -n 5 /tmp/ladrc_track_200hz.csv
```

## 安全上の注意

- **モータを動かす前に非常停止手段を確保**し、初回は車輪を浮かせて回転方向・停止挙動を確認してください。
- `current_limit_a` は主として TRACK の電流指令を制限します。BREAKAWAY 電流や内部の特別な処理を含めた包括的な安全制限ではありません。ソフトウェア側とドライバ側の許容電流を必ず確認してください。
- CANopen ノード、モータ配線、符号、エンコーダ分解能が異なる環境では、実装を検証してから使用してください。
- 同定用などの独立した CAN 操作プログラムと bringup を同時実行しないでください。
- CAN 断線、ドライバ故障、PC 停止時の安全はこのソフトウェアだけでは保証できません。非常停止回路、ドライバの保護機能を別途用意してください。
- `can_launch_watchdog.py` はこのリポジトリにはありません。別途用いる場合は bringup の二重起動を避け、再起動に伴うオドメトリ原点の変化を考慮してください。

## ライセンス

現行 `package.xml` のライセンス欄は `TODO` です。公開・再利用条件は正式なライセンス表記が設定されるまで未確定です。
