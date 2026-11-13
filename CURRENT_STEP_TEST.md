# ZLAC8015D 電流指令・実電流・車輪回転速度のステップ試験

## 目的

MIMO-LADRC使用時、直進と旋回を重ねた指令で速度が大きく超過し、電流指令が2.0 Aまで到達した事象を切り分ける。

本試験は**LADRC/ESO/PID、BREAKAWAY、TRACK、`/cmd_vel`およびROSの速度制御を通さず**、ZLAC8015DへCANopen経由で電流指令を直接送信する。指令された電流、ドライバが通知した実電流、左右の車輪速度を比較する。MIMOの実装は変更しない。

ただし、以下を区別すること：

- モータに印加された**実電流**の通知は CAN TPDO1（6077h:01/:02、0.1 A単位）
- 車輪速度の通知は CAN TPDO0（606Ch:01/:02、0.1 rpm単位）
- 電流**指令**は RPDO0（6071h:01/:02、mA単位）
- ここで記録する時刻はPCの受信処理時刻。ドライバ内部の厳密なサンプリング時刻ではない
- 実電流の指令追従を確認しても、モデル同定誤差、制御符号、ESO、無駄時間、電流の飽和や速度の非線形性等の原因が除外されるわけではない

## 試験プロファイル（ROS前進を正、各3秒）

| 時刻[s] | 電流指令[A] |
|---:|---:|
| 0–3 | +2 |
| 3–6 | +4 |
| 6–9 | −4 |
| 9–12 | −2 |
| 12–15 | +6 |
| 15–18 | −2 |
| 18–21 | +4 |
| 21以降 | 0 |

単輪試験がデフォルト（`--wheel left`）。`--wheel right`、`--wheel both` も指定可能だが、両輪同時印加は機体移動の危険が大きい。左輪のモータドライバ側の符号のみ反転する（既存のROS正方向に統一）。

**危険：このプロファイルは全区間、速度の閉ループ制御をしない。車輪を浮かせた場合は速度が上昇し続ける場合があり、3秒の段階終了前に過速度停止となる可能性がある。**

## 事前条件

1. ホイールを浮かせて固定し、機体が転倒・移動しない状態にする。回転部に人・ケーブルを近づけない
2. 物理的に独立した非常停止手段を用意する。CAN断線時にはソフトウェアのゼロ電流送信が届かない場合がある
3. モータおよびZLAC8015Dの連続・ピーク電流定格、電源容量、連続時間・逆転条件を確認する
4. `ros2 launch zlac_can_bringup ...`、`ros2_control_node`、`can_launch_watchdog.py`、他のCAN制御送信プログラムをすべて停止する。**既存のROS2コントローラと同時に実行してはいけない**
5. CANインターフェース `can0` がUPであることを確認する。別のCAN監視端末（受信専用）は許容する

このプログラムは既存の `zlac8015d_hardware` と同じCANopenパラメータを**自分で初期化**するため、`colcon build` とROS起動は不要。LinuxのPython 3標準ライブラリだけを使用する。

**初期の電流上限は2.0 A。指定の最大6.0 Aプロファイルでは、そのまま実行すると事前検査で拒否する。** 6 Aを試す場合は定格を実測・仕様で確認した後に、`--max-current-a` を明示的に設定する。ドライバ内蔵電流上限を自動的に引き上げる処理は一切行わない。

## 実行前のプレビュー（実機には指令を送信しない）

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking

python3 -m unittest discover -s tests -p test_current_step_test.py -v

python3 tools/current_step_test.py
~~~

デフォルトで表示だけを行う。**この段階ではCANソケットを開かず、電流は印加しない。**

## 低電流の予備試験（例：最大0.6 A）

モータ・ドライバに対する安全条件を満たし、車輪を浮かせ、独立した非常停止を準備した場合のみ実行する。

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware

python3 tools/current_step_test.py \
  --execute \
  --wheel left \
  --profile "0.2,0.4,-0.4,-0.2,0.6,-0.2,0.4" \
  --max-current-a 0.6 \
  --max-speed-rpm 120 \
  --confirm-rated-current \
  --confirm-wheels-raised \
  --confirm-e-stop \
  --confirm-exclusive-can
~~~

これは**通信・符号・停止動作の予備試験**であり、目標の2～6 A試験の代わりではない。静止摩擦で車輪が回らない可能性もある。速度上限に達した場合、全プロファイル完了前に試験は停止する。

## 要求された2～6 Aの試験

**最大6 Aがモータ・ドライバの定格・電源・治具の許容内であることを確認できた場合のみ**、次のように電流上限を明示する。

~~~bash
python3 tools/current_step_test.py \
  --execute \
  --wheel left \
  --max-current-a 6.0 \
  --max-speed-rpm 120 \
  --confirm-rated-current \
  --confirm-wheels-raised \
  --confirm-e-stop \
  --confirm-exclusive-can
~~~

回転速度制限のデフォルトは120 rpmであり、**6 Aを含む全21秒の実行を保証しない**。モータの許容回転速度が未確認のまま `--max-speed-rpm` を増加させない。測定時に有意な電流応答・回転速度応答が確認できたら、過度に高い電流を続けず停止すること。

`Ctrl+C` またはSIGTERMで中断を要求できるが、物理的非常停止の代わりにはならない。正常終了・中断のいずれでも、ゼロ電流送信を複数回行い、CiA402の動作停止を要求する。ただし通信断線時に指令到達を保証しない。

## CSVデータ

保存先（自動生成）：

`~/ros2_ws/current_step_results/current_steps_YYYYMMDD_HHMMSS_left.csv`

左右モータとも同時に観測する（試験対象以外は電流指令0 A）。

| 列名 | 内容 |
| --- | --- |
| `elapsed_s` | ステップ開始からの経過時間[s] |
| `wall_time_iso` | PCの現在時刻（UTC） |
| `event` | `step_change`、`velocity_rx`、`current_rx`、`complete_zero`など |
| `step_index` / `requested_step_A` | 段番号（0始まり）/ 印加した指令電流[A] |
| `command_left_A` / `command_right_A` | ROS正方向の左右指令電流[A] |
| `driver_left_mA` / `driver_right_mA` | CANに実際にパックした左右指令[mA]（左輪の符号は逆） |
| `actual_left_A` / `actual_right_A` | TPDO1で受信した左右の実電流[A] |
| `left_rpm` / `right_rpm` | TPDO0で受信した左右速度[rpm] |
| `left_rad_s` / `right_rad_s` | 同じ左右角速度[rad/s] |
| `velocity_rx_age_ms` / `current_rx_age_ms` | その行で保持している値の受信からの経過時間[ms] |
| `feedback_can_id` / `can_rx_time_s` | 受信したCAN-IDとPC単調時計時刻 |

ログ行は**電流指令ステップの変更時と各TPDO受信時**に書き込み、制御周期で新しいデータが届かない場合に架空のセンサ点を作らない。CAN速度/電流通知はいずれも約10 ms周期に設定されるが、これは実測値の取得精度や実通信周期の保証ではない。

時刻差と各電流区間を比較するときは、`event=current_rx` の行で電流値を、`event=velocity_rx` の行で速度値を確認する。それ以外の欄には直近で受信した値が保持される。

## 保存データの分析とグラフ作成

試験終了後、CSVファイル名を確認し、次のコマンドを実行する。

~~~bash
ls -lt ~/ros2_ws/current_step_results/*.csv | head

# 必要なら matplotlib をインストール（PNGグラフ出力用）
sudo apt install python3-matplotlib

LATEST=$(ls -t ~/ros2_ws/current_step_results/*.csv | head -n 1)
python3 ~/ros2_ws/src/zlac8015d_hardware/tools/analyze_current_step.py "$LATEST" --plots
~~~

各段の指令電流と実電流の平均値、平均絶対誤差（MAE）、最大絶対誤差、最高回転速度を表示する。同じディレクトリに電流指令・実電流比較グラフ（`*_current.png`）と車輪回転速度グラフ（`*_rpm.png`）を別々に保存する。

**注意：** この集計はステップ応答初期の過渡区間も含むため、指令追従の定常誤差だけを示すものではない。電流応答遅れと定常値はグラフから別々に評価すること。途中で過速度停止した場合、記録されていない段の結果は表示されない。

## 全区間の実電流・車輪速度がゼロだった場合の診断

実電流がおおむね0 A、回転速度も0 rpmのまま、例えば+2～+6 Aまで指令が変化した場合は、電流制御の時間応答よりも、**CiA402のサーボ状態・動作モード・RPDOの受理状況**を先に確認する。

`tools/current_step_test.py` は、ゼロ電流指令での初期化後に以下を**SDOで読み戻す**ように変更した。

- `6041h:00` Statusword: 32ビットの上下16ビットを個別に分解し、**両モータ**の `Operation enabled` を確認（それぞれ `statusword16 & 0x006F == 0x0027`）
- `6061h:00` Mode display: `4`（Profile Torque）を確認
- 有効状態またはモードが違う場合、**非ゼロの電流ステップへ進まず中断**
- 1 A以上の指令を出し、約0.75秒間、実電流が0.2 A未満かつ車輪速度が1 rpm未満のままであれば中断

停止後のプログラムは `6040h=6` を送信し、動作を無効にする。そのため、**試験終了後にStatuswordを読むとOperation enabledではないことがある**。終了後の値だけで、運転中の状態を判断しないこと。

実機を駆動せずにSDOオブジェクトを照会したい場合は（装置停止・安全確保済み）、次を使用する。

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware
python3 tools/inspect_drive_status.py --interface can0 --node-id 1
~~~

このプログラムは**SDOアップロード（読み出し）だけ**を送信する。Statusword `6041h`、Mode display `6061h`、Error code `603Fh`、指令・実電流の読み出し可否を表示する。機種でサブインデックスが未対応の場合はSDO Abortが返ることがある。

また、電流ステップログの**最後の約1秒間**は停止後のゼロ電流期間である。従来の分析器はこれを最終ステップへ混ぜ、MAEが不当に低下していた。修正版の `analyze_current_step.py` は `complete_zero` 以降をステップごとの統計から除外する。

## 原因切り分けの観点

- **指令と実電流が大きく異なる：** CAN符号/単位、モータドライバの内部電流制御、電流制限、電源、通信・更新周期などを調査する
- **実電流は指令に追従するが回転が過大：** 指令電流による加速自体は当然起こり得る。モータの動特性（摩擦、飽和、逆起電力）、\(b_0\) 同定、MIMO-ESO/遅延によるモデル誤差を調査する
- **特に正負電流反転時に追従が悪い：** 電流応答の遅れ、電流制限、ドライバの保護動作、回生電圧等を調査する
- **MIMOと独立制御のどちらでも起こる：** モータ/電流ループ側の問題を優先する。ただしモデル化・センサ値の影響も残る

**注：モータ回転中の負方向電流は減速トルクとは限らない。** 実際の回転方向・電流の符号によって加速トルクにも減速トルクにもなるため、正負の指令切替は十分注意すること。
