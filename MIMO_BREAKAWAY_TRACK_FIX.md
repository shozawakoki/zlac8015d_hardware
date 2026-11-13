# MIMO-LADRC：回転中のBREAKAWAY→TRACK遷移修正

## 対象・現象

- 再現ログ（2026-11-13）：旋回の目標車輪角速度 L=+2.766, R=-2.766 rad/s に対し、実際には L=-5.582, R=-24.588 rad/s。
- 両輪の TRACK=0 のまま左右へ約 +2/-2 A を長時間指令し、ESOはリセット状態になった。
- 原因：既存のBREAKAWAY終了判定が「車輪の実速度が指令方向で+0.30 rad/s以上」を要求し、逆方向に高速回転してもTRACKを許可しなかった。
- また、片方だけ始動条件を満たすと、連動旋回のREADY待機中にその車輪の電流を上限（既定で2 A）へ引き上げていた。

## 変更

1. MIMO-LADRC / MIMO-PIDモードでは、0.30 rad/s以上の実車輪速度があれば、回転方向に関係なくBREAKAWAYの開始カウンタを増やす。従来と同様、連続した受信時刻ではなく制御周期サンプル数による判定であり、既定では6サンプル。純旋回時は双方が5サンプル到達した時点で連動TRACK移行する。
2. MIMOモードで片輪がREADYになっても、READY待機だけを理由に始動電流を2 Aへ引き上げない。READY後は、それまでのBREAKAWAY電流を保持する。
3. 連動旋回で逆回転中にTRACKへ移行した場合、ROSログへ警告を記録する。
4. 従来のSISO-LADRC/PID制御の方向別始動判定・始動電流挙動には変更を加えない。
5. ESO初期化、既存の電流上限、電流変化率上限、通信タイムアウト、ストール監視、通常停止ランプとZERO HOLDはそのまま維持。

**注：** 逆回転している車輪に対してMIMO-LADRCが制動電流を要求することは、電流が直ちに反転することと同義ではない。既存の20 A/sの電流変化率制限を通過するため、実機応答は別途評価が必要。

## 端末A：無通電のC++回帰テストとビルド

~~~bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking

g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude \
  tests/test_mimo_control.cpp -o /tmp/test_mimo_control
/tmp/test_mimo_control

cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware --symlink-install
source ~/ros2_ws/install/setup.bash
~~~

期待：MIMO model/LADRC/PID checks passed。

## 端末A：確認試験のログ収集（安全な条件を優先）

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/zlac8015d_can_bringup
python3 scripts/record_mimo_accel_session.py \
  --label mimo_transition_fix \
  --trial 03
~~~

内部でZLAC_CONTROL_MODE=mimo_ladrc_torqueと既存launchを起動する。重複起動しない。操作コントローラは別端末から /cmd_vel に指令を送る。
まず車輪を安全に浮かせ、低速の方向反転と超信地旋回で遷移を確認する。問題がなければ適切な安全条件で地上走行評価を実施する。

テスト終了後は機体の停止を確認してCtrl+C。セッションディレクトリ内の `mimo_ladrc_track_200hz.csv`、`ros_launch.log` を保存。

## 比較する項目

- 以前の15.60〜19.35秒のような純旋回区間でtracking_left/tracking_rightが0/0のまま継続していないか
- READY待機の電流が即座に2 Aへ跳ね上がらないか
- TRACK遷移後のMIMO-ESO推定値が更新されるか
- 急加速区間で `cmd_current_left_A`、`cmd_current_right_A` に減速方向の指令が発生するか
- `actual_current_left_A`、`actual_current_right_A` が減速指令へどの程度遅れて追従するか
- 目標角速度に対する実角速度の最大偏差、0到達時間、指令からの遅れ

**未検証：** 本修正による実機の急加速抑制やオーバーシュート低減は、この時点では確定していない。必ずテスト・ビルド・実機ログで確認する。


## 旧バイナリによる誤検証の防止：ランタイム識別マーカー

前回の修正後実機ログでは、実車輪速度が十分に高くてもTRACKへの遷移が1.1〜1.3秒遅れ、READY待機電流が2 Aになった。旧ビルドが読み込まれた可能性がある。
ソースのgit pullだけではインストール済み共有ライブラリは更新されない。

新しいプラグインは起動時に識別メッセージ MIMO_TRANSITION_FIX_20261113_R2 を出力する。
以下をROS launch停止状態で実行する。

~~~bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking
g++ -std=c++17 -Wall -Wextra -pedantic -Iinclude tests/test_mimo_control.cpp -o /tmp/test_mimo_control
/tmp/test_mimo_control
cd ~/ros2_ws
colcon build --packages-select zlac8015d_hardware --symlink-install
source ~/ros2_ws/install/setup.bash
LIB="$(ros2 pkg prefix zlac8015d_hardware)/lib/libzlac8015d_hardware.so"
readlink -f "$LIB"
sha256sum "$LIB"
strings "$LIB" | grep -F MIMO_TRANSITION_FIX_20261113_R2
~~~

識別文字列がなければ再走行せず、ROS overlayとビルドを修正する。文字列がインストール済みライブラリにあるだけでは実際のプロセスに読み込まれた保証にならないため、実際のros_launch.logでも同じマーカーを確認する。
ログ収集プログラムは実行前にインストール済みライブラリを検査し、バイナリのパスとSHA-256をcapture_metadata.jsonに保存し、終了時に実行プロセス由来のマーカーの有無も報告する。
最初は車輪を浮かせた低速の方向反転・旋回試験に限定し、停止後にデータを比較する。
