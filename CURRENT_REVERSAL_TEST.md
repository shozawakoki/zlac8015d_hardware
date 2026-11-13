# ZLAC8015D +1 A → −1 A 電流反転試験

## 目的

+1 Aで正方向に一定速度まで加速し、**新しく受信した速度が15 rpmを超えた瞬間**に−1 Aへ切り替える。負のトルクで減速するか、0 rpmを通過して逆回転するか、電流・速度の応答時間を観測する。−1 Aは逆速度指令ではなく逆**トルク**指令。即座に負の回転速度になるとは限らない。

## 安全上の制約

過去の1 A空転試験では0.142秒で62.5 rpmとなり過速度中断した。固定時間の +1→−1 電流ステップではなく**速度トリガー**とする。

- デフォルトはプレビュー専用。--execute が無ければCANソケットも開かない。
- 車輪を浮かせて確実に固定し、独立した物理非常停止、モータ・ドライバ・電源の許容電流と回生制動対策を確認する。ZLAC8015Dのメーカー資料には急停止時の回生エネルギー／放電回路に関する注意がある。推測でブレーキ抵抗を追加しない。
- ROS2コントローラ・CAN監視自動再起動など、他の制御プログラムと同時実行しない。
- 事前に両輪速度±0.5 rpm以内、6077h±0.1 A以内の状態が連続1秒成立することを確認し、6041h/6061h/6071h/6077h/DCバス電圧を読み戻す。
- +1 Aで15 rpmに達した最初の新規速度受信時に−1 Aへ切り替える。両輪速度の絶対値が45 rpmを超えたら中断する。正方向の速度が閾値に達しなければ0.25秒で中断、逆回転を0.40秒以内に確認できなければ中断。
- −3 rpm以下の新規速度受信を2回連続確認したら、0 Aを指令して動作無効化を要求する。非指令車輪が5 rpmを超えた場合も中断。
- 停止指令は通信断線時に到達保証がない。**物理非常停止に代わらない**。設定された45 rpmはソフト側の検出値であり、超過しない保証ではない。
- 既存ドライバ初期化処理のPDO割当と 605Ah/6085h の設定は行う。6087hや電流制御ゲインは変更しない。

## 1. GitHub同期・単体テスト・プレビュー：実機駆動なし

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking

python3 -m unittest discover -s tests -p 'test_current_reversal_test.py' -v
python3 -m unittest discover -s tests -p 'test_analyze_current_reversal.py' -v

python3 tools/current_reversal_test.py \
  --wheel left \
  --current-a 1.0 \
  --switch-rpm 15 \
  --reverse-confirm-rpm 3 \
  --max-speed-rpm 45 \
  --max-forward-s 0.25 \
  --max-reverse-s 0.40
~~~

テストがすべてOK、プレビューにPREVIEW ONLYが出ることを確認する。

## 2. 実機試験（機械・電源・回生の安全性を確認後）

前回の過速度中断と急減速時の回生に備えた安全対策が確認できない場合、実行しない。

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware

python3 tools/current_reversal_test.py \
  --execute \
  --wheel left \
  --current-a 1.0 \
  --switch-rpm 15 \
  --reverse-confirm-rpm 3 \
  --max-speed-rpm 45 \
  --max-forward-s 0.25 \
  --max-reverse-s 0.40 \
  --confirm-rated-current \
  --confirm-wheels-raised \
  --confirm-e-stop \
  --confirm-exclusive-can \
  --confirm-regeneration-safe
~~~

左輪のROS正方向+1 AはCAN値−1000 mA、ROS負方向−1 AはCAN値+1000 mA。右輪への指令は0 A。途中で中断したら閾値を安易に引き上げない。

## 3. 保存データと解析

保存先：~/ros2_ws/current_step_results/current_reversal_YYYYMMDD_HHMMSS_left.csv

イベント：forward_start、reverse_command、zero_speed_crossing、reverse_confirmed、complete_zero、current_rx、velocity_rx。個々のTPDO受信イベントは新しい値を測定したことを表す。

~~~bash
python3 tools/analyze_current_reversal.py \
  ~/ros2_ws/current_step_results/current_reversal_YYYYMMDD_HHMMSS_left.csv \
  --wheel left
~~~

表示するのは、負方向電流フィードバック開始までの遅れ、速度低下0.5 rpmまでの遅れ、0 rpm通過時間、逆回転確認時間、反転後最大正方向速度など。

6077hが負になるまで遅いなら電流ループ・トルクスロープの影響が候補。6077hが負でも速度低下が遅ければ実トルク、負荷、速度推定、計測遅延などを切り分ける。逆回転の確認が無い場合、タイムアウト・安全停止による未完了とモータの異常は区別する。

受信時刻はPC単調時計。6077hと606Chの内部更新やフィルタの遅延は未確認なので、得られる値は**フィードバック上の観測遅れ**であり、瞬時の実軸トルク遅れそのものではない。
