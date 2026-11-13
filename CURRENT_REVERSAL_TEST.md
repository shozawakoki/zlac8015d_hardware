# ZLAC8015D 5秒保持→電流反転試験

## 検証目的

モータが十分に回転している状態で+1.0 Aから−1.0 Aに切り替え、減速応答および逆回転を確認する。負電流は逆方向のトルク指令であり、回転方向が即座に反転する指令ではない。

標準シーケンス（新しい --switch-after-s オプション）：

- 事前確認：両輪の速度と電流が連続1秒安定、ドライバの状態と目標電流値を読み戻す。**通電直前に2008hを追加読出しし、ソフト側停止閾値がこの設定値以下であることを確認する**。2008hは速度リミッタの作動確認ではない
- t = 0：左輪へ+1.0 A
- t = 5.0 s：実際に正方向へ5 rpm以上で回転していることを確認したうえ、左輪へ−1.0 A
- 反転後：電流実測値、速度低下、0 rpm通過、逆回転までを記録。最大2秒間監視
- −3 rpm以下を2回連続検出：0 Aへ戻してサーボ無効化を要求
- どの段階でも速度超過・CANフィードバック断・時間切れ・反対側車輪の予期しない回転があれば中断

**重要：指定した5秒の間に安全上限へ達した場合、−1 Aへの反転はせず中断する。**

## 速度上限について

以前の1 A空転試験では0.142秒で62.5 rpmに達して安全中断した。今回は「モータに速度上限がある」との見込みだが、**最大機械回転数とドライバ内部の能動的なトルク/電流制限が等価とは限らない**。

ソフトウェア側の過速度監視を無効にはしない。

- 初期設定は45 rpmで中断する。この条件では5秒前に中断する可能性が高い。
- 60 rpmまでの停止閾値は既定の事前確認で使用できる。
- 60 rpmを超える停止閾値は、実機の**モータ・ドライバ・固定治具全体に対して検証済みの安全速度**を --verified-safe-rpm で明示した場合に限る。なお、実機通電直前の2008h読出し値をソフトウェア停止閾値が超えた場合は、通電せず中断する。--max-speed-rpm の値がこれを超えてはならない。最大300 rpmまでしか指定できない。
- --verified-safe-rpm に推測した値を入れない。モータ銘板・ドライバ設定・メーカー資料・治具仕様から検証した値のみを指定する。過速度を検出してから0 Aを送るので、設定値を一時的に超過する可能性がある。
- 回転数制限がドライバ内で動作しているか確認するだけでは、急な回生制動時のDCバス過電圧を防げる保証はない。回生エネルギーを吸収できる電源/放電回路と独立非常停止を確認する。

## 1. 試験プログラムの更新・テスト・プレビュー（安全：モータ駆動なし）

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking

python3 -m unittest discover -s tests -p 'test_current_reversal_test.py' -v
python3 -m unittest discover -s tests -p 'test_analyze_current_reversal.py' -v

python3 tools/current_reversal_test.py \
  --wheel left \
  --current-a 1.0 \
  --switch-after-s 5.0 \
  --reverse-confirm-rpm 3 \
  --max-speed-rpm 45
~~~

PREVIEW ONLY が出ることを確認する。--execute が無い場合はCANを開かない。新規テストケースはローカルの単体テストで成功する必要がある。GitHubへの保存だけでは実機確認にはならない。

## 2. 実機検証（安全条件が成立した場合のみ）

実機試験には --execute と明示的な安全確認フラグが必要。周囲に作業者がいない支持台、確実な車輪浮上・固定、手の届く独立物理非常停止、負荷・摩擦・共振・回生過電圧対策、正しい定格値、他制御プログラム停止を確認する。

具体的な停止回転速度は対象モータ・治具の検証済み速度から決定し、**他の試験で高回転が観測されたことだけを根拠に設定してはならない**。

従来の45 rpm閾値のまま実行した場合、5秒より前に停止する見込みなので、目的の試験は成立しない。この状態で閾値を上げるためだけに --verified-safe-rpm を大きく指定してはならない。

実行形式：

~~~bash
python3 tools/current_reversal_test.py \
  --execute \
  --wheel left \
  --current-a 1.0 \
  --switch-after-s 5.0 \
  --reverse-confirm-rpm 3 \
  --max-speed-rpm <検証済み停止閾値> \
  --verified-safe-rpm <検証済み安全回転速度> \
  --confirm-rated-current \
  --confirm-wheels-raised \
  --confirm-e-stop \
  --confirm-exclusive-can \
  --confirm-regeneration-safe
~~~

角括弧内は例示のプレースホルダーであり、そのまま実行しない。左輪のROS正方向+1 Aはドライバ値−1000 mA、−1 Aはドライバ値+1000 mA。右輪は0 A指令のまま。

## 3. 時間応答のオフライン解析

CSVの保存先：~/ros2_ws/current_step_results/current_reversal_YYYYMMDD_HHMMSS_left.csv

~~~bash
python3 tools/analyze_current_reversal.py \
  ~/ros2_ws/current_step_results/current_reversal_YYYYMMDD_HHMMSS_left.csv \
  --wheel left
~~~

評価指標：電流反転指令時刻、6077h が負になるまでの観測遅れ、速度が0.5 rpm低下するまでの観測遅れ、0 rpm通過時間、逆回転確定時間、反転後の最大正方向速度。6077hと606Chはドライバ内部のフィルタや更新周期の影響を受け得るため、観測値は物理的な電流/トルク応答開始時刻と必ずしも同じではない。

## 4. 従来の速度トリガーモード

--switch-after-s を省略すると従来どおり --switch-rpm 15 で速度検出時に電流を反転する。こちらの逆回転観測時間の既定は0.4秒。


## 5. 5秒保持の実機コマンド（安全確認済み構成・2008h=370 rpm）

既知の設定値が 2008h=370 rpm であり、試験装置・制動回生の安全性を確認済みの場合：

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking
python3 -m unittest discover -s tests -p 'test_current_reversal_test.py' -v

python3 tools/current_reversal_test.py \
  --execute \
  --wheel left \
  --current-a 1.0 \
  --switch-after-s 5.0 \
  --reverse-confirm-rpm 3 \
  --max-speed-rpm 370 \
  --verified-safe-rpm 370 \
  --confirm-rated-current \
  --confirm-wheels-raised \
  --confirm-e-stop \
  --confirm-exclusive-can \
  --confirm-regeneration-safe
~~~

370 rpmは速度目標ではなく、**速度超過時に試験を停止する閾値**である。試験開始前に2008hを読み直し、設定値より高いソフトウェア閾値なら非ゼロ電流を印加せずに拒否する。

停止成功・失敗にかかわらずCSVファイルは current_step_results 配下に記録。逆回転を2回連続検出したら0 A指令・サーボ無効化へ進む。最大2秒で逆回転が確認できなければ中断。実機の回転数が370 rpmを超過した場合は5秒を待たずに中断するため、5秒後の反転は実施されないことがある。物理非常停止は常に使用可能にする。


## 6. 2026-11-13 過速度中断CSVのオフライン解析

5秒保持試験を370 rpmの停止閾値で実行したところ、左輪372.3 rpmを受信して中断。反転コマンドが送られなかったため、旧解析器は reverse_command がない旨のエラーを出した。これはCSVの欠損ではなく、試験が反転前に安全停止したため。

更新した analyze_current_reversal.py は反転イベントがなくても forward_start を起点に最後の新規速度受信時刻、最大速度とその時刻、45/100/200/300/350/370 rpmへの初到達時刻、実電流の最小・平均・最大値を表示する。

~~~bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking
python3 -m unittest discover -s tests -p 'test_analyze_current_reversal.py' -v
python3 tools/analyze_current_reversal.py \
  ~/ros2_ws/current_step_results/current_reversal_20261113_112805_left.csv \
  --wheel left
~~~

この試験では **−1 A指令による制動・逆回転を確認できていない**。370 rpmという設定値に達してもトルク制御で安全に速度が維持されると証明できないため、上限を上げたり無効化して5秒試験を強行しない。
