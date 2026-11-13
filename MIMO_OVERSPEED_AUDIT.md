# MIMO-LADRCの複合指令における速度超過の切り分け

## 目的
直進・旋回それぞれの単独指令では問題が小さい一方、直進と旋回の複合指令で速度超過と2 A飽和が生じた原因を、既存CSVから確認する。現在のMIMO制御則・ゲイン・ドライバ設定は変更しない。

## 既存コードから読み取れる切替条件
現行の src/zlac8015d_system.cpp では、左右のBREAKAWAYが終了し、両輪がTRACK状態にあり、なおかつ左右両方の目標角速度が非ゼロであるときだけmimo_readyが真となる。片輪目標がゼロになるとMIMO-LADRCのESOがリセットされ、独立輪制御の計算へ移る。この挙動は検討を要するが、複合指令で必ず発生しているとは限らない。

また、MIMO-LADRCのESOは前周期の実電流ではなく、CANへ送った指令電流を既知入力として利用する。指令・実電流の差や遅れはESOの総外乱推定によって補償され得るが、常に十分とは限らない。

## オフライン分析の実行
```bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking
python3 -m unittest discover -s tests -p 'test_analyze_mimo_overspeed.py' -v
mkdir -p ~/ros2_ws/mimo_results
if [ -s /tmp/mimo_ladrc_track_200hz.csv ]; then
  cp /tmp/mimo_ladrc_track_200hz.csv ~/ros2_ws/mimo_results/mimo_ladrc_original.csv
  python3 tools/analyze_mimo_overspeed.py ~/ros2_ws/mimo_results/mimo_ladrc_original.csv --limit-a 2.0
else
  echo '保存済みMIMOログが見つかりません。実機の再走行はせず、以前のCSVを探してください。'
fi
```

ログはROS起動時に上書きされる可能性があるため、必ず元CSVを別フォルダへコピーする。

## 分析結果の解釈
- 複合指令の判定：|v_ref| >= 0.05 m/s、|omega_ref| >= 0.1 rad/s
- 2 A上限時の飽和判定：いずれかの指令電流の絶対値が1.98 A以上
- 確認対象：車体v・omegaの速度誤差、左右の電流飽和、速度誤差と逆符号の電流、片輪ゼロ目標、TRACKフラグ
- 出力：元CSV名に _overspeed_audit.csv を追加した診断用CSV

注意：mimo_readyの専用ログは存在しない。片輪目標ゼロ・TRACKフラグから切替の可能性を調べるものであり、断定はできない。また一瞬の電流指令と速度誤差の符号不一致は、外乱補償や加速度フィードフォワードの可能性もあり、制御則の誤りとは断定できない。

新たな開ループ電流試験やMIMOの実機再走行は、既存ログを確認するまでは実施しない。
