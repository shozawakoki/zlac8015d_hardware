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
  python3 tools/analyze_mimo_overspeed.py ~/ros2_ws/mimo_results/mimo_ladrc_original.csv --limit-a 8.0
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

## 重要：2 Aではなく実際のTRACK上限を使用する（2026-11-13追記）

実測の複合運動ログでは、0.1秒の区間平均電流指令が3～4 Aを超えていた。このため、旧コマンド `--limit-a 2.0` の44.5%「飽和」は実際には **|Icmd|>=1.98 Aを満たすサンプルの割合**であり、最終電流上限の飽和率ではない。

両GitHubリポジトリの `feat/mimo-ladrc-pid-straight-tracking` ブランチを確認すると、bringup側のURDFには `current_limit_a=8.0`、hardware側C++のデフォルトも8.0 A、`breakaway_max_a` のデフォルトは2.0 Aと定義されている。**しかし、実機に読み込ませたURDFや実行バイナリが同一かどうかは未確認**。local/installのURDFを照合する。

```bash
grep -nE '<param name="(control_mode|current_limit_a|breakaway_max_a|current_rate_limit_a_per_s)"' \
  ~/ros2_ws/src/zlac8015d_can_bringup/urdf/orange_go_2025.urdf

grep -nE '<param name="(control_mode|current_limit_a|breakaway_max_a|current_rate_limit_a_per_s)"' \
  ~/ros2_ws/install/zlac_can_bringup/share/zlac_can_bringup/urdf/orange_go_2025.urdf

# 記録時も電流上限が8.0Aだったと確認できた場合にのみ実行
python3 tools/analyze_mimo_overspeed.py \
  ~/ros2_ws/mimo_results/mimo_ladrc_original.csv --limit-a 8.0

python3 tools/analyze_mimo_focus.py \
  ~/ros2_ws/mimo_results/mimo_ladrc_original.csv \
  --start 22.5 --end 24.3 --bin-s 0.1 --limit-a 8.0
```

解析器には、CSV中の電流指令の最大値が指定した `--limit-a` を超えていたらエラーで停止する検査を追加した。なお、逆に最大値が設定上限に達していない場合、上限設定値そのものをCSVから推定することはできない。ログ取得時の設定ファイルと照合が必要。

22.5–24.3秒は左右輪が全区間TRACKのため、TRACK/BREAKAWAY切替がこの連続飽和区間の主原因である可能性は低い。角速度偏差と左右指令電流差 `I_R-I_L` を確認する。左/右はROS正方向の電流単位である。符号不一致 `YAWbad` は「角速度偏差の補正方向と指令電流差が反対」という診断フラグに過ぎず、ESO外乱推定や摩擦を含む場合、直ちに制御ミスと判定しない。

## 追加：連続飽和区間を0.1秒ごとに解析

既存の解析で 22.634〜24.193 s に約1.56秒の連続飽和が見つかった場合は、次の追跡解析を実行する。ROSやCANを起動しないオフライン処理である。

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking
python3 -m unittest discover -s tests -p 'test_analyze_mimo_focus.py' -v
python3 tools/analyze_mimo_focus.py \
  ~/ros2_ws/mimo_results/mimo_ladrc_original.csv \
  --start 22.5 --end 24.3 --bin-s 0.1 --limit-a 8.0
```

全複合運動のTRACK可否による飽和率・車輪速度超過率を比較する。指定区間では0.1秒ごとに車体目標/実速度、左右指令/実電流、飽和率、TRACK率、車輪速度超過率、速度超過中に加速方向または制動方向の**指令**を出した率を表示し、対応するCSVを保存する。

「加速方向/制動方向」は電流指令と実際の車輪速度の符号から分類したもので、ドライバが発生した物理トルクの独立計測ではない。ブレーキ指令が出ていても電流フィードバックの反転に時間がかかる可能性がある。複合運動時のmimo_readyそのものは既存ログに含まれないため、TRACKの組合せだけでは正確な切替履歴は分からない。

## 2026-11-13：8 A上限での再解析とESO補償項の検証

実機ログの複合運動 26,171 サンプルに対し、8 A上限での飽和率は **0.076%**（20サンプル、171.753–171.848秒）。旧解析の44.5%は2 Aを閾値にした割合であり、飽和率ではなかった。

23.5–24.3秒の全サンプルは両輪TRACKであり、8 A飽和なし。例として23.9秒の区間平均値：
- 目標 `omega_ref=-0.60 rad/s`、ESO速度推定 `hat_omega=-0.78 rad/s`
- ESO総外乱推定 `hat_d_omega=+4.67 rad/s²`
- `wc_omega=5 rad/s`、`b0=18 rad/s²/A`、`wheel_radius=.064 m`、`wheel_separation=.354 m`
- 速度偏差項 `wc*(omega_ref-hat_omega)=+0.90 rad/s²`
- 外乱補償項 `-hat_d_omega=-4.67 rad/s²`
- `I_R-I_L=(0.90-4.67)/(18*.064/.354)=-1.1585 A`
- ログの最終左右指令電流差 `1.32-2.49=-1.17 A`

この一致はMIMO-LADRCの**計算とログに整合性がある**ことを示す。推定総外乱が物理的に正しいことは保証しない。また左・右の最終電流指令には上限、スルーレート、床値処理が存在するので、全区間で式が厳密に一致するとは限らない。

記録済みCSVからESO制御項を分離して検証する（ROS/SocketCANに触れず、実機を動かさない）。

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking
python3 -m unittest discover -s tests -p 'test_analyze_mimo_eso_balance.py' -v
python3 tools/analyze_mimo_eso_balance.py \
  ~/ros2_ws/mimo_results/mimo_ladrc_original.csv \
  --start 23.5 --end 24.3 --bin-s 0.1 \
  --wc-omega 5.0 --b0 18.0 \
  --wheel-radius 0.064 --wheel-separation 0.354
python3 tools/analyze_mimo_overspeed.py \
  ~/ros2_ws/mimo_results/mimo_ladrc_original.csv --limit-a 8.0
```

後者の解析結果には、最大並進速度偏差・最大角速度偏差を記録した時刻、およびその時刻の左右指令電流とTRACK状態も出力する。

**重要：** 観測された23.9秒の `I_R-I_L<0` は、推定総外乱による加速度 `+4.67 rad/s²` と相殺され、推定正味角加速度は `+0.90 rad/s²` になる。これは目標角速度との差を減らす方向であるため、負の電流差だけで「制御が逆向き」とは断定できない。ESO外乱の推定誤差・推定遅れは別途評価する。

## 追記：最大追従誤差の時刻（170.633秒・418.098秒）を再調査

実データの電流上限8 Aによる飽和率は0.076%に過ぎない。23.5～24.3秒の両輪TRACK区間では、ESO角速度外乱補償項がフィードバック項を100%のサンプルで上回るが、最終電流差の再構成平均絶対残差は0.0256 Aだった。制御計算がログと一致することと、推定外乱が現実と一致することは別の問題である。

複合指令の最大 |並進速度偏差|=1.548 m/s (170.633 s, 両輪非TRACK)、最大 |角速度偏差|=3.327 rad/s (418.098 s, 両輪非TRACK) は、いずれも制御器の MIMO 有効条件を満たしていない。これらは絶対誤差であり、正方向の「速度超過」と断定しないこと。

コードでは各輪の目標角速度の符号反転（非ゼロから反転）時にTRACKを解除し、BREAKAWAYへ戻す。BREAKAWAY中はESOをリセットし、符号方向への実角速度0.30 rad/s以上の連続観測がTRACK移行条件となる。したがって、車輪が逆方向に回っている状態での方向転換は長い非TRACK区間を作り得る。ただし、当該最大偏差に符号反転が関与したかは未確認。

未確認の理由を明示するため、専用の**再駆動不要な**分析を実行する：

```bash
cd ~/ros2_ws/src/zlac8015d_hardware
git pull --ff-only origin feat/mimo-ladrc-pid-straight-tracking
python3 -m unittest discover -s tests -p 'test_analyze_mimo_extremes.py' -v
python3 tools/analyze_mimo_extremes.py \
  ~/ros2_ws/mimo_results/mimo_ladrc_original.csv \
  --before 0.8 --after 1.2 --bin-s 0.1
```

出力に最大偏差の正負、元の車体v・ω目標と実測、左右の目標/実測車輪角速度、指令電流/6077h実電流、TRACK切替、目標角速度符号反転、ロックアウト状態を表示する。

注意：CSVには実際の `mimo_ready`、停止ランプ `normal_stop_ramp_active_`、レート制限前の要求電流、制御周期ごとのCiA402状態が記録されていない。これらをログから特定したと主張しない。さらにCSVの `ref_v_m_s` および `ref_omega_rad_s` はC++の `hw_commands_` から計算され、通常停止ランプ後の `control_ref_` と必ずしも一致しないため、停止遷移中の精密な再構成には注意を要する。
