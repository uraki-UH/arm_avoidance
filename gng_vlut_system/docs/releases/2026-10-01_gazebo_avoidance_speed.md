# 2026-10-01 - 前方伸展時の回避速度と点群ROI

変更:

- 前方伸展用Gazebo設定: 速度上限0.45 → 1.2 rad/s、制御周期0.15 → 0.05 s。旧ゼロ姿勢設定の速度は保持。
- 実点群ROI: ToPoDualArmの`environment_voxelization.reachability_margin_x`を0.1 → 0.4 mへ変更。保存済みTmapでの前方端0.528 → 0.828 m。左右・上下の余白は従来値。
- 合成障害物: 前方伸展設定の`hand_far_x`を0.65 → 0.85 mへ変更。離れたときの復帰条件を満たす初期配置。実点群の配置には非適用。
- 計画元: 既存`topological_map_avoidance_node`のC++計画。Gazebo関節状態と実環境Tmapから退避目標を生成し、Python側で速度・区間余裕・失効・停止ラッチを検査。危険時の安全候補を関節距離順32個へ限定し、`enable_safety_penalty: false`で候補間の探索結果を共有。衝突・危険ノードへの進入禁止は有効。既存ノード単独起動の既定動作は保持。
- 状態更新: 初回Tmap取得後、時刻・トポロジー照合値付き`TopologicalNodeStates`で安全ラベルを受信。毎回のPython完全Tmap復元を回避。10,801ノード・1,296,948 byteの復元時間は3標本で147.5〜161.9 ms。[測定](../../../artifacts/gazebo_avoidance_speed_20261001/graph_decode.json)。
- 反映: 通常Viewerの環境入力launchとGazebo launchの再起動が必要。速度上限は常時速度の指定ではなく、停止距離・入力失効・実機側制限の緩和なし。[操作仕様](../dynamixel_sim_control.md)。

調査・試験:

- 既存実入力の10秒読取り: Gazebo実時間比0.945、停止理由「デモ停止距離に到達」、記録済み最小余裕0.025722 m、実機出力OFF。[読取り結果](../../../artifacts/gazebo_avoidance_speed_20261001/live_before.json)。読取りノード終了済み。
- 高速設定の開始・停止試験: 前方伸展から左腕0.100499 rad変位、Space実測停止・B解除成功。ゆっくりした合成接近のため、この試験だけでは速度改善の根拠なし。[結果](../../../artifacts/gazebo_avoidance_speed_20261001/fast_start/report.json)。所有試験停止済み。
- 初回比較: 障害物がROI外で環境点群が空となり、両設定とも開始待ちで期限超過。予測180 s、実測185.71 s。[比較記録](../../../artifacts/gazebo_avoidance_speed_20261001/comparison/report.json)。
- 床点群追加後: 旧ROIの前方端から障害物が現れた時点で近接し、両設定とも停止距離に到達。予測140 s、実測30.14 s。速度設定だけでは解消せず、ROI拡大を追加。[比較記録](../../../artifacts/gazebo_avoidance_speed_20261001/comparison_with_floor/report.json)。両比較の全試験プロセス終了・既存ノード維持。
- ROI拡大のみ: 旧速度・新速度とも停止距離に到達。[結果](../../../artifacts/gazebo_avoidance_speed_20261001/comparison_expanded_roi/report.json)。
- C++接続時の失敗: 空の目標トピックによる全ノード探索、退避時の候補別探索による目標失効。目標トピック分離・候補数制限・共有探索へ修正。初期失敗時はlaunchのSIGKILLで所有ノード終了、直後のROS探索情報残留により復元未確認の記録あり。[起動失敗](../../../artifacts/gazebo_avoidance_speed_20261001/comparison_native/report.json)、[目標失効](../../../artifacts/gazebo_avoidance_speed_20261001/native_without_qp/report.json)。
- QP併用試験: `solved inaccurate`で停止。[結果](../../../artifacts/gazebo_avoidance_speed_20261001/native_bounded/report.json)。QPは別セッションでYAML有効化・既定OFFへ変更とのユーザー確認。最終通し試験はQP無効、QP実装への変更なし。
- 最終通し試験: 成功、69.16 s。同じ0.53 m / 8 s接近条件で最大関節変位0.6293 rad、最大実測関節速度0.2111 rad/s、最小点群余裕0.07772 m。C++計画7回、局所退避41回。Viewer実測一致1,124件、Gazebo自己ボクセル差分1,029セル、実機マスク差分0セル。実機関節入力欠測・点群欠測の停止と復帰操作成功。[結果](../../../artifacts/gazebo_avoidance_speed_20261001/native_shared_retry/report.json)。合成点群による検証、実RealSense接近速度への保証なし。上限1.2 rad/s到達の確認なし。
- 検証: active側`ais_gng_msgs`と`gng_vlut_system`のReleaseビルド成功。初回はCOLCON_IGNORE対象の`MSG/ais_gng_msgs`へ追加したため失敗し、active側へ修正。関連単体123件成功、最終診断追加後の入力検証・QP既存試験29件成功。全所有試験停止済み、実機指令なし。

比較コマンド（コンテナ内、source済み）:

```bash
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1
python3 /ros2_ws/src/skills/run-benchmark-batch/scripts/run_batch.py \
  /ros2_ws/src/artifacts/gazebo_avoidance_speed_20261001/cases.json \
  --output /ros2_ws/src/artifacts/gazebo_avoidance_speed_20261001/comparison_expanded_roi \
  --repeats 1 --timeout-sec 210 --max-total-sec 450 --estimate-sec 65 --continue-on-error
```

比較条件: 旧速度と新速度の各1試行、前方伸展姿勢、同一点群の0.53 m / 8 s接近・3 s保持・8 s離脱。実点群パイプラインを通す試験生成点群、実機出力OFF。試験ごとの起動コマンドは比較出力の`trial/command.json`。

最終通し試験の起動コマンド（終了済み）:

```bash
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1
python3 /ros2_ws/src/gng_vlut_system/test/check_viewer_environment_gazebo.py \
  --enable-left-forward --approach-sec 8 --withdraw-sec 8 --no-enable-local-qp \
  --output /ros2_ws/src/artifacts/gazebo_avoidance_speed_20261001/native_shared_retry
```

診断試験: 同じcheckerの出力先`native_diagnosis`・`native_trace`・`native_bounded`（QP無効引数なし）、`native_without_qp`・`native_shared`（QP無効引数あり）。`native_shared`は別試験とのdomain競合を検出し起動前終了、既存プロセスへの操作なし。単独C++診断はdomain97で`timeout -k 3 8 ros2 run gng_vlut_system topological_map_avoidance_node --ros-args -r __ns:=/sim_ToPoDualArm --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml --log-level debug`、ほかに名前空間なし・sim時計ありの切分け。全診断終了済み。

## 続報: トルク設定と物理ソルバーの追従比較

- 調査: 前方伸展用設定での`world`切替を試験。追従比較後の統合試験で数値異常が発生し、製品設定への採用は見送り。最終差分にソルバー・トルク・速度上限・ゲインの変更なし。
- 比較条件: 左肩−45°から−0.24 radの指令、同一モデル・位置ゲイン60・URDF上限6 N·mの95%（5.7 N·m）。単一区間は0.375 sの5次補間、再計画相当は50 msごとに実測起点から最大−0.032 radの静止間補間を再発行。各方式1試行、シミュレーション時間基準。

| 指標 | quick | world |
| --- | ---: | ---: |
| 単一区間の最大実測速度 [rad/s] | 0.1268 | 1.1828 |
| 単一区間の最大追従誤差 [rad] | 0.22169 | 0.01749 |
| 単一区間の90%到達時間 [s] | 1.2 s以内に未到達 | 0.302 |
| 50 ms再発行の最大実測速度 [rad/s] | 0.1334 | 0.8347 |
| 50 ms再発行の90%到達時間 [s] | 1.2 s以内に未到達 | 0.483 |

- 評価: 同一トルク設定で追従改善。今回の大きな遅れの主要因は`quick`方式での物理追従。再発行時の静止間補間・退避量による速度低下は別要因。実機トルク不足の判定、実RealSense入力の回避速度保証は対象外。
- 根拠: [quick](../../../artifacts/avoidance_motor_tracking_20261001/comparison_quick/report.json)、[world](../../../artifacts/avoidance_motor_tracking_20261001/comparison_world/report.json)。各`result: passed`は測定・後片付けの完了であり、追従合格の意味ではない。`world`単関節ログにもLCP内部エラー77件、数値安定性の合格根拠なし。所有プロセス・ROSノード残留なし。
- 実入力読取り: 18秒間は`monitoring`、左肩指令最大0.02813 rad/sに対して実測0.02817 rad/s。小さい指令だけでは回避時の追従評価ができないため上記隔離試験を追加。[記録](../../../artifacts/avoidance_motor_tracking_20261001/report.json)。
- 統合試験: `world`で水平姿勢−90°から点群保持・自動再開を試験。点群待機への移行前にODE `LCP internal error, s <= 0`と関節速度超過が発生し、停止ラッチ。最大関節速度22.84 rad/s、`obstacle_wait`待機で期限超過。単関節試験の成功だけでは採用不可。[失敗記録](../../../artifacts/avoidance_motor_tracking_20261001/world_auto_resume/report.json)。所有試験停止・ROSノード残留なし、既存launchと実機への操作なし。

起動コマンド（コンテナ内、ROS環境source済み、いずれも終了済み）:

```bash
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1 \
  timeout -k 3 30 python3 /ros2_ws/src/artifacts/avoidance_motor_tracking_20261001/probe.py
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1
MOTOR_TEST_SOLVER=quick timeout -s INT -k 30 110 python3 /ros2_ws/src/artifacts/avoidance_motor_tracking_20261001/compare.py
MOTOR_TEST_SOLVER=world timeout -s INT -k 30 110 python3 /ros2_ws/src/artifacts/avoidance_motor_tracking_20261001/compare.py
timeout -s INT -k 30 180 python3 /ros2_ws/src/gng_vlut_system/test/check_obstacle_auto_resume.py \
  --output /ros2_ws/src/artifacts/avoidance_motor_tracking_20261001/world_auto_resume
```

予備比較の出力先: `comparison`、`comparison_reference`。同じスクリプトの比較条件整理前、共通avoidance設定の付与なし。前者は終了時の非実行デモノードに`rclpy`終了例外、両方とも所有プロセス残留なし。表は条件統一後の`comparison_quick`と`comparison_world`のみ。
