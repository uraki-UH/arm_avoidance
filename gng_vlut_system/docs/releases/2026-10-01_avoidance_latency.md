# 2026-10-01 - 回避周期・隣接安全による復帰判定と未解決の回避性能

変更:

- 指令周期: 壁時計タイマーごとにシミュレーション時刻の次回期限をずらす処理から、絶対周期を維持する処理へ変更。遅延分のまとめ実行なし。
- 停止診断: 入力失効時の関節・点群・ボクセル・グラフ・native目標の経過時間を保存。停止ラッチ受信による元エラーの消去を撤去。
- 復帰判定: ユーザー指定のGNG隣接（辺で直接接続）を採用。実測関節に最も近いノードと隣接ノードがすべて安全であること。固定の開始姿勢余裕0.22 m条件を撤去。欠測ラベルは復帰不許可。
- 点群の扱い: 接近中の退避優先と復帰経路の停止余裕検査を維持。退避完了の離散化許容1 mm。復帰中の点群余裕減少だけを理由とした局所退避への差戻しを撤去。隣接危険の発生時は復帰中断。
- 比較試験: 合成障害物の高さに対応する水平姿勢を試験入力内で固定。製品設定の左肩−45°への別作業の変更は保持。

観測・検証:

- 稼働系の読取り: 入力失効による停止ラッチ、12秒間の指令0件、実時間比0.879。記録済みC++計画12件・採用0件・局所補正351件。どの入力が失効したかは旧診断から特定不能。[結果](../../../artifacts/avoidance_latency_20261001/live.json)。既存プロセスの再起動・停止解除なし。
- 単体: 最終コード174件成功。周期半減防止、元停止理由保持、実測関節によるノード選択、直接隣接だけの参照、衝突・危険・欠測時の復帰拒否、復帰中の点群余裕減少を含む検証。
- 周期修正のみ: 0.53 m / 8 s接近で回避・復帰・Viewer・自己ボクセル・欠測停止に成功、最大実測速度0.2008 rad/s。[結果](../../../artifacts/avoidance_latency_20261001/slow_input_absolute_horizontal/report.json)。これは隣接復帰変更前の成功。
- 速い接近: 0.53 m / 2 sでは周期修正前後とも停止距離に到達。修正前の指令間隔中央値は実時間99.2 ms・sim時刻84.0 ms、修正後は92.2 ms・52.5 ms。最大実測速度は0.1743 / 0.1354 rad/sで、速度改善の根拠なし。[修正前](../../../artifacts/avoidance_latency_20261001/fast_input/report.json)、[修正後](../../../artifacts/avoidance_latency_20261001/fast_input_absolute_horizontal/report.json)。実時間比・計算負荷は固定されていない比較。
- 隣接安全判定: 接近中もGNGノード83と隣接58が安全のまま。水平実姿勢とノード83の関節角差ノルム0.67064 rad。[ノード角度](../../../artifacts/avoidance_latency_20261001/nearest_node.json)。点群近接と代表姿勢のラベルの不一致を確認、VLUT全体の誤りと断定する根拠なし。
- 最終隣接復帰方式: 0.53 m / 8 sでも停止距離到達。局所補正42件・C++計画0件。回避と復帰の通し成功は未達。[結果](../../../artifacts/avoidance_latency_20261001/slow_neighbors_guarded/report.json)。単体成功との区別。
- 補間比較: 位置線形補間・最大変位0.06 radでも高速接近に失敗、最大実測0.1350 rad/s。試験変更は取り下げ、既存5次補間を保持。[結果](../../../artifacts/avoidance_latency_20261001/fast_linear_guarded/report.json)。
- 途中試験: `fast_input_absolute`は製品姿勢−45°と試験の水平復帰条件の不一致で期限超過。`fast_linear_neighbors`は点群退避優先なしでは監視に留まり停止距離到達。両条件を最終試験から除去。
- 終了状態: 全所有ROS診断・Gazebo試験の終了、各reportのcleanup成功・残留なし。実機出力OFF。稼働launchへの変更反映は未実施。

所有プロセスの起動コマンド（コンテナ内、ROSとworkspaceをsource後、すべて終了済み）:

```bash
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1 timeout -k 3 20 python3 /ros2_ws/src/artifacts/avoidance_latency_20261001/probe.py
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1
python3 /ros2_ws/src/gng_vlut_system/test/check_viewer_environment_gazebo.py --enable-left-forward --approach-sec 8 --withdraw-sec 4 --output /ros2_ws/src/artifacts/avoidance_latency_20261001/slow_neighbors_guarded
```

ほかの試験も同じchecker・共通引数。`--approach-sec 2`の出力先は`fast_input`、`fast_input_absolute`、`fast_input_absolute_horizontal`、`fast_linear_neighbors`、`fast_linear_guarded`。`--approach-sec 8`の周期修正のみの出力先は`slow_input_absolute_horizontal`。各出力先の`command.json`に内部launchコマンドを保存。

## 続報: 静止環境での退避・停止

- 観測: 当初の停止記録は点群余裕33.6 mm、局所補正81件、最大関節変位0.1683 rad。停止後12秒間は指令0件。[読取り結果](../../../artifacts/avoidance_live_response_20261001/live.json)。停止瞬間の部位・点群位置は旧実装で保存なし。
- 再計測: 環境入力の再起動・変更はユーザー確認済み。復帰後12秒・180ボクセルメッセージで、設定上の開始姿勢の左指先余裕64.9〜73.7 mm。最小時の点群位置はベース基準(0.33, 0.15, 0.07) m。静止物の物体種別・停止時の点群との同一性は未特定。[入力再起動中](../../../artifacts/static_environment_stop_20261001/input_restart.json)、[復帰後](../../../artifacts/static_environment_stop_20261001/live.json)。
- 挙動: 点群退避目標180 mmに届かない開始姿勢では、障害物を動かさなくても退避対象。再計測の稼働系は回避中、計測期間中の姿勢での余裕86.7〜181.0 mm。今回の修正反映後の実動作成功を示す測定ではないこと。
- 修正: 計画側15 mm指定でも、経路検査の下限を実行監視側35 mmで制限。より大きい指定値は維持。停止時のリンク・点群位置・距離・関節姿勢を`avoidance/status.stop_clearance`へ保持し、状態行の理由へリンクと距離を追加。通常端末の行数は維持。
- ユーザー指定の設定変更: 前方伸展設定の`target_clearance`を180 mmから50 mmへ変更。停止距離35 mmとGNG隣接安全・復帰経路検査は維持。上記180 mmでの実測とは区別。
- 検証: 関連167件成功。安全な両端の間にある20 mmの区間の拒否、40 mmの区間の通過、厳しい50 mm指定の保持、停止後の点群移動による診断上書き防止。退避目標変更後の設定・launch関連29件も成功。実環境での停止再現・物体特定・修正後Gazebo通し動作は未検証。
- 終了: 所有診断2回・単体試験の終了と残留なしを確認。既存launch・実機・停止解除への操作なし。インストール先はソースへのリンク、次回の回避ノード起動から反映。

所有プロセスの起動コマンド（コンテナ内、ROSとworkspaceをsource後、終了済み）:

```bash
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1 timeout -k 3 25 python3 /ros2_ws/src/artifacts/static_environment_stop_20261001/probe.py
cd /ros2_ws/src/gng_vlut_system
PYTHONDONTWRITEBYTECODE=1 timeout -k 3 60 python3 -m pytest -q test/test_gng_lidar_path.py test/test_dual_arm_limits.py test/test_avoidance_timing.py test/test_viewer_environment.py test/test_local_qp.py test/test_dual_arm_control.py test/test_dynamixel_sim_keyboard.py
PYTHONDONTWRITEBYTECODE=1 timeout -k 3 60 python3 -m pytest -q test/test_pointcloud_avoidance.py test/test_topodualarm_launch.py
```

## 続報: 実物付近に物がない場合の距離表示の照合

- ユーザー確認: 「点群がない」は実物のロボット付近に物がない状態。Viewer非表示の意味ではないこと。
- 読取り: 自己除去後1,025〜1,080ボクセル、最接近部位`L_gripper_base`、ボクセル中心(0.25, 0.03, 0.21) m、計算余裕約−2.7 mm。過去の14.0 mm拒否時点とは別の計測。[結果](../../../artifacts/absent_cloud_stop_20261001/live.json)。
- 元点群照合: 後続計測で同一stampのRealSense元点群→自己除去後ボクセルを確認。中心(0.23, −0.03, 0.23) mのセル内に元点群1点、当該点から10 mm以内に94点。孤立した単一点とは言えないこと。発行元の重複なし、自己マスク最近傍まで182 mm。[結果](../../../artifacts/absent_cloud_stop_20261001/trace.json)。Gazebo終了後のため姿勢は直前の記録値、点群は後続の実測。
- 姿勢差: 計測時の実機左肩約＋4°、Gazebo記録約−45°。距離判定の対象はGazebo姿勢。自己除去の対象は実機姿勢。
- 形状比較: 同一ボクセル中心とGazebo記録姿勢のURDF三角形表面距離95.0 mmに対し、外接球半径44.7 mm・セル半径17.3 mmを差し引く従来の計算余裕48.8 mm。実物表面間の距離と表示値が異なる要因を確認。[保存点群の計算](../../../artifacts/absent_cloud_stop_20261001/analysis.json)。実際の物体種別・取付較正の正しさ・過去14.0 mm拒否時の対応点は未特定。
- 修正・検証: 開始拒否時も部位・座標・球半径・関節姿勢を保存し、拒否理由に部位を追加。関連133件成功。計画形状・停止距離の変更なし。
- 初回失敗: 読取り中のGazebo終了による実測姿勢欠測。記録済み姿勢と明記した再計測で照合。三角形計算のリンク添字誤りは修正後に再計算。所有診断・試験はすべて終了、既存ノードへの停止・開始指令なし。

所有プロセスの起動コマンド（コンテナ内、ROSとworkspaceをsource後、終了済み）:

```bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1
timeout -k 3 25 python3 /ros2_ws/src/artifacts/absent_cloud_stop_20261001/probe.py
timeout -k 3 25 python3 /ros2_ws/src/artifacts/absent_cloud_stop_20261001/trace.py
timeout -k 3 25 python3 /ros2_ws/src/artifacts/absent_cloud_stop_20261001/analyze.py
cd /ros2_ws/src/gng_vlut_system
timeout -k 3 60 python3 -m pytest -q test/test_dual_arm_limits.py test/test_avoidance_timing.py test/test_gng_lidar_path.py test/test_dual_arm_control.py test/test_dynamixel_sim_keyboard.py
```
