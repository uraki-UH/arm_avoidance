# 2026-10-01 - 回避後の復帰・再監視

- 対象: ToPoDualArm前方伸展設定のC++計画接続。A開始時の実測姿勢を復帰先として保持。
- 動作: 点群接近時の`avoiding`、復帰先・経路が塞がれた場合の`waiting_for_clearance`、経路の安全確認後の`returning`、開始姿勢到達後の`monitoring`。再接近時は再び回避。停止ラッチ中の自動復帰なし。
- 判定設定: 退避の点群余裕0.18 m、復帰先の点群余裕0.22 m、開始姿勢への関節誤差0.015 rad。現在姿勢の安全確認後も、復帰経路の停止余裕検査を継続。
- 初回失敗: 退避余裕0.10 m・復帰余裕0.12 mでは0.53 m / 8 sの合成接近に追従できず、停止距離に到達。上記余裕へ変更。停止距離・失効・速度制限の緩和なし。[初回結果](../../../artifacts/gng_base_frame_20261001/avoid_return/report.json)。
- 単体: 回避→安全待ち→復帰→監視→再接近を含む関連124件成功。
- 通し試験: 成功、84.17 s。全4動作状態を通過して`monitoring`へ復帰。最大関節変位0.56545 rad、最大実測速度0.22412 rad/s、最小点群余裕0.10563 m。Viewer姿勢・自己ボクセル追従、実機関節入力・点群入力の欠測停止成功。QPは既定OFF。全所有試験停止済み。[結果](../../../artifacts/gng_base_frame_20261001/avoid_return_early/report.json)。
- 反映条件: Gazebo launchの再起動。稼働中のユーザープロセスへの停止・再起動操作なし。

通し試験コマンド（コンテナ内・source済み、終了済み）:

```bash
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
python3 /ros2_ws/src/gng_vlut_system/test/check_viewer_environment_gazebo.py \
  --enable-left-forward --approach-sec 8 --withdraw-sec 8 \
  --output /ros2_ws/src/artifacts/gng_base_frame_20261001/avoid_return_early
```


## 続報: 退避先の直接隣接安全

- 要求・修正: 退避先自身と一次隣接に危険・衝突ノードがない候補だけを選択。C++の自律退避は候補数制限より先に安全条件を適用、Pythonグラフ探索でも終点の同条件を追加。二次隣接は対象外。
- 継続条件: 現在ノード自身が安全でも隣接危険があれば退避開始。退避中の現在位置の隣接危険だけによる反復再計画を抑止し、退避先の隣接悪化時は再選定。経路途中のノード自身の危険・衝突は引き続き禁止。
- Python実行: 点群余裕の達成だけによる`waiting_for_clearance`への遷移を廃止。最寄りノードと直接隣接の安全も必要。距離による対象腕選択が空でも隣接危険があれば計画対象関節で退避継続。局所補正は中間指令用、点群・関節・内部干渉・停止ラッチの検査は維持。
- 検証: C++13件、Python41件成功。隣接衝突候補の除外、二次隣接の許可、中間ノードを経由した安全終点への経路、ラベル変更後の終点不許可、点群余裕達成後の退避継続を確認。Release回避コンポーネントとテストのビルド成功、インストール先の共有ライブラリ更新確認。
- 初回試験: 協調試験の欠測隣接と、作業中に0へ手動変更されたマージンを取り込む試験設定で2件失敗。協調試験へ正しい空隣接を明示、共通設定をコピーする試験は有効な固定マージンへ独立化。ユーザーの実設定への上書きなし。
- 反映・制限: Gazebo launch再起動後の反映。今回のGazebo通し動作・実RealSense接近は未検証。ROSノードの新規起動・既存launchへの操作・実機指令なし。ビルド・テストは終了済み。

検証コマンド（コンテナ内、ROSとworkspaceをsource後、終了済み）:

```bash
cmake --build /ros2_ws/build/gng_vlut_system --target topological_map_planning topological_map_avoidance_node test_candidate_metric_availability test_goal_tasks -j 2
/ros2_ws/build/gng_vlut_system/test_candidate_metric_availability
/ros2_ws/build/gng_vlut_system/test_goal_tasks
cd /ros2_ws/src/gng_vlut_system
PYTHONDONTWRITEBYTECODE=1 timeout -k 5 90 python3 -m pytest -q -p no:cacheprovider \
  test/test_gng_lidar_path.py test/test_viewer_environment.py test/test_obstacle_auto_resume.py
```

## 続報: 隣接危険による退避と距離判定の分離

- 原因: 隣接危険を退避開始条件へ追加した後も、Python出力側に点群余裕増加の必須条件が残留。距離による対象腕選択もGNGの関節目標を部分的に変更。Python単独計画では距離だけによる保持・復帰も残留。
- 修正: 隣接安全が未成立の場合、計画対象関節全体で退避目標を追従し、各ステップの余裕増加条件を除外。Python単独計画の保持・復帰にも隣接安全を追加、非同期計画結果の終点隣接を受信時に再検査。経路の最低余裕・内部干渉・関節制限は維持。
- 検証: 修正前、隣接ラベル危険／衝突と点群余裕70／400 mmの4条件で指令停止・対象関節欠落を再現。修正後は関連Python70件成功。距離一定の退避指令、距離十分時のPython経路選択、経路余裕不足時の指令拒否を確認。既存試験1件の距離補正期待値を、新しい隣接条件別の期待値へ更新。SciPyとNumPyのバージョン警告・非推奨警告3件あり。
- 反映・制限: Gazebo launch再起動後の反映。今回のGazebo実動作・RealSense接近は未検証。ROSノードの新規起動・既存launchへの操作・実機指令なし。全検証プロセス終了済み。

検証コマンド（`gng_cpu_container`内、ROSとworkspaceをsource後、終了済み）:

```bash
PYTHONDONTWRITEBYTECODE=1 timeout -k 5 60 python3 -m pytest -q -p no:cacheprovider \
  /ros2_ws/src/gng_vlut_system/test/test_gng_lidar_path.py \
  /ros2_ws/src/gng_vlut_system/test/test_viewer_environment.py \
  /ros2_ws/src/gng_vlut_system/test/test_obstacle_auto_resume.py \
  /ros2_ws/src/gng_vlut_system/test/test_local_qp.py
```

## 続報: 回避・復帰のチャタリング抑制

- 変更: 共通YAMLへ`return_clear_sec: 0.5`を追加。復帰条件の継続成立まで保持、条件の悪化時は計測初期化。隣接危険・衝突・欠測や点群接近に対する回避の開始遅延なし。C++計画接続・Python単独計画の両方へ適用。[現行仕様](../pointcloud_avoidance.md)。
- 検証: Python117件成功。安全時間の途中のラベル・距離・経路変化、復帰中の即時再回避、確認間隔の中断、再開始・停止の初期化、設定0と不正値を確認。既存SciPy／NumPy警告3件。
- 初回試験: 試験fixtureのタプル参照誤り5件と、既存の即時復帰期待値1件で失敗。fixture参照を修正、対象腕選択だけの既存試験では待機時間0を明示。新しい継続時間の検証は別試験で実施。
- 反映・制限: インストール先のソース参照確認、反映はGazebo launch再起動後。Gazebo実動作は未検証。既存ROSノード・実機への操作なし。所有テストと子プロセスは全終了済み。

検証コマンド（`gng_cpu_container`内、ROSとworkspaceをsource後、終了済み）:

```bash
PYTHONDONTWRITEBYTECODE=1 timeout -k 5 60 python3 -m pytest -q -p no:cacheprovider \
  /ros2_ws/src/gng_vlut_system/test/test_avoidance_chattering.py \
  /ros2_ws/src/gng_vlut_system/test/test_gng_lidar_path.py \
  /ros2_ws/src/gng_vlut_system/test/test_viewer_environment.py \
  /ros2_ws/src/gng_vlut_system/test/test_obstacle_auto_resume.py \
  /ros2_ws/src/gng_vlut_system/test/test_local_qp.py \
  /ros2_ws/src/gng_vlut_system/test/test_pointcloud_avoidance.py
```
