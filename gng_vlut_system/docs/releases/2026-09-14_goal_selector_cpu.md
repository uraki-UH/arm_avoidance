# 2026-09-14 - 計画目標選択のCPU負荷削減

## Summary

Python目標選択ノードの1コア張り付きを実測し、C++へ置換。実入力6秒のCPU測定は103.27%から12.17%へ低下、最終版の再測定は11.33%。100%は1コア相当。

## Changed

- ROSメッセージの復号・保持をC++へ移管。受信時は最新スナップショットのみ保持し、既存`goal_update_hz`の周期へ計算を集約。
- 到達セルに属する計画GNGの添字を直接参照。候補ごとの全ノード走査と中間mapのdeepcopyを除去。
- 座標系ペア単位で最新TFを1回取得し、上位候補だけを部分ソート。選択結果のノードコピーは最後の1回。
- 可操作性重みが0の場合は特徴量topicの購読なし。

## Added

- `topological_map_goal_selector_node`実行ファイルと、ROS実行に依存しない`goal_node_selection.hpp`。
- 既存6ケースを移管し、順位・条件数補正・姿勢・無効グリッドを追加したC++回帰テスト9件。

## Fixed

- 約10,000ノードのmap・特徴量配列をPythonオブジェクトへ繰り返し展開する高負荷。修正前のprofileでsubscriptionの取出しが6.32秒中5.16秒、約82%を占有。
- 入力コールバックとタイマーの重複による目標更新頻度の超過。

## Removed

- Python版選択ノードと直接呼出しテスト。二重実装の保持なし。
- Pythonファイルを直接起動する旧CLI。直接起動は`ros2 run gng_vlut_system topological_map_goal_selector_node --ros-args -p ...`へ変更。

## Behavior Impact

- 評価式、候補ごとの採択数、INSIDEだけの選択、衝突ラベル除外、TF追従、選択IDの重複排除は維持。
- 受信直後の計算を周期処理へ変更。通常時の反映待ちは既定5Hzの1周期分。空候補・全領域外・TF未取得も次の周期で旧目標を失効。
- `grasp_joint_candidates.launch.py`からC++版を起動。既存の稼働中Python版は自動停止しないため、反映にはlaunchの再起動が必要。

## Topics / Params / Messages

- ROSメッセージ形式・launch引数の追加削除なし。選定mapのQoSをreliable・transient local・depth 1へ変更し、目標IDと同じ最新スナップショット方式へ統一。
- 同時進行の[tmap短縮](2026-09-14_tmap_topics.md)を維持。新ノードの既定入力`/ToPoDualArm/tmap_static`、出力`/selected_tmap`。稼働中の旧名topicによる比較では引数で入力名を明示。
- 現行仕様は[TECHNICAL_SPEC](../TECHNICAL_SPEC.md)を参照。

## Verification

- Docker Releaseビルドに成功。初回の定数所属・整数型不一致を修正後、再ビルド・最終ビルドとも成功。
- C++選択テスト9件と既存候補評価テストに成功。
- 実入力10,801ノード・8候補（INSIDE 2、OUTSIDE 6）をdomain 227で再生。Python版と選定mapの全フィールド・ID配列が一致。選択IDは`654, 3425, 3485, 4105, 5548, 10038`。
- 同スナップショットで候補再送なしのTF移動・復帰、空候補による失効・復帰を確認。
- 実入力6秒のCPU使用率はPython版103.27%、C++版12.17%、出力5.00Hz。最終QoS版は11.33%、4.83Hz。入力は逐次変化するため、厳密な同時A/B比較ではない。
- 修正前の同一入力の選択計算だけのCPU時間は29.59〜33.20ms/更新（5回）。Pythonメッセージ復号の時間は含めず、全体CPU値と区別。
- domain 228で既存`check_grasp_joint_candidates_integration.py`相当の結合テストに成功。短縮名の選定map、候補経路・評価・ロボットプレビュー、静止時の再探索抑制、明示要求・関節変更時の更新、全領域外・空入力の失効、関節指令なしを確認。隔離用domainだけ218から228へ置換した一時スクリプトを使用。
- 計測・再生・launchの全プロセス終了と既存ROSのPID・コンテナ状態の維持を確認。新規ROS daemonの残留なし。

ビルド・回帰テスト（すべて終了済み）:

```bash
docker exec gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
colcon build --packages-select gng_vlut_system --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
'
docker exec gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ctest --test-dir /ros2_ws/build/gng_vlut_system -R "test_candidate_metric_availability|test_grasp_candidate_reachability" --output-on-failure
'
```

計測・ROS検証の起動コマンド（一時スクリプトと取得データは終了後削除）:

```bash
docker exec -i gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
timeout --signal=INT --kill-after=10s 60s python3 -u -
' < /tmp/codex-goal-selector-probe.py
docker exec -i gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
timeout --signal=INT --kill-after=10s 35s python3 -u -
' < /tmp/codex-goal-selector-live.py
docker exec -i gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 SELECTOR_CAPTURED=1 timeout --signal=INT --kill-after=15s 75s python3 -u -
' < /tmp/codex-goal-selector-cpp-probe.py
docker exec -i gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
timeout --signal=INT --kill-after=15s 40s python3 -u -
' < /tmp/codex-goal-selector-cpp-probe.py
docker exec -i gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15s 180s python3 -u -
' < /tmp/codex-selector-integration.py
```

C++計測用子プロセス（停止済み、実入力時は既存domain、再生時はdomain 227）:

```bash
/ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/topological_map_goal_selector_node --ros-args -r __node:=codex_goal_selector_probe -p topological_map_topic:=/ToPoDualArm/topological_map_static -p output_topic:=/codex_selector_probe/selected -p goal_candidate_ids_topic:=/codex_selector_probe/goals -p orientation_weight:=0.0 -p manipulability_weight:=0.25
```

結合テスト用子プロセス（domain 228、すべて停止済み）:

```bash
ros2 run gng_vlut_system safety_monitor_node --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml -p gng_model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin -p vlut_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vlut.bin -p base_frame:=ToPoDualArm/base_link -r topological_map:=/ToPoDualArm/tmap_static
ros2 launch gng_vlut_system grasp_joint_candidates.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

## Risk / Notes

- profile・再計測でFast DDSの既存SHMポートのロック警告を観測。実受信・検証は成功し、共有ポートの削除や既存プロセスの停止なし。
- ブラウザやデータセット再生のCPU負荷は今回の対象外。実画面・実機把持動作の検証なし。
- 長時間・全入力条件のp95や候補数依存性の計測は未実施。[既存の詳細計測タスク](../TASK_LIST.md)は対象ファイル名だけ更新し、完了扱いにしない。
