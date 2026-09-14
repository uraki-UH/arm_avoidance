# 2026-09-14 - 把持対象ノードのTopologicalMap配信

## Summary

把持対象ノードを`/grasp_pose_cands/Tmap`へ配信。同じ候補内の元GNGエッジだけを収録。

## Changed

- 1候補を1clusterへ対応。平面・付属非平面のノードと元エッジを保存し、短期欠測時も一緒に保持。
- ノードIDとedges添字は出力内で再採番。候補が重なる場合は候補ごとに独立したノードを保持し、候補間接続を防止。
- 到達性更新はsemantic_labelのみ反映。元環境labelと保存済み幾何を維持。

## Added

- 既存TopologicalMapの定数`SEMANTIC_GRASP_UNKNOWN=2`、`SEMANTIC_GRASP_INSIDE=3`、`SEMANTIC_GRASP_OUTSIDE=4`。メッセージフィールド追加なし。
- Viewerの共通ラベル設定に把持候補の到達性を追加。

## Fixed

- HANDLE判定をsemantic_label=1へ限定し、他の意味ラベルとの混同を防止。

## Removed

- `/grasp_pose_cands/nodes`のMarkerArray配信と`candidate_nodes_topic`。

## Behavior Impact

- Viewerでは`/grasp_pose_cands/Tmap`をON。既存clusterのホバー枠・詳細表示経路を使用。
- 同トピックのViewer既定値はノードサイズ0.008、エッジ表示OFF、ノード不透明度0.5。他トピックや手動変更済みの設定は上書きせず、再読み込み後の初期設定へ適用。
- 空候補・TF欠落では空TopologicalMapを配信。遅延購読はtransient_local/depth 1で最新集合を取得。
- 稼働中の旧版は自動再起動しない。新形式を使うには上方候補launchを再起動し、Viewerを再読み込み。

## Topics / Params / Messages

- `candidate_graph_topic`: 既定は`candidate_topic + "/Tmap"`。
- `/grasp_pose_cands`のGraspCandidateArrayは維持。`cluster.id`は同配列の候補ID。
- 出力型は`ais_gng_msgs/msg/TopologicalMap`。edgesは添字、cluster.nodesは出力内ノードID。
- 点サイズはViewer設定。`candidate_node_diameter`は非平面領域の試験Marker専用。

## Verification

Docker `gng_cpu_container`、`/ros2_ws`でReleaseビルド成功。既存ノードの停止・再起動なし。

```bash
colcon build --packages-select ais_gng_msgs grasping_system topo_fuzzy_viewer --symlink-install --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_BUILD_PARALLEL_LEVEL=2
/ros2_ws/build/grasping_system/test_top_grasp_surface_estimator
/ros2_ws/build/topo_fuzzy_viewer/test_graph_inspection
ROS_DOMAIN_ID=117 ROS2CLI_NO_DAEMON=1 python3 /ros2_ws/src/grasping_system/test/check_top_grasp_topic_integration.py
```

- C++把持推定・グラフ抽出とViewer詳細抽出7件に成功。
- ROS結合3ケースに成功。既定・上書き出力先、座標変換、候補内エッジ、候補間エッジ除外、候補ID別状態、遅延購読、空配信を確認。
- 結合テストによる`ros2 launch grasping_system top_grasp_pose_candidates.launch.py params_file:=<一時YAML>`の起動3回。個別指定ケースはcandidate_topic/candidate_graph_topic/summary_topicも指定。全launch・子ノードを停止済み。
- frontendで`npm run lint`、`npm run build -- --configLoader runner --outDir /tmp/codex-candidate-tmap-dist`、`node tests/boundary_evidence.test.mjs`、`npm run test:hover`に成功。
- 表示既定値変更後に`node tests/boundary_evidence.test.mjs`と`./node_modules/.bin/tsc -p tsconfig.app.json --noEmit --incremental false`を再実行して成功。static/dynamic両方の既定値・他トピック維持・手動設定優先を確認。コマンドは終了済み、ROS起動なし。
- 検証後、既存ROSのPIDとコンテナ状態の維持を確認。ROSデーモンの新規残留なし。

## Risk / Notes

- 旧`/nodes`購読設定・独自YAMLは新topic/paramへ変更が必要。
- 元環境ノードIDとの直接対応には使えない。出力IDはuint16のため全候補合計65536ノードが上限で、超過は明示的エラー。
- 候補TCPの位置到達性の表示であり、衝突回避や把持成功の保証なし。
- 実画面のGPU描画・マウス操作、大規模実入力のCPU負荷は未検証。
