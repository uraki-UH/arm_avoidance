# 2026-09-11 - IDと到達性状態を持つ単一候補配信

## Summary

`/grasp_pose_cands`を候補ID・姿勢・形状スコア・到達性状態の単一配列へ統一。
候補生成ノードが唯一の配信元となり、Viewerは同じIDの矢印の色だけを更新。

## Changed

- 上方把持・ボクセル把持・点群候補生成・ダミー候補の出力を`GraspCandidateArray`へ変更。
- 位置到達性を生成側の共通publisherへ移動。登録TCPセルと最新TFで判定。
- 計画側は範囲内候補だけを使用。未評価・範囲外・空候補では旧目標を消去。
- Viewerの既存Marker描画を再利用。未評価=黄、範囲内=緑、範囲外=灰。

## Added

- `GraspCandidate`: `id`、`pose`、`shape_score`、`state`（UNKNOWN=0、INSIDE=1、OUTSIDE=2）。
- `GraspCandidateArray`: `header`、`update_id`、`candidates`、評価座標系・時刻と到達セル寸法・原点。

## Fixed

- 候補と到達性の二重矢印、別トピック間の姿勢・時刻照合への依存。

## Removed

- `/grasp_pose_cands/reachability`、`/grasp_pose_cands/reachability_markers`の配信。
- 旧`GraspCandidateReachability`と`GraspCandidateReachabilityArray`。
- 計画launchの旧`target_pose_topic`、`target_point_topic`、`target_pose_array_topic`、`target_score_topic`。
- 計画側のreachability設定引数と`allow_untransformed_target`。TF不明時の計画なし。

## Behavior Impact

- 新しい候補集合では`update_id`を増加。状態だけの更新では`update_id`と候補`id`を維持。
- IDは集合内のみ有効。配信元の再起動で集合番号を再初期化。
- map未受信・TF不明・無効姿勢は未評価。空の到達mapは登録セルなしとして範囲外。
- TF移動は生成側タイマーで再評価。状態・評価座標系が不変なら再配信なし。
- 候補生成は一方式だけ起動。複数生成方式の自動排他・統合は対象外。
- no-motionの動作指令抑制は維持。位置範囲の判定は姿勢到達性・衝突回避・把持成功の保証ではない。

## Topics / Params / Messages

- 正規候補出力: `/grasp_pose_cands` (`gng_control_msgs/msg/GraspCandidateArray`)。
- 生成側: `reachability_map_topic`（空欄は未評価）、`reachability_voxel_size`（0.05 m）、`reachability_voxel_origin`（[0,0,0]）、`reachability_publish_hz`（5 Hz）。
- ToPoDualArm.yamlでは生成ノードの到達mapを`/ToPoDualArm/topological_map_static`へ設定。
- 計画launch: `candidate_topic:=/grasp_pose_cands`、`goal_update_hz:=5.0`。
- 互換用`/grasp_pose_cand_scores`と診断summaryは維持。候補のスコアは配列内に格納済み。
- 汎用PoseArray表示は維持。旧手動Marker変換launchの入力既定値は`/pose_array`へ分離。
- WSは既存`stream.marker_array`。候補IDと状態色へ変換し、全置換のまま配信。

## Verification

- Docker内の`gng_control_msgs`、`grasping_system`、`topo_fuzzy_viewer`ビルド。
- 変更対象の補助候補ノードをCMakeでビルド。
- `test_grasp_candidate_publisher`: ID保持、TF移動、負座標・境界、無効姿勢、空入力、単一配信元。
- `python3 gng_vlut_system/test/test_grasp_candidate_reachability.py`: 計画入力の6件。
- `python3 grasping_system/test/check_top_grasp_topic_integration.py`: 標準・個別出力先の2件。
- `python3 gng_vlut_system/test/check_grasp_candidate_joint_planning_integration.py`: 実GNGによる候補関節計画・旧計画消去。
- `node tests/check_pose_array_stream.cjs`: ROSからWebSocketへの同一IDの3色更新、汎用PoseArray、空候補、再接続。
- `node tests/pose_marker_renderer.test.mjs`: 矢印オブジェクトを増やさない色更新、TF、方向、消去。
- フロントエンドlint、tscによる型検査、Viteによる一時出力先へのビルド。

## Risk / Notes

- 破壊的なメッセージ型変更。生成・計画・Viewerを同じ定義で再ビルド・再起動する必要あり。
- 旧メッセージ削除後のPython型サポートが残る場合、`colcon build --packages-select gng_control_msgs --symlink-install --cmake-clean-first --cmake-force-configure`で生成物を再構築。
- 隔離ROSドメイン217/218で検証。検証用ノード・launchは停止済み。既存ノードの停止・再起動なし。
- ブラウザ実画面・実機動作の検証は対象外。
