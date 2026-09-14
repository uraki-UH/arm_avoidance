# 2026-09-14 - グラフトピック名の短縮

## Summary

環境入力の`/topological_map`を維持し、関連グラフトピック名の`topological_map`を`Tmap`へ短縮。

## Changed

| 旧名 | 新名 |
| --- | --- |
| `/selected_topological_map` | `/selected_Tmap` |
| `/ToPoDualArm/topological_map_static` | `/ToPoDualArm/Tmap_static` |
| `/ToPoDualArm/plan_topological_map` | `/ToPoDualArm/plan_Tmap` |
| `/ToPoDualArm/cand_topological_map` | `/ToPoDualArm/cand_Tmap` |
| `topological_map_vis_L0` | `Tmap_vis_L0` |
| `plan_topological_map_vis_L0` / `cand_topological_map_vis_L0` | `plan_Tmap_vis_L0` / `cand_Tmap_vis_L0` |
| `/<template_id>/topological_map_static` | `/<template_id>/Tmap_static` |
| `/object_hypothesis/topological_map` | `/object_hypothesis/Tmap` |
| `/<template_id>/hypotheses/<hypothesis_id>/topological_map` | `/<template_id>/hypotheses/<hypothesis_id>/Tmap` |
| `L_grip_V_topological_map`など | `L_grip_V_Tmap`など（左右・各体積共通） |
| `arm_avoid/topological_map` | `arm_avoid/Tmap` |
| `/fuzzy_classifier/topological_map` | `/fuzzy_classifier/Tmap` |
| `/topological_map_transformed` | `/Tmap_transformed` |

ROSノードの既定値、launch、YAML、エクスポート設定例、現行仕様を更新。名前空間は維持。

## Added

Viewerの新旧経路トピック名の回帰テスト。目標選択の結合テストへ`/selected_Tmap`の受信確認を追加。

## Fixed

Viewerの経路用色設定判定を新旧名へ対応。通常のグラフ認識・バイナリ配信はメッセージ型ベースのため変更不要。

## Removed

旧名への重複配信や変換中継は追加なし。

## Behavior Impact

配信側・購読側の同時更新が必要。既存ノードは未再起動なので、稼働中の旧名配信は維持。Viewerの保存済みレイヤー設定はトピック名単位であり、新名のレイヤーは再選択が必要。旧rosbagは旧名のまま閲覧可能。

## Topics / Params / Messages

変更対象は上記トピック名。`ais_gng_msgs/msg/TopologicalMap`、WebSocketプロトコルの型名、launch名、`topological_map_topic`などのパラメータ名は維持。

## Verification

以下は小文字`tmap`時点の検証記録。`Tmap`への表記変更後の確認は末尾に追記。

- Docker内のbridge・経路計画共有ライブラリ・召喚・グリッパ・再分類ノードのビルド成功。
- frontendの型チェックと新旧経路名テスト成功。
- domain 217、WS port 19017で、新旧5トピックの検出・購読・2ノードのバイナリグラフ受信を確認。
- domain 218の経路結合テストは、並行作業によるC++目標選択ノードへの移行後、実行ファイル未配置で停止。結合成功とは扱わず、移行作業との調整待ち。
- `gng_transformer_node`は既存CMakeにビルド対象がなく、ソースの文字列変更のみ。今回ビルド成功とは扱わない。
- 検証起動プロセスはすべて終了。既存ROSノードのPID維持を確認。

実行コマンド（すべて終了済み）:

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 600s cmake --build /ros2_ws/build/gng_vlut_system --target topofuzzy_bridge_node topological_map_path_planner_node topological_map_avoidance_node arm_avoid_node visualization_gng_static_node object_hypothesis_summon_node object_template_map_publisher_node object_match_hypothesis_publisher_node -j2'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 360s cmake --build /ros2_ws/build/gng_vlut_system --target topological_map_planning -j2'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 240s cmake --build /ros2_ws/build/grasping_system --target gripper_volume_graph_node grasp_voxel_matcher_node -j2'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 180s cmake --build /ros2_ws/build/fuzzy_classifier --target cluster_relabel_node -j2'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 180s cmake --build /ros2_ws/build/pointcloud_transformer_cpp --target gng_transformer_node -j2'
timeout 70s node /tmp/tmap_viewer_probe.cjs
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 180s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py'
# frontendディレクトリ内
timeout 60s node --test tests/graph_topic_names.test.mjs
timeout 120s ./node_modules/.bin/tsc -p tsconfig.app.json --noEmit --incremental false
```

## Risk / Notes

実画面でのGPU描画操作は未検証。過去の進捗・リリース記録中の旧トピック名は履歴として維持。C++目標選択移行の結合検証は[別作業の完了記録](2026-09-14_goal_selector_cpu.md)を参照。

### Tmap表記への統一

ユーザー指定で短縮表記を`Tmap`へ統一。ROSの既定値・設定・Viewer判定・テストを更新し、実処理の行数増加なし。Viewerは従来の`tmap`と`topological_map`も認識。
関連3パッケージの再ビルド、frontend型チェック・名前判定テスト、domain 218の経路結合テストに成功。変換ノードの既存ビルド対象欠落は変更なし。
検証コマンドは上記のgrasping_system・fuzzy_classifier・frontend・結合テストと同じ。gng_vlut_systemの再ビルドは以下。すべて終了済み。既存ROSへの停止・再起動操作なし。

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 600s cmake --build /ros2_ws/build/gng_vlut_system --target topofuzzy_bridge_node topological_map_planning topological_map_goal_selector_node arm_avoid_node visualization_gng_static_node object_hypothesis_summon_node object_template_map_publisher_node object_match_hypothesis_publisher_node -j2'
```
