# 2026-08-14 - Stable object grasp voxel matching

## Summary

`SAFE_TERRAIN,HUMAN,CAR`を除いた時間的に安定したGNGボクセルを物体候補とし、既存グリッパ体積graphで把持候補TCP Poseを照合できるようにした。

## Changed

- `topological_grid_node`を、現在点群の支持がある`UNKNOWN_OBJECT`だけを時間確認する物体候補抽出へ変更した。
- 把持占有評価のROS候補生成未接続という仕様記述を、実装済みtopic契約へ更新した。

## Added

- 整数量子化済みoffsetとhash占有集合を使う`GraspVoxelMatcher`。
- `grasp_voxel_matcher_node`と`grasp_voxel_matcher.launch.py`。
- 最大把持領域、最小把持領域、基部禁止領域を独立ゲートで照合するテスト。

## Fixed

- `topological_grid_node`のlaunch、ノード、共通グリッド定義で不一致だった既定セルサイズを`0.02 m`へ統一した。

## Removed

- なし。

## Behavior Impact

- `minimum_observations`を2以上にすると、指定回数へ達するまでボクセルをpublishしない。
- `grid_size`を省略した場合も、物体候補ボクセルは一辺`0.02 m`で生成する。
- 既定では隣接セルを3回、26近傍に候補がない孤立セルを5回連続確認してからpublishする。
- 点群支持のない更新では出力せず、孤立セルの確認履歴は即時削除する。
- matcherは既定で500 ms周期、最大500アンカー、12 yaw姿勢、上位50候補に制限する。
- `environment_voxels_topic`未指定時は、物体候補占有を禁止領域検査にも使用する。

## Topics / Params / Messages

- `topological_grid_node`: `pointcloud_topic`、`pointcloud_timeout_sec`、`included_labels`、`require_input_points`、`minimum_input_points_per_voxel`、`neighbor_radius_cells`と、通常・孤立セル別の時間確認parameterを追加。
- 両launchへ複数grid・左右matcherを同時起動するための`node_name`を追加。
- matcher入力: `object_voxels_topic`、`environment_voxels_topic`、`required_graph_topic`、`undersize_graph_topic`、`forbidden_graph_topic`。
- matcher出力: `geometry_msgs/PoseArray`と`std_msgs/String` JSON summary。
- 新規messageは追加していない。

## Verification

- Docker内で`voxel_msgs`、`ais_gng`、`grasping_system`を`BUILD_TESTING=ON`でビルド。
- `grasp_voxel_matcher`、`grasp_pose_occupancy_evaluator`、`gripper_volume_graph`、`test_topological_grid_assignment`が成功。
- 合成3セル対象で候補1件、占有必須3/3、最小体積外支持2、禁止領域衝突0をROS topicで確認。
- `minimum_observations=3`で初期出力0、3回観測後1ボクセル、`SAFE_TERRAIN`除外を確認。
- 20 mmセル既定値、点群支持、近傍判定、通常・孤立セルの時間確認を含む`test_topological_grid_assignment` 9件が成功。
- 分離した`ROS_DOMAIN_ID=126`で引数なし起動し、`grid_size=0.020`を確認後にノードを終了。
- 実`/topological_map`入力で`frame_id=graspnet_table`、`voxel_size=0.02 m`、`origin=(0,0,0)`を確認。
- 合成Mapで点群支持付き隣接unknown 2セルが1回目は0件、3回目は2件になることを確認。
- 点群支持なし、対象外label、26近傍、孤立セル5回確認と即時履歴削除を単体テストで確認。
- 実`/topological_map`と`/downsampling/unknown`を同期入力し、1479ノード中311ノードが点群支持条件を満たして216セルだけが出力候補になることを確認。

## Risk / Notes

- 現行POCは固定姿勢集合による直接照合であり、局所法線からの3次元姿勢生成と物体ID分離は未実装。
- 実点群で処理上限を超える場合は、接触posting indexによる逆引きへ置き換える。
