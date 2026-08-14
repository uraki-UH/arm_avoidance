# 2026-08-14 - Stable object grasp voxel matching

## Summary

`SAFE_TERRAIN,HUMAN,CAR`を除いた時間的に安定したGNGボクセルを物体候補とし、既存グリッパ体積graphで把持候補TCP Poseを照合できるようにした。

## Changed

- `topological_grid_node`を、現在点群の支持があり、`SAFE_TERRAIN`、`HUMAN`、`CAR`以外のlabelを持つセルの物体候補抽出へ変更した。
- 把持占有評価のROS候補生成未接続という仕様記述を、実装済みtopic契約へ更新した。

## Added

- 整数量子化済みoffsetとhash占有集合を使う`GraspVoxelMatcher`。
- `grasp_voxel_matcher_node`と`grasp_voxel_matcher.launch.py`。
- 最大把持領域、最小把持領域、基部禁止領域を独立ゲートで照合するテスト。
- 安定化済みボクセルを頂点、GNG edgeを間接的なボクセル接続として中間セルを補間する処理。

## Fixed

- `topological_grid_node`のlaunch、ノード、共通グリッド定義で既定セルサイズを`0.01 m`へ統一した。

## Removed

- `topological_grid_node`の`included_labels`。候補を特定labelだけへ限定できないようにし、`excluded_labels`だけを除外条件とする。

## Behavior Impact

- 各セルは直近100同期更新の非除外label出現回数と点群input更新回数を独立に保持する。
- `DEFAULT`、`WALL`、`UNKNOWN_OBJECT`は同じ物体候補占有として履歴を合算し、相互のlabel変更では確定状態を解除しない。
- `grid_size`を省略した場合も、物体候補ボクセルは一辺`0.01 m`で生成する。
- 既定では隣接セルはlabel・点群inputが各3回、26近傍に候補がない孤立セルは各5回を履歴内で満たす必要がある。連続観測は要求しない。
- 新規追加時だけ現在点群を必須とし、確定後に非除外labelがセルから消えても既定10更新は旧セルを維持する。
- 確定後の点群履歴低下で削除するのは孤立セルだけとし、非孤立セルは点群履歴だけでは解除しない。
- label履歴最低カウントを新規・再追加専用とし、確定済みセルはlabel履歴が0でもlabel不在猶予内なら保持する。
- 孤立判定を非除外GNGセル同士の近傍関係で行い、点群消失だけでは孤立扱いに変更しない。
- GNG edgeの両端が直接観測ボクセルへ所属する場合だけ、既定`0.10 m`以下のボクセル間を補間する。補間セルから再帰的に補間しない。
- 物体候補の主topicは直接観測とedge補間の和集合とし、補間由来だけのtopicも別途publishする。
- matcherは既定で500 ms周期、最大500アンカー、12 yaw姿勢、上位50候補に制限する。
- `environment_voxels_topic`未指定時は、物体候補占有を禁止領域検査にも使用する。

## Topics / Params / Messages

- `topological_grid_node`: `pointcloud_topic`、`pointcloud_timeout_sec`、`excluded_labels`、`require_input_points`、`minimum_input_points_per_voxel`、`neighbor_radius_cells`、`history_window_size`、`maximum_missing_label_updates`と、通常・孤立セル別のlabel・点群履歴最低カウントparameterを追加。
- edge補間parameter: `edge_inference_enabled`、`edge_max_length`、`edge_inferred_topic`。
- 両launchへ複数grid・左右matcherを同時起動するための`node_name`を追加。
- matcher入力: `object_voxels_topic`、`environment_voxels_topic`、`required_graph_topic`、`undersize_graph_topic`、`forbidden_graph_topic`。
- matcher出力: `geometry_msgs/PoseArray`と`std_msgs/String` JSON summary。
- 新規messageは追加していない。

## Verification

- Docker内で`voxel_msgs`、`ais_gng`、`grasping_system`を`BUILD_TESTING=ON`でビルド。
- `grasp_voxel_matcher`、`grasp_pose_occupancy_evaluator`、`gripper_volume_graph`、`test_topological_grid_assignment`が成功。
- 合成3セル対象で候補1件、占有必須3/3、最小体積外支持2、禁止領域衝突0をROS topicで確認。
- 非連続観測を含むlabel履歴4回、点群input履歴4回を独立に取得し、現在支持がある場合だけ出力することを確認。
- 10 mmセル既定値、点群支持、GNG近傍判定、100更新リング、孤立セルだけの点群削除、label履歴0での保持、位置揺れ猶予、edge補間を含む`test_topological_grid_assignment` 18件が成功。
- 分離した`ROS_DOMAIN_ID=126`で引数なし起動し、`grid_size=0.010`を確認後にノードを終了。
- 実`/topological_map`入力で`frame_id=graspnet_table`、`voxel_size=0.01 m`、`origin=(0,0,0)`を確認。
- 点群支持なし、対象外label、26近傍、label別カウント、孤立セルの各5回判定と100更新後の履歴削除を単体テストで確認。
- 確定済みセルは現在点群が消えても履歴内の点群inputが最低カウント以上なら保持し、窓から抜けると削除されることを確認。
- 非除外label間の変更では占有が途切れず、ノードが隣接セルへ移動した場合は旧セルと新セルが設定期間だけ重なることを確認。
- 非孤立セルは点群履歴が0でも保持され、孤立セルだけが点群履歴低下で削除されることを確認。
- 確定済みセルはlabel履歴が0まで抜けても不在猶予の最終更新まで保持され、猶予超過後に削除されることを確認。
- 実`/topological_map`と`/downsampling/unknown`を同期入力し、1479ノード中311ノードが点群支持条件を満たして216セルだけが出力候補になることを確認。
- 分離したROS domainの合成edgeで、直接観測2セル、補間3セル、和集合5セルをROS topic本文から確認。

## Risk / Notes

- 現行POCは固定姿勢集合による直接照合であり、局所法線からの3次元姿勢生成と物体ID分離は未実装。
- 実点群で処理上限を超える場合は、接触posting indexによる逆引きへ置き換える。
- 現在のedge補間は観測free空間との交差判定を持たないため、`edge_max_length`を実データのGNG edge分布に合わせて調整する。
