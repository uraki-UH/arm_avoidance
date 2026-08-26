# 2026-08-26 - grasp voxel matcher YAML parameters

## Summary

把持ボクセル照合の入出力 topic と照合設定を `ToPoDualArm.yaml` に集約。

## Changed

- `grasp_voxel_matcher.launch.py` は `params_file` と `node_name` だけを受け付ける構成
- `grasp_voxel_matcher.launch.py` は matcher と Marker bridge に同じ parameter file を適用
- `grasp_voxel_template.launch.py` は matcher launch を再利用

## Removed

- `required_graph_topic`、`undersize_graph_topic`、`forbidden_graph_topic` などの matcher launch 引数

## Behavior Impact

グリッパー体積 graph と候補 topic の変更箇所を robot ごとの YAML に一本化。

## Topics / Params / Messages

- `/left_grasp_voxel_matcher` の `ros__parameters` に object voxel、体積 graph、候補出力、照合条件を配置
- `/grasp_pose_marker_bridge_node` の `ros__parameters` に Marker bridge の入出力 topic を配置

## Verification

- `ros2 launch grasping_system grasp_voxel_matcher.launch.py --show-args`
- `ros2 launch grasping_system grasp_voxel_matcher.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml`

## Risk / Notes

robot またはグリッパーごとに matcher を追加する場合は、対応する node 名の `ros__parameters` block を YAML に追加する。
