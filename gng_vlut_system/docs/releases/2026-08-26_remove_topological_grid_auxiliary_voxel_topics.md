# 2026-08-26 - remove topological grid auxiliary voxel topics

## Summary

物体候補ボクセルは主 `output_topic` と孤立セル・summaryだけをpublishする構成へ整理。

## Changed

- `grasp_voxel_matcher_node` は全量 `Voxel` snapshot による照合へ統一
- edge と triangle の補間セルは主 `output_topic` にだけ統合

## Removed

- `<output_topic>/delta`
- `<output_topic>/edge_inferred`
- `<output_topic>/triangle_inferred`
- matcher の `incremental_matching_enabled`、`object_delta_topic`、`environment_delta_topic` parameter

## Behavior Impact

補間セルの把持候補への寄与は変わらない。候補照合は snapshot 受信ごとに全量を再評価する。

## Topics / Params / Messages

- 主 `output_topic` は非孤立の直接観測、edge補間、triangle面の和集合
- `isolated_topic` と `summary_topic` は継続

## Verification

- `colcon build --packages-select ais_gng grasping_system --symlink-install`

## Risk / Notes

既存の差分topic購読者は主 `output_topic` の snapshot 購読へ切り替えが必要。
