# 2026-08-14 - Incremental grasp voxel matching

## Summary

物体候補ボクセルの全量照合を、初期snapshot後は変更セルだけから把持候補へ反映する方式に拡張した。

## Changed

- `voxel_msgs/Voxel`へ`revision`を追加した。revision 0は既存publisher互換の未管理streamを表す。
- `topological_grid_node`は全量`Voxel`を維持したまま、既定`<output_topic>/delta`へ`VoxelLabelDelta`をpublishする。
- `grasp_voxel_matcher_node`はrevision付き入力で、`grip_V`、`grip_minV`、`grip_baseV`に対応する候補ごとの占有数を増分更新する。

## Added

- `voxel_msgs/VoxelLabelDelta`。`data`、`old_labels`、`new_labels`は同じ長さで、`255`はセル不在を表す。
- `topological_grid_node`の`delta_topic` parameter。
- matcherの`incremental_matching_enabled`、`object_delta_topic`、`environment_delta_topic` parameters。
- summaryの`matching_mode`、`candidate_states_tracked`、`candidate_states_updated`、grid側のrevisionと差分セル数。

## Behavior Impact

- 初回、template変更、格子変更、差分欠落時だけ全量snapshotから候補状態を再構築する。
- revisionを送らない既存`voxel_msgs/Voxel` publisherは、従来どおり全量照合される。
- 現在の判定はproducerが物体候補として出力したセルの有無だけを使う。同じ存在セル内のlabel変更は候補の占有数を変えない。

## Verification

- `voxel_msgs`、`ais_gng`、`grasping_system`を`BUILD_TESTING=ON`でビルド。
- `grasp_voxel_matcher`単体テストで、必須セルと禁止セルの追加・削除が増分状態へ反映されることを確認。
- `test_topological_grid_assignment` 26件が成功。
