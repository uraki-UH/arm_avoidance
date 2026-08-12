# 2026-08-12 - grip_V topic rename

## Summary

グリッパー体積graphのROS topic名を短縮し、`gripper_volume`を`grip_V`へ変更した。

## Changed

- ToPoDualArmの最大把持領域と`undersize`領域の左右4topic。
- rosbag bundle exporterのtopicとalias。
- 単体HTMLのrosapi自動発見条件。
- 汎用`gripper_volume_graph_node`とlaunchの既定出力topic。

## Behavior Impact

旧topicはpublishせず、旧topic名のライブ自動発見互換も持たない。bundle内の
`role: gripper_volume_graph`はtopic名ではなく意味上の内部契約なので変更しない。

## Topics / Params / Messages

- `/ToPoDualArm/L_grip_V_topological_map`
- `/ToPoDualArm/R_grip_V_topological_map`
- `/ToPoDualArm/L_grip_V_undersize_topological_map`
- `/ToPoDualArm/R_grip_V_undersize_topological_map`
- message: `ais_gng_msgs/msg/TopologicalMap`（変更なし）

## Verification

- `grasping_system`をbuildし、CTest 2件・計9ケースの成功を確認した。
- 3 YAMLのparseと、単体HTML内3 inline scriptの構文compileに成功した。
- `ROS_DOMAIN_ID=133`で指定launchを起動し、新名称の4topicだけが生成されることを確認した。
- 最大領域は左右とも504 nodes / 1321 edges、`undersize`は左右とも130 nodes / 279 edgesで、名称変更前のgraph内容を維持した。
- 検証用launchを停止し、既存launchとDocker containerの実行状態を維持した。

## Risk / Notes

- ROS topic名はcase-sensitiveであり、`grip_V`の`V`は大文字とする。
- recorder、subscriber、保存済み設定で旧topic名を直接指定している場合は更新が必要。
