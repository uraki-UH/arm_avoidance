# 2026-08-12 - grip_minV topic rename

## Summary

全閉時に残る最小側の自由領域topicを`grip_V_undersize`から`grip_minV`へ短縮した。

## Changed

- ToPoDualArm左右の最小領域topic。
- rosbag bundle exporterのtopicとalias。
- 単体HTMLのrosapi自動発見条件。

## Behavior Impact

旧`grip_V_undersize` topicはpublishしない。graphの生成条件、node、edge、frame、message型は変更しない。

## Topics / Params / Messages

- `/ToPoDualArm/L_grip_minV_topological_map`
- `/ToPoDualArm/R_grip_minV_topological_map`
- message: `ais_gng_msgs/msg/TopologicalMap`（変更なし）

## Verification

- 3 YAMLのparseと、単体HTML内3 inline scriptの構文compileに成功した。
- `ROS_DOMAIN_ID=135`で指定launchを起動し、左右の`grip_V`と`grip_minV`の4topicだけが
  生成され、旧`grip_V_undersize` topicがないことを確認した。
- `grip_minV` graphは左右とも130 nodes / 279 edgesで、名称変更前の内容を維持した。
- 検証用launchを停止し、既存launchとDocker containerの実行状態を維持した。

## Risk / Notes

- ROS topic名はcase-sensitiveであり、`grip_minV`の`V`は大文字とする。
- recorder、subscriber、保存済み設定で旧topic名を直接指定している場合は更新が必要。
