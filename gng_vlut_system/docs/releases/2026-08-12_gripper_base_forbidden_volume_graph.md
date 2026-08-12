# 2026-08-12 - Gripper base forbidden-volume graph

## Summary

把持候補Poseでグリッパ基部が環境へ衝突するか評価するため、左右基部STLの占有形状を
`TopologicalMap`として配信する`grip_baseV` graphを追加した。

## Added

- builderの`retain_occupied_meshes`モード。従来のメッシュ除外とは逆に、メッシュ占有格子だけを残す。
- ToPoDualArm左右基部STLの禁止領域設定。
- Viewerライブ自動発見とrosbag bundle exporter設定。
- `gripper_forbidden_volume_graph` role。

## Behavior Impact

- `grip_V`と`grip_minV`の形状・topic・生成件数は変更しない。
- `grip_baseV`は把持可能領域ではなく、将来`required_empty`として使う禁止領域である。
- 現時点ではgraph配信とViewer/bundle評価入力への格納までで、候補生成nodeの自動却下には未接続。

## Topics / Params / Messages

- `/ToPoDualArm/L_grip_baseV_topological_map` (`ToPoDualArm/L_tcp`)
- `/ToPoDualArm/R_grip_baseV_topological_map` (`ToPoDualArm/R_tcp`)
- message: `ais_gng_msgs/msg/TopologicalMap`（変更なし）
- `resolution: 0.005 m`
- `exclusion_clearance: 0.005 m`
- `retain_occupied_meshes: true`
- `include_cluster: false`

## Verification

- `grasping_system`をwarningなしでbuildし、CTest 2件・計9ケースの成功を確認した。
- 占有メッシュ保持、従来のメッシュ除外、メッシュ未指定時の拒否を単体テストで確認した。
- 3 YAMLのparseと、単体HTML内3 inline scriptの構文compileに成功した。
- `ROS_DOMAIN_ID=136`で指定launchを起動し、左右の`grip_V`、`grip_minV`、`grip_baseV`の
  6 topicを確認した。
- 基部graphは左右とも1271 nodes / 3156 edges、`occupied_only=true`、`clusters=0`だった。
- 既存graphは最大領域248 nodes / 585 edges、最小領域130 nodes / 279 edgesを維持した。
- 検証用launchを停止し、既存launchとDocker containerの実行状態を維持した。

## Risk / Notes

- 5 mm安全余裕は初期値であり、実機の位置誤差や環境点群のvoxel寸法に合わせた調整が必要。
- `required_empty`評価への接続時は、単一占有点での棄却が過敏なら占有数または占有率閾値を設定する。
