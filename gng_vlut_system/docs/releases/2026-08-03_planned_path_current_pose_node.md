# 2026-08-03 - planned path current pose node

## 1. 要約

`/ToPoDualArm/planned_topological_map` に現在EE poseの仮想ノードを含めるようにした。

- 確定経路の可視化がGNG上のstart nodeから始まり、現在姿勢ノードを含んでいなかった。
- active trajectory publishでは `buildPathMessageWithCurrentPose()` を使い、現在EE poseから経路先頭ノードへのedgeを追加する。
- 現在姿勢のFK結果がNaN/infの場合は仮想ノードを追加しない。

## 2. 条件・検証

`/planned_topological_map` の先頭に仮想現在姿勢ノードが入り、現在姿勢から確定経路への接続が可視化される。
経路なしの場合も現在姿勢ノードだけを含むmapを出す。

- Topic: `/ToPoDualArm/planned_topological_map`
- Message type: `ais_gng_msgs/msg/TopologicalMap`
- Message schema changes: none

- `git diff --check`
- Docker内で `colcon build --packages-select gng_vlut_system`

**制約**

- 仮想現在姿勢ノードのIDは既存実装の `65535` を使う。
- 仮想現在姿勢ノードはsafe terrainではないため、`label=0` とする。
