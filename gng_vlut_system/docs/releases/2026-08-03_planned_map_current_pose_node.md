# 2026-08-03 - planned map current pose node

## Summary

`/ToPoDualArm/planned_topological_map` が、軌道未確定時にも現在EE poseの仮想ノードを含むようにした。

## Fixed

- 軌道なしで空pathをpublishする分岐が、現在姿勢を持たない `buildPathMessage()` を使っていた。
- `planned_topological_map` のpublish経路を `buildPathMessageWithCurrentPose()` に統一した。
- 現在姿勢のFK結果がNaN/infの場合は仮想ノードを追加しない。

## Behavior Impact

`/ToPoDualArm/planned_topological_map` は、経路ノードが空でも現在EE poseを先頭ノードとしてpublishする。
有効な経路がある場合は、現在EE poseノードから最初のGNG path nodeへedgeを張る。

## Topics / Params / Messages

- Topic: `/ToPoDualArm/planned_topological_map`
- Message type: `ais_gng_msgs/msg/TopologicalMap`

## Verification

- `git diff --check`
- Docker内で `colcon build --packages-select gng_vlut_system`

## Risk / Notes

- 現在EE poseノードは `id=65535` の仮想ノードとして表現する。
- 仮想現在姿勢ノードはsafe terrainではないため、`label=0` とする。
