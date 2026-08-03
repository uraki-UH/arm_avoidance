# 2026-08-03 - candidate paths per-goal start selection

## Summary

`/ToPoDualArm/candidate_topological_map` の候補経路を、各goal候補ごとに現在姿勢近傍のstart候補から生成するようにした。

## Changed

- 以前は全候補goalを、最良goalに対して選ばれた1つの `selected_start_id` から再計画していた。
- 変更後は各goal候補ごとに複数のstart候補を試し、そのgoalに対する最良start/pathを `candidate_paths` に入れる。
- `selected_start_id` は、最終的に選ばれたbest pathのstartとして維持する。

## Behavior Impact

候補軌道可視化で、すべての候補が同一start nodeへ強制集約されにくくなる。
現在姿勢から複数の近傍start候補へ伸びる候補pathがある場合、`candidate_topological_map` 上でもその違いが表現される。

## Topics / Params / Messages

- Topic: `/ToPoDualArm/candidate_topological_map`
- Message type: `ais_gng_msgs/msg/TopologicalMap`
- Message schema changes: none

## Verification

- `git diff --check`
- Docker内で `colcon build --packages-select gng_vlut_system`

## Risk / Notes

- 複数候補pathが同じ近傍start nodeを選ぶ場合、現在姿勢から最初のGNG nodeまでのedgeは同じ座標に重なって見える。
- `TopologicalMap` はpath IDを持たないgraph表現のため、完全に候補ごとのpolyline分離が必要な場合は別topicまたはpath ID付きmessageを検討する。
