# 2026-08-03 - evaluation metrics reduced manipulability metrics

## Summary

`/evaluation_metrics` の可操作性指標から、楕円体などの基礎データから派生できる値を減らした。

## Changed

- HTMLは受信した `EvaluationMetrics` schemaをそのまま動的に扱い、個別metric名を特別扱いしない。

## Removed

- `/evaluation_metrics` の固定schemaから `manipulability_condition_number` と `min_singular_value` を削除した。
- `/evaluation_metrics` の固定schemaから `manipulability_singular_values` を削除した。
- `metric_names` 経由で `/evaluation_metrics` に追加されていた `joint_limit_score` を削除した。
- `metric_names` 経由で `/evaluation_metrics` に追加されていた `is_colliding` と `is_danger` を削除した。
- `metric_names` 経由で `/evaluation_metrics` に追加されていた `collision_count` と `danger_count` を削除した。

## Behavior Impact

HTMLは受信した `EvaluationMetrics` schemaをそのまま動的に扱う。
条件数、最小特異値、特異値配列が必要な場合は、後段で可操作性楕円体などの基礎データから派生計算する。
関節限界余裕は固定metricの `joint_limit_margin_min` / `joint_limit_margin_mean` を使う。
診断用の `/grasp_candidate_metrics` では互換性のため既存フィールドを維持する。

## Topics / Params / Messages

- Topic: `/evaluation_metrics`
- Message type: `gng_control_msgs/msg/EvaluationMetrics`
- Source diagnostic topic remains: `gng_control_msgs/msg/GraspCandidateMetricArray`

## Verification

- `git diff --check`
- `ToPo-FUZZY_Manipulation_v1.html` のscript構文チェック
- Docker内で `colcon build --packages-select gng_vlut_system`

## Risk / Notes

- 既存の古いrosbagや旧publisherが削除済みmetricを含むschemaを出す場合、HTMLはそのschemaに従って表示する。
