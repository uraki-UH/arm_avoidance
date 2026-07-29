# 2026-07-29 - candidate robot preview opacity follows viewer defaults

## Summary

候補ロボットプレビューの `opacity` を ROS 側で持たず、ToPoFuzzy Viewer 側の既定値に委ねるように整理した。

## Changed

- `topological_map_avoidance_node` から `candidate_robot_preview_opacity` パラメータを削除
- 候補プレビュー payload 生成時に `opacity` を送らないように変更
- 技術仕様書から ROS 側の `candidate_robot_preview_opacity` 記載を削除

## Added

- なし

## Fixed

- viewer 側の default opacity を使いたいのに、ROS 側が値を上書きしていた問題を解消

## Removed

- ROS 側の候補ロボットプレビュー opacity 設定

## Behavior Impact

- 候補ロボットプレビューの見た目は viewer 側の既定値に従う
- ROS 側はプレビュー送信の有無だけを制御する

## Topics / Params / Messages

- Removed: `candidate_robot_preview_opacity`

## Verification

- `colcon build --packages-select gng_vlut_system --cmake-args -DCMAKE_BUILD_TYPE=Release`

## Risk / Notes

- ToPoFuzzy Viewer 側の default opacity が期待どおりでない場合は、viewer 側設定を調整する
