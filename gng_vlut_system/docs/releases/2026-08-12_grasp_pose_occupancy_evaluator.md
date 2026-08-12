# 2026-08-12 - Grasp pose occupancy evaluator

## Summary

把持手先Pose候補を、TCPローカル座標で定義した占有領域ルールにより判定する独立APIを追加する。

## Added

- `GraspPoseOccupancyEvaluator`と領域別の判定結果・違反理由。
- `required_occupied`、`required_empty`、`optional`、`optional_not_sole_support`の4ルール。
- `GraspGraphModel`の有効ノードを占有領域サンプルへ変換する関数。
- 点群、ボクセル集合、占有グリッドのいずれにも接続できる占有照会callback。
- 候補Poseの並進・回転を考慮した領域サンプル変換の単体テスト。

## Behavior Impact

- 既存ROS nodeの入出力と候補生成挙動は変更しない。
- 今後、候補生成後のfilterまたはscore計算から同じ判定器を利用できる。

## Verification

- `grasp_pose_occupancy_evaluator`単体テストで4ルール、Pose変換、graph変換を確認する。

## Risk / Notes

- ROS topicとYAML定義への接続は未実装。
- 重複する領域サンプルの優先順位は付けず、各領域を独立に判定する。
