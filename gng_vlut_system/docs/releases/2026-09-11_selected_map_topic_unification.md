# 2026-09-11 - 選択ノードの可視化トピック統一

## Summary

選択ノードの配信を`/selected_topological_map`へ統一。

## Changed

TopologicalMapを直接可視化する構成。計画処理用の`/selected_goal_candidate_ids`は維持。

## Added

選択ノードIDの保持とマーカpublisher不在の回帰確認。

## Fixed

同一選択ノードに対するTopologicalMapとMarkerArrayの二重配信。

## Removed

- `/selected_topological_map_markers`のpublisherとマーカ生成。
- selectorの`--marker-topic`、selector launchと`grasp_joint_candidates.launch.py`の`marker_topic`引数。

## Behavior Impact

ノード選定・TF変換・空の選択結果によるクリア処理は維持。Viewerでは`/selected_topological_map`を選択。

## Topics / Params / Messages

TopologicalMapのメッセージ形式変更なし。廃止対象は選択ノード用の重複マーカだけ。

## Verification

`python3 /ros2_ws/src/gng_vlut_system/test/test_grasp_candidate_reachability.py`で6テスト成功。有限timeout内で終了、テストノードは破棄済み。

## Risk / Notes

- 既存の計画ノードは停止せず維持。次回起動から新構成を適用。
- 独自起動コマンドの`--marker-topic`指定は削除が必要。
- 実画面での表示確認は未実施。
