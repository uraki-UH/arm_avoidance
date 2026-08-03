# 2026-08-03 - visualization trajectory bridge

## Summary

元姿勢GNGノードIDで配信される実行軌道と候補軌道を、topofuzzy bridge内で
可視化GNGノード列へ線形時間で変換する配信経路を追加した。

## Added

- 可視化layerごとに`source_node_id -> visual_node_id`の密な逆引き配列を起動時に構築。
- planned/candidate `TopologicalMap`を可視化ノードとedgeへ変換するhelper。
- 現在姿勢仮想ノードID `65535`の位置と接続の保持。
- planned/candidateそれぞれの可視化layer topic。

## Behavior Impact

既存の元軌道topicと静的可視化GNG topicは変更しない。追加topicは入力を
transient-localで購読し、変換後の最新軌道をtransient-localで配信する。
同一可視化ノードへ属する連続元ノードは統合され、自己loopと重複edgeは出力しない。

変換時にFKや全元ノード探索は行わない。逆引き構築は起動時`O(n)`、単一路径変換は
`O(L)`である。angle edgeをFK補間した中間可視化ノード列の展開は本変更に含まない。

## Topics / Params / Messages

- Input: `/ToPoDualArm/planned_topological_map`
- Output: `/ToPoDualArm/planned_topological_map_visualization_layer_0`
- Input: `/ToPoDualArm/candidate_topological_map`
- Output: `/ToPoDualArm/candidate_topological_map_visualization_layer_0`
- Message: `ais_gng_msgs/msg/TopologicalMap`
- Params: `visualization_gng.trajectory_input_topic`
- Params: `visualization_gng.trajectory_topic_prefix`
- Params: `visualization_gng.candidate_trajectory_input_topic`
- Params: `visualization_gng.candidate_trajectory_topic_prefix`

## Verification

- Humbleコンテナ内で`gng_vlut_system`全体build成功。
- 隔離ROS domainで元ID `[65535,318,381,722]`をplanned入力し、可視化ID
  `[65535,0,3]`、edge `[0-1,1-2]`を実受信。
- 隔離ROS domainで元ID `[318,381,722]`をcandidate入力し、可視化ID
  `[0,3]`、edge `[0-1]`を実受信。
- 元ID `318`と`381`が同じ可視化ノードへ統合されることを確認。
- テスト用bridgeとrclpy nodeをSIGINTで終了し、一時logを削除。
- `git diff --check`。

## Risk / Notes

現在の`TopologicalMap`は複数候補経路をedgeの和集合として表すため、変換後も候補ごとの
経路識別情報は持たない。必要になった場合は経路配列を保持する別messageで扱う。
