# 2026-08-27 - Object match hypothesis skeleton

## 1. 要約

環境側GNGと物体テンプレートGNGの照合候補を、簡略化グラフとAABBとして配信するROS 2雛形を追加した。

- `object_match_hypothesis_publisher_node`
- `object_match_hypothesis_publisher.launch.py`
- 環境クラスタ選択または環境全体選択
- 空間セル単位のnode集約とedge重複除去
- AABBの`MarkerArray`と候補metadata JSON

## 2. 条件・検証

- 現段階では照合、位置合わせ、yaw探索を実行しない。
- `score`と`yaw_deg`は将来の照合器から受け取る候補属性。
- `ais_gng_cpu`、基幹GNGバイナリ、既存のテンプレート配信topicは変更しない。

**トピック**

- `/<template_id>/hypotheses/<hypothesis_id>/topological_map`
- `/<template_id>/hypotheses/<hypothesis_id>/markers`
- `/<template_id>/hypotheses/<hypothesis_id>/metadata`
