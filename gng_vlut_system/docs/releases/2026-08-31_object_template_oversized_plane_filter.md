# 物体テンプレート照合の過大平面除外

## 変更内容

- `object_template_matcher_node`が`PlaneClusterArray`を購読し、テンプレートGNGの平面クラスタと比較するよう変更
- 法線方向が適合し、寸法が全テンプレート平面の許容上限を超える環境平面を背景として除外
- 除外平面の所属nodeとedgeを対応、反証、スケール評価から除外
- 候補JSONへ`ignored_plane_node_num`と`ignored_plane_cluster_ids`を追加
- テンプレート平面と環境平面をyaw仮説ごとに一対一対応させ、対応済み平面の所属nodeだけをGNG node照合へ入力
- 平面対応score合計を飽和変換した`plane_support_score`によって複数平面を説明する姿勢を優先
- 十分な平面根拠がある候補は、GNG edge一致率の不足だけで検証器が除外しない形式

## 設定項目

- `enable_oversized_plane_filter`
- `plane_clusters_topic`
- `max_plane_normal_angle_deg`
- `max_plane_extent_overflow_ratio`
- `max_plane_cluster_frame_lag`
- `enable_plane_cluster_evaluation`
- `min_plane_extent_allow_ratio`
- `min_plane_extent_full_match_ratio`
- `max_plane_extent_full_match_ratio`
- `plane_weight`
- `plane_support_score_scale`
- `min_plane_support_score`

## 互換性

- テンプレートに`gng.plane_clusters`がない場合、または平面クラスタtopicが未受信・frame番号が離れている場合は従来どおりのnode照合
- GNG学習器、平面クラスタ生成器、保存済みGNGのバイナリ化経路は変更なし
