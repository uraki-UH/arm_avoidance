# 物体テンプレート照合のスケールファジー評価

## 目的

環境GNGとテンプレートGNGの大きさが完全一致しない場合でも、テンプレートごとの許容幅に従って
連続的に照合scoreを評価する仕組み。

## 変更

- 対応済みGNG edge長の環境/テンプレート比の中央値による`scale_ratio`推定
- `min_scale_allow_ratio`、`min_scale_full_match_ratio`、`max_scale_full_match_ratio`、`max_scale_allow_ratio`による両側ファジー評価
- `scale_weight`による全体scoreへの寄与設定
- 許容域外の`scale_ratio`を持つ候補の反証
- 対応edge不足時のスケール未観測扱い

## 設定

`object_template_matching.yaml`の`enable_scale_evaluation`を有効にし、対象物ごとに比率の許容幅を設定する。
初期値は0.95から1.05を満点域、0.70から1.30を許容域とする。
