# 2026-09-02 - 非平面成分のMarkerArray出力統一

## 変更

- `nonplane_component_node`の`/nonplane_components` (`TopologicalMap`) publishを廃止
- `/nonplane_components/markers` (`MarkerArray`)を非平面成分の唯一の実行時出力へ統一

## 理由

- MarkerArrayに成分node、内部GNG edge、平面アンカーedgeを含む可視化情報の集約
- 未購読の`TopologicalMap`複製publishの削減

## 影響

- `output_topic` parameterの廃止
- 構造化した照合入力は、今後`/nonplane_component_features`で提供
