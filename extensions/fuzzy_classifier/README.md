# fuzzy_classifier

TopologicalMapのクラスタ再ラベル・統計集計・Marker表示。
標準構築・標準起動から分離した、再利用用の任意パッケージ。

## I/O

入力: /topological_map。出力: /fuzzy_classifier/topological_map・cluster_summary・cluster_boxes・cluster_nodes。
型・出力設定は[launch](launch/)・[config](config/)を参照。

## 実行

```bash
ros2 launch fuzzy_classifier cluster_relabel.launch.py
ros2 launch fuzzy_classifier cluster_summary.launch.py input_topic:=/fuzzy_classifier/topological_map
```

ais_gng_msgsと点群処理側をビルド済みのROS環境が必要。
再利用時は移動先を指定して再構築。通常探索からは親の`extensions/COLCON_IGNORE`で除外。
設定は[relabel.yaml](config/relabel.yaml)を参照。分類結果は観測クラスタの評価であり、実物の確定認識とは別。
