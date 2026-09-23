# AiS-GNG

## 立ち上げ方

```
docker network create gng   # 初回のみ
docker compose up -d --build
```

## docker 入り方
```
docker compose exec gng_cpu bash
```

## コアのビルド方法 && ros2へコピー
```
cd /ros2_ws/src/ais_gng/core/scripts
./build.sh
```
## ビルド
```
cb
```

## 入力トピックの指定

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml input_topic:=/lidar_points
```

`input_topic`の明示指定は、センサー別YAMLの`input.topic_names`より優先。
省略時はYAMLのトピック配列をそのまま使用。保存用元点群の`source_point_cloud_topic:=auto`も同じ指定に追従。
[パラメータ適用順の修正と起動検証](../gng_vlut_system/docs/releases/2026-09-23_gng_input_topic_override.md)。

## 把持候補近傍の重点学習（CPU・既定OFF）

`ais_gng.launch.py`へ`enable_grasp_attention:=true`を追加すると、`/grasp_pose_cands/Tmap`のノード近傍へ学習回数の一部を配分。設定・失効条件・観測統計の扱いは[重点学習](docs/grasp_attention.md)を参照。

境界ノード周辺への距離重み付き重点学習は[境界重点学習](docs/boundary_attention.md)を参照。`graspnet.yaml`で有効、把持重点と併用可能。

## CPUのパラメータ反映

`input.voxel_grid_unit: 0.0`はボクセル間引きなし。YAMLの範囲・入力点数上限は引き続き適用。正値ではセル内平均化を適用。

実行中のROS変更に対応する値は`node.learning_num`、`node.interval`、`node.s1_age_max`、`node.clusted_s1_age`、`node.static.age_min`、`node.static.s1_age_max`、`node.eta_decay_rate`、`node.unknown_learning_rate`、`node.s1_reset_range`、`ds.range_max`、`performance.log_interval_ms`。平面クラスタが有効な場合は`plane_cluster.use_node_rho_for_seed_order`も対応。

`node.static.s1_age_max`は長期記憶ノードの未観測・未選択寿命（GNG更新回数、正の整数、既定100）。近傍入力の観測または学習の勝者選択でカウンタをリセットし、削除判定後に1加算。寿命到達時に削除。秒数・学習反復数ではなく、`node.static.age_min: -1`による長期記憶への昇格無効は別設定。新項目の対応はCPUのみ。[対応範囲と検証](../gng_vlut_system/docs/releases/2026-09-23_gng_static_age_parameter.md)。

それ以外のCPU ROSパラメータ変更は再起動必須として拒否。入力範囲・容量・初期学習係数・トピック・機能ON/OFFを変更する場合はYAML編集後にlaunchを再起動。無効値や未対応値を含む一括変更は適用前に拒否。[仕様と検証](../gng_vlut_system/docs/releases/2026-09-23_gng_parameter_application.md)。

## CPU GNGの実行時間ボトルネック

当時計測の処理時間内訳、全ボクセル近傍照合の役割、設定変更比較、二乗メモリ、未検証の改善候補は[ボトルネック調査報告](../gng_vlut_system/docs/designs/gng_runtime_cost_20260923.md)を参照。通常CPU版の条件付き実測であり、現在の設定やSpatialTree実験版の性能とは区別。

## CPUの空間被覆とノード上限

ノード生成候補はボクセル番号順の偏りを避けるため、フレーム番号を種にした順序で全件処理。入力点の追加除外なし。ノード上限到達時の全域被覆・密度は保証されず、YAMLの範囲・間隔・上限が引き続き適用。[原因・回帰検証](../gng_vlut_system/docs/releases/2026-09-23_gng_spatial_coverage.md)。

## CPU直結の平面クラスタリング（既定OFF）

`lidar:=at128.yaml`などで選ぶセンサー別YAMLの`ais_gng_node.ros__parameters`で、Pl・Curveの計算ON/OFFを指定可能。`at128.yaml`には両方falseで明記。

```yaml
ais_gng_node:
  ros__parameters:
    plane_clustering: false # PlのCPU直結平面クラスタ計算
    curve_clustering: false # Curveの別ノード曲面計算
```

優先順位は共通設定、センサー別YAML、対応するlaunch引数の順。省略時は`config/plane_cluster_incremental.yaml`と`config/surface_model.yaml`の共通設定を使用。`nonplane_component.*`もCPUセンサー別YAMLを優先。[設定経路と検証](../gng_vlut_system/docs/releases/2026-09-23_gng_clustering_yaml.md)。

短い2項目は`ais_gng.launch.py`用の設定。内部では既存の`plane_cluster.direct_enabled`・`surface_model.enable`へ変換。センサー別YAMLに旧名もある場合は短い名前を優先。値は引用符なしの`true`／`false`。

`plane_clustering: false`では平面クラスタ計算と、その結果に依存する非平面成分抽出を停止。GNG学習・ノード・エッジ出力は継続。設定反映にはlaunchの再起動が必要。GPU版・独立ノードによる平面計算はこの項目の対象外。

不要な平面可視化ノードの起動も省く場合は`start_plane_cluster:=false`を追加。この引数だけではCPU内蔵の平面計算は停止しないため、YAML設定との併用。

## 曲面検出（既定OFF）

`curve_clustering: false`により、曲面検出・追跡・曲面出力を無効化。GNG学習と平面検出は継続。`ais_gng.launch.py`ではセンサー別YAMLを優先し、未指定時は`config/surface_model.yaml`を使用。設定の反映はlaunchの再起動後。共通設定はCPU・GPU・単独の曲面launchに適用。

曲面が必要な場合は`curve_clustering: true`へ変更し、`start_plane_cluster:=false`を外して再起動。通常のCPU構成では平面クラスタを入力とするため、`plane_clustering: true`も必要。外部の平面入力を使用する構成は別。以下の比較方式も有効化後に利用可能。

## モデル当てはめなしの連続面抽出（比較用）

GNGの位置・法線・実エッジだけで滑らかな連結成分をまとめる方式。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml surface_method:=smooth_graph
```

`surface_method:=model` で従来方式へ復帰。省略時はセンサー別YAMLの`surface_model.method`を優先し、未指定時は`config/surface_model.yaml`の設定（既定`model`）を使用。
新方式は `smooth_surface` と所属を出力し、球・円柱の係数や曲率フィットは出力しない。
実入力で高速化を確認した一方、背景平面まで大きく統合する場合があるため比較用の選択肢。
設定・出力契約・測定値は[仕様と検証記録](../gng_vlut_system/docs/releases/2026-09-15_smooth_surface_graph.md)を参照。

## 物体GNGデータセット保存

`ais_gng.launch.py`は学習nodeと同時に、`/topological_map`の最新GNGを保存する
`object_gng_dataset_exporter_node`を起動する。

```bash
ros2 launch ais_gng ais_gng.launch.py
```

GNGが学習済みの時点で、保存先IDだけを指定する。

```bash
ros2 run ais_gng save_object_gng_dataset mug_complete_v1
```

`/datasets/mug_complete_v1_object_surface_dataset_v1.json`へ、node、edge、cluster、
勝者点群共分散を含む`gng_template`を保存する。`/topological_map`だけでは元の点群座標を
復元できないため、`surface_points`は空配列となる。

## 平面クラスタ未所属nodeの連結成分ID

`config/plane_cluster_incremental.yaml`の次の設定を有効にすると、平面クラスタ未所属nodeを
既存GNG edgeだけで連結成分化し、`/topological_map.nodes[].nonplane_component_id`へ書き込む。

```yaml
ais_gng_node:
  ros__parameters:
    nonplane_component.direct_enabled: true
    nonplane_component.min_component_nodes: 2
    nonplane_component.output_topic: /nonplane_components
```

起動コマンドは通常どおりとする。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml input_topic:=/semantic_points
```

- `NONPLANE_COMPONENT_NONE` (`4294967295`): 平面所属node、または最小node数未満の成分
- `0`以上: `TopologicalMap.nodes`配列内で同じ値を持つnodeの連結成分

成分内edgeは同じIDを持つnode間の`/topological_map.edges`、平面anchor edgeは
`/plane_clusters`のnode添字集合との接続から復元する。持ち手のように複数の平面クラスタを
結ぶ成分は分割しない。

`/nonplane_components`も`std_msgs/UInt32MultiArray`として出力する。座標・edge・平面情報は
複製せず、同じframeの成分所属だけを次の順で格納する。

```text
[frame_number, component_num,
 component_id, node_num, node_index..., ...]
```

`node_index`は同一frameの`/topological_map.nodes`配列への添字である。ToPo-FUZZY Viewerは
`/topological_map`と`/plane_clusters`を参照して、非平面node・成分内edge・平面anchor edgeを
復元表示する。
