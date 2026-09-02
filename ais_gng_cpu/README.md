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
