# fuzzy_classifier

`ais_gng_gpu` が publish する `/topological_map` を受信し、クラスタ情報を集約した JSON を publish するモジュールです。
ノード実装は C++ (`rclcpp`) です。

## I/O

- Subscribe: `/topological_map` (`ais_gng_msgs/msg/TopologicalMap`)
- Publish: `/fuzzy_classifier/cluster_summary` (`std_msgs/msg/String`)
- Publish: `/fuzzy_classifier/cluster_boxes` (`visualization_msgs/msg/MarkerArray`)
- Publish: `/fuzzy_classifier/cluster_nodes` (`visualization_msgs/msg/MarkerArray`)
- Publish: `/fuzzy_classifier/topological_map` (`ais_gng_msgs/msg/TopologicalMap`, `cluster_relabel_node`)

## できること

- ラベル別クラスタ数集計（`label`）
- 信頼度、マッチ率、ノード数、速度、体積の統計値算出
- クラスタ詳細の JSON 出力（必要に応じて上限件数でトリム）
- クラスタの `pos/scale/quat` を使った BOX 描画用 MarkerArray 出力
- クラスタ内 `nodes` 配列を使った対応ノード描画用 MarkerArray 出力
- `ais_gng_gpu` の追加項目（`frame_number`, `label_reliability`, `has_velocity_observation`, `vel_cov_*`）をサマリ JSON に反映

## 実行

`ais_gng_gpu` 側をビルド済みで `ais_gng_msgs` が利用できる状態にした上で実行してください。

#### terminal 1) relabel
```
ros2 launch fuzzy_classifier cluster_relabel.launch.py \
  input_topic:=/topological_map \
  output_topic:=/fuzzy_classifier/topological_map \
  params_file:=/ros2_ws/src/FuzzyClassifier/config/relabel.yaml
```
#### terminal 2) summary
```
ros2 launch fuzzy_classifier cluster_summary.launch.py \
  input_topic:=/fuzzy_classifier/topological_map \
  box_topic:=/fuzzy_classifier/cluster_boxes \
  publish_cluster_boxes:=true
```

### can execute with these below
#### terminal 3
```
docker compose exec gng_gpu bash

ros2 launch ais_gng ais_gng.launch.py
```

#### terminal 4
```
cd /ros2_ws
ros2 bag play /rosbag/macnica/algo_0000_ros2 -l --remap /lidar_points:=/scan --clock
```

#### terminal 5
```
ros2 launch ais_gng rviz.launch.py
```
<!--
### launchで実行

```bash
ros2 launch fuzzy_classifier cluster_summary.launch.py
```

### ラベル張り替えノードを実行

`cluster_relabel.launch.py` はデフォルトで `config/relabel.yaml` を読み込みます。

```bash
ros2 launch fuzzy_classifier cluster_relabel.launch.py \
  output_topic:=/fuzzy_classifier/topological_map \
  human_threshold:=0.8 \
  car_threshold:=0.9
```

別の YAML を使う場合:

```bash
ros2 launch fuzzy_classifier cluster_relabel.launch.py \
  params_file:=/home/hai/src/FuzzyClassifier/config/relabel.yaml
```

`ros2 run` で直接起動する場合:

```bash
ros2 run fuzzy_classifier cluster_relabel_node --ros-args \
  --params-file /home/hai/src/FuzzyClassifier/config/relabel.yaml
```

### Docker (ais_gng_gpuコンテナ) で実行
`/home/hai/src/ais_gng_gpu/docker-compose.yaml` に以下のマウントが必要です。

```yaml
- ../FuzzyClassifier:/ros2_ws/src/FuzzyClassifier
```

起動手順:

```bash
cd /home/hai/src/ais_gng_gpu
docker compose up -d
docker compose exec gng_cpu bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fuzzy_classifier
source /ros2_ws/install/setup.bash
ros2 launch fuzzy_classifier cluster_summary.launch.py
```

## 主なパラメータ

- `input_topic` (default: `/topological_map`)
- `output_topic` (default: `/fuzzy_classifier/cluster_summary`)
- `include_cluster_details` (default: `true`)
- `max_detail_clusters` (default: `200`)
- `moving_speed_threshold_mps` (default: `0.2`)
- `log_interval_sec` (default: `1.0`)
- `pretty_json` (default: `false`)
- `publish_cluster_boxes` (default: `true`)
- `box_topic` (default: `/fuzzy_classifier/cluster_boxes`)
- `box_alpha` (default: `0.35`)
- `render_safe_terrain_box` (default: `false`)
- `publish_cluster_nodes` (default: `false`)
- `node_topic` (default: `/fuzzy_classifier/cluster_nodes`)
- `node_alpha` (default: `0.9`)
- `node_scale` (default: `0.08`)
- `color_label_source` (default: `label`, `label_inferred` 指定時も `label` として扱う)
- `render_safe_terrain_node` (default: `false`)

`cluster_relabel_node` 主要パラメータ:

- `input_topic` (default: `/topological_map`)
- `output_topic` (default: `/fuzzy_classifier/topological_map`)
- `score.min_nodes` (default: `10`)
- `score.human.threshold`, `score.car.threshold`
- `score.human.*_weight`, `score.car.*_weight`（評価関数の重み）
- `rewrite_label`（`rewrite_label_inferred` は互換用パラメータで、`ais_gng_gpu` では実質無効）
- `preserve_structural_labels` (`SAFE_TERRAIN`,`WALL` を保持)

### パラメータ例

```bash
ros2 run fuzzy_classifier cluster_summary_node --ros-args \
  -p include_cluster_details:=false \
  -p moving_speed_threshold_mps:=0.3 \
  -p pretty_json:=true
```

```bash
ros2 launch fuzzy_classifier cluster_summary.launch.py \
  include_cluster_details:=false \
  moving_speed_threshold_mps:=0.3 \
  pretty_json:=true \
  color_label_source:=label \
  publish_cluster_boxes:=true \
  publish_cluster_nodes:=true \
  box_alpha:=0.5 \
  node_scale:=0.1
```

## RViz表示
RViz2 で `MarkerArray` 表示を追加し、必要に応じて以下を設定してください。

- BOX: `/fuzzy_classifier/cluster_boxes`
- ノード: `/fuzzy_classifier/cluster_nodes`

-->
