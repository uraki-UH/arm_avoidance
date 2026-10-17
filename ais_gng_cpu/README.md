# AiS-GNG

点群からのCPU GNG学習・平面／曲面抽出。
前提条件はROS 2 HumbleとCPU版PyTorch。共通環境は[起動手順](../MD/RUN_GUIDE.md)を参照。
SimulatorのCPU専用構成にはAIS-GNGライブ学習を含まない点に注意。

## 環境構築・ビルド

ホストのワークスペースでコンテナを起動:

```bash
docker compose up -d gng_cpu
docker compose exec gng_cpu bash
```

コンテナ内でビルド（依存メッセージ構築済みの環境）:

```bash
cd /ros2_ws
colcon build --packages-select voxel_idx pointcloud_sampling gng_cpu ais_gng \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

依存は[package.xml](src/ais_gng/package.xml)を参照。

## 入力トピックの指定

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml input:=/livox/lidar
```

センサ設定は[gng_cpu設定](src/ais_gng/config/gng_cpu)を参照。
`input`はYAMLの`input.topic_names`より優先し、保存用元点群の`source_point_cloud_topic:=auto`も追従。
変換先の明示指定は`base_tf`。指定時は`input.local_coordinates: false`も適用。
旧launch引数`input_topic`・`base_frame_id`は`input`・`base_tf`へ置換。YAMLのキー名は変更なし。

Simulator点群をworldへ変換する例:

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml \
  input:=/sim/lidar/points base_tf:=world
```

座標契約は[SimulatorのROS連携](../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md)を参照。
異なる入力・出力フレームには接続TFが必要。TF欠落時は学習・出力を抑止し、フレーム名だけの置換なし。
`input.enable_strict_transform: true`では取得時刻のTFを使用。
観測支持・自己点除外が有効な場合も取得時刻、それ以外の非strict構成は最新TF。
TF診断は`--ros-args --log-level ais_gng_node:=debug`。

## 自己点候補の学習除外

通常起動へ`self_mask_topic:=/topo_dual_arm_max_long/self_voxel`を追加。
マスク未指定時は無効。設定はセンサ別YAMLの`self_filter.*`を参照、変更後は再起動。
VLUT用ROIも生成する場合は[ROI登録との共有構成](../gng_vlut_system/README.md#roi登録とais-gngの自己判定共有)を使用可能。
`input.shared_point_store`と単独の`self_filter.mask_topic`は併用不可。

マスクは点群に写る同一機体・姿勢・座標系・時刻との整合が必要。実機点群への別姿勢のGazeboマスク流用は禁止。
マスク未受信・不正・失効、取得時刻TF欠落、全点除外時は学習保留。最新TFへの代替なし。
この保留はロボットの停止指令とは別。

入力は単一PointCloud2、先頭XYZ float32・ホストと同じbyte order。
複数センサは事前に座標・時刻を整合して統合。
任意の`scan/self_labelled`出力は元点順・既存フィールドを保持し、UINT8の`self_candidate`を追加。
値は`0`: マスク外、`1`: 自己候補、`2`: 非有限XYZ。出力には`enable_labelled_cloud`と購読者が必要。

自己セルによる近似判定。取付・姿勢誤差による除去漏れ、近接物体の誤除去、学習済み自己ノードの即時削除は未解決。
CPU版の検証に限定し、実機精度・GPU版の実動作は未検証。
自作検証・ベンチコードはGit管理外、新規checkoutには未同梱。

## 交差点bagの位置・姿勢をYAMLで補正

設定は[intersection_tf.yaml](src/ais_gng/config/intersection_tf.yaml)。`pos`はm、`rot_deg`は度、Euler XYZ順。
初期値は目視による暫定補正。取付校正は別途必要。

```bash
ros2 launch ais_gng intersection_tf.launch.py
```

配信は`world → map`の恒等変換と`map → hesai_lidar`の補正。
別端末のGNGは`input.base_frame_id: map`・`input.local_coordinates: false`で起動。
変更後はTF launch・GNGを再起動し、Viewerの手動変換は位置・回転0、スケール1へリセット。
固定TFを持つ旧`at128.launch.py`との併用は禁止。

## 把持候補近傍の重点学習（CPU・既定OFF）

通常起動へ`enable_grasp_attention:=true`を追加。
設定・失効条件は[重点学習](docs/grasp_attention.md)、境界条件は[境界重点学習](docs/boundary_attention.md)、
CPU学習配分は[共通サンプラー](docs/sampling.md)を参照。

## CPUのパラメータ反映

設定はセンサ別YAMLを参照。実行中の変更対応範囲は[パラメータ適用仕様](../gng_vlut_system/docs/releases/2026-09-23_gng_parameter_application.md)。
未対応値・無効値を含む一括変更は適用前に拒否。容量・入力範囲・トピック・機能切替はYAML編集後に再起動。
`node.static.s1_age_max`の単位はGNG更新回数で、秒・学習反復数とは別。
CPUコアの`node.num_max`は1〜65535、`edge.age_max`は0〜4294967294。
C APIのfloat経由設定では大きな整数の表現精度に制約あり。
ノード上限到達時の全域被覆・密度は保証なし。

## CPUクラスタの所属情報と人・車の分類

`/topological_map`の`clusters[].nodes`は同一メッセージの`nodes[]`への添字で、永続ノードIDとは別。
`label_inferred`は当該フレームの推論、`label`は確認・保持後のラベル。
人・車の分類は平面・曲面計算と独立。条件はセンサ別YAML、
確認・失効の契約は[クラスタラベル仕様](../gng_vlut_system/docs/releases/2026-09-23_gng_cluster_labels.md)を参照。

## CPU直結の平面クラスタリング（既定OFF）

センサ別YAMLの`plane_clustering`で切替。
共通設定は[plane_cluster_incremental.yaml](src/ais_gng/config/plane_cluster_incremental.yaml)。
適用順は共通設定 → センサ別YAML → 対応するlaunch引数。
旧`plane_cluster.direct_enabled`は`plane_clustering`へ置換。

無効時は依存する非平面成分・Publisherも停止し、GNGの学習・ノード・エッジ出力は継続。
外部平面入力・独立再計算は別構成。変更後はlaunchを再起動。

## 曲面検出（既定OFF）

センサ別YAMLの`curve_clustering`で切替。共通設定は[surface_model.yaml](src/ais_gng/config/surface_model.yaml)。
通常のCPU構成では`plane_clustering: true`も必要で、`start_plane_cluster:=false`を外して再起動。
外部平面入力を使う構成は別。曲面無効時もGNG・平面検出は継続。

## モデル当てはめなしの連続面抽出（比較用）

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml surface_method:=smooth_graph
```

`surface_method:=model`で復帰。`smooth_surface`と所属を出力し、球・円柱係数や曲率フィットは対象外。
背景平面まで大きく統合する場合あり。設定・出力契約・評価は[連続面抽出仕様](../gng_vlut_system/docs/releases/2026-09-15_smooth_surface_graph.md)を参照。

## 物体GNGデータセット保存

`ais_gng.launch.py`で学習後、保存先IDを指定:

```bash
ros2 run ais_gng save_object_gng_dataset mug_complete_v1
```

保存先は`/datasets/mug_complete_v1_object_surface_dataset_v1.json`。
node・edge・cluster・勝者共分散を含む`gng_template`を保存。
元点群は`/topological_map`から復元不可のため、`surface_points`は空配列。

## 平面クラスタ未所属nodeの連結成分ID

設定は[plane_cluster_incremental.yaml](src/ais_gng/config/plane_cluster_incremental.yaml)の`nonplane_component.*`。
既存GNG edgeで未所属nodeを連結成分化し、`nodes[].nonplane_component_id`へ出力。
`4294967295`は平面所属または最小node数未満、`0`以上は同一フレーム内の成分ID。
複数平面を結ぶ成分は分割なし。成分内edge・平面anchorは同一フレームのGNG・平面所属から復元。

`/nonplane_components`は`std_msgs/UInt32MultiArray`、格納順:

```text
[frame_number, component_num, component_id, node_num, node_index..., ...]
```

`node_index`は同一フレームの`/topological_map.nodes`への添字。
性能評価は[CPUボトルネック調査](../gng_vlut_system/docs/designs/gng_runtime_cost_20260923.md)、
[本番CPU版の比較](../benchmarks/gng_production_efficiency_20260924/README.md)を参照。
