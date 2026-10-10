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

`input_topic`の明示指定は、センサ別YAMLの`input.topic_names`より優先。
省略時はYAMLのトピック配列をそのまま使用。保存用元点群の`source_point_cloud_topic:=auto`も同じ指定に追従。
[パラメータ適用順の修正と起動検証](../gng_vlut_system/docs/releases/2026-09-23_gng_input_topic_override.md)。

点群のトピックと座標系は別設定。`input_topic`だけの変更では、YAMLの`input.base_frame_id`も維持。
変換先を変更する場合は`base_frame_id`を指定。この明示指定は`input.local_coordinates: false`も適用。
ブラウザシミュレータの`/sim/lidar/points`は、MID-360の点を`base_footprint`座標で配信。
Viewerとシミュレータの共通基準は`world`。AT128用の学習パラメータで`world`座標へ変換する起動例:

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml \
  input_topic:=/sim/lidar/points base_frame_id:=world
```

入力と出力の座標系が同じ場合はTF不要。異なる場合は、両座標系を接続するTFが必要。
TF取得失敗時は、その入力組の学習・出力を抑止。変換前の座標を別フレーム名で配信するフォールバックなし。
`input.enable_strict_transform: true`では点群取得時刻、既定の`false`では通常は最新のTFを使用。
CPUの観測支持機能、または自己点除外が有効な場合は、`false`でも点群取得時刻のTFを使用。
シミュレータの`world → base_footprint`のTF配信が必要。Viewerの表示基準は`world`。
`base_frame_id:=base_footprint`はロボット基準での処理を意図する場合の指定であり、Viewerの基準変更ではない。

getBase2LidarFrameのTF取得失敗メッセージ`Could not transform ...`はDEBUGログ。通常起動での警告行の割込みなし。ノード直接起動時の`--ros-args --log-level ais_gng_node:=debug`で診断可能。[表示変更・検証](../gng_vlut_system/docs/releases/2026-09-30_gng_tf_log.md)。

## 自己点候補の学習除外

VLUT用ROIを同時生成する場合は、[ROI登録との共有構成](../gng_vlut_system/README.md#roi登録とais-gngの自己判定共有)を選択可能。直接ROI登録、または一回だけ構築したworld_bucketからのROI登録。`input.shared_point_store`で同一プロセスの元点・セルラベルを参照し、GNG側の点群購読・自己形状照合を省略。入力格子も共通登録結果の受渡しによる内部再量子化・再ソートの省略、学習範囲・代表元点・外部サンプラーの維持。単独の`self_filter.mask_topic`との併用不可。以下は単独GNG用の設定。

既存の自己ボクセルマスクで元点を判定し、抽出候補から除外。環境点群の粗いグリッドを別途作成せず、残った実測点を既存のGNGボクセル処理へ投入。通常・unknown・重点学習のすべてが除外後の入力を使用。

GNGを停止した状態で、Docker内のワークスペースへ反映します。

```bash
cd /ros2_ws
colcon build --packages-select voxel_idx pointcloud_sampling gng_cpu ais_gng --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

既存の`ais_gng.launch.py`起動コマンドへ次の引数を追加します。

```bash
self_mask_topic:=/topo_dual_arm_max_long/self_voxel
```

マスク配信は既存のViewer／自己認識側が担当。点群に写る同一機体の姿勢・座標系・時刻との整合が必要です。実機点群へGazeboの別姿勢マスクを流用しないでください。

センサ別YAMLの`ais_gng_node.ros__parameters`でも設定可能。設定変更は再起動後に反映。

```yaml
self_filter.mask_topic: /topo_dual_arm_max_long/self_voxel
self_filter.max_mask_age_sec: 0.5
self_filter.enable_labelled_cloud: false
```

- 有効化: `mask_topic`指定時。未指定の既定はOFF。launch引数はYAMLより優先。
- 照合: センサ座標の粗い外接箱 → 必要点のみ座標変換 → 自己セル照合。粗い箱には丸め誤差用の余白、最終判定のセル膨張なし。自己領域のビット列は上限2 MiB、広い領域はハッシュ方式。マスク内容が同一なら索引を再利用。
- ランダム抽出: 少数抽出時は重複なしの候補判定、必要数到達時に終了。大量抽出・大量除外時は連続走査の判定キャッシュへ切替。有効な非自己点が不足する場合は、その全点を返却。
- 点群走査: XYZ有効性検査と自己判定を共通化。添字領域は入力1点あたり4 byte、判定キャッシュ使用時は追加1 byte。全点×全リンク探索・全点XYZ複製なし。stratified／head／uniformと全点ラベル出力は全点判定、抽出後の点群コピーは従来の投入用バッファを再利用。
- 保護: マスク未受信・不正・失効、取得時刻のTF欠落、全点除外時は学習保留。期限は受信経過・点群とマスクの時刻差・現在時刻との差に適用。保留時の残存入力による学習、最新TFへの代替なし。
- ラベル出力: `enable_labelled_cloud: true`かつ購読者がいる場合のみ`scan/self_labelled`へ配信。元点の順序・座標系・時刻・既存フィールドを保持し、UINT8の`self_candidate`を追加。`0`: マスク外、`1`: 自己候補、`2`: 非有限XYZ。同じ入力・seedではラベル出力の有無によらず同じ学習点・順序。OFF／購読なしではラベル配列・全点群複製なし。
- 対応入力: 単一PointCloud2トピック、先頭XYZ float32・ホストと同じbyte order。head／uniform／random／stratifiedに対応。複数センサは事前に座標・時刻を整合した単一点群へ統合。
- 制限: 受信した自己セルとの幾何照合であり、意味認識・確定ラベルではありません。追加膨張なし。姿勢や取付校正の誤差による除去漏れ・近接物体の誤除去、過去に学習済みの自己ノードの即時削除は未解決。ロボットの停止指令ではありません。

検証コード: ローカル保持・Git管理外、新規checkoutには未同梱。未配置時の通常ビルドは継続。

検証用: `test_self_point_filter`（単体）、`test/check_self_point_filter_ros.py`（専用domain194・`--node-executable`／`--output`指定）、`benchmark_self_point_filter`（合成点群の前処理比較）。CPU版での検証、実機精度・GPU版の実動作は未検証。

計測引数: `benchmark_self_point_filter off|filter|labels SEED SELF_PERCENT MAX_POINTS METRICS_JSON`。入力30.7万点、抽出上限の指定、前処理のみの測定。ROS通信・GNG本体・マスク生成は対象外。

## 交差点bagの位置・姿勢をYAMLで補正

[intersection_tf.yaml](src/ais_gng/config/intersection_tf.yaml)の`pos`（m）と`rot_deg`（度）を編集。
初期値は`pos: [0, 0, 6]`、`rot_deg: [3, 13, 3]`。ToPoFuzzy Viewerと同じEuler XYZ順の目視による暫定補正。

```bash
ros2 launch ais_gng intersection_tf.launch.py
```

このlaunchは`world → map`の恒等変換と`map → hesai_lidar`の補正だけを配信。
TFの配信開始後、別ターミナルで上記の`ais_gng.launch.py`を起動。GNGは`input.base_frame_id: map`、`input.local_coordinates: false`を使用。
補正前のGNGが起動中の場合は再起動してグラフを初期化し、Viewerの点群・GNG両方の手動変換を位置・回転0、スケール1へリセット。Viewerの基準フレームは`world`または`map`。
YAMLの変更はTF launchとGNGの再起動で反映。固定TFを持つ旧`at128.launch.py`との併用は不可。
別のYAMLは`config_file:=/path/to/config.yaml`で指定可能。[検証結果](../gng_vlut_system/docs/releases/2026-09-25_intersection_tf_yaml.md)。

## 把持候補近傍の重点学習（CPU・既定OFF）

`ais_gng.launch.py`へ`enable_grasp_attention:=true`を追加すると、`/grasp_pose_cands/Tmap`のノード近傍へ学習回数の一部を配分。設定・失効条件・観測統計の扱いは[重点学習](docs/grasp_attention.md)を参照。

境界ノード周辺への距離重み付き重点学習は[境界重点学習](docs/boundary_attention.md)を参照。`graspnet.yaml`で有効、把持重点と併用可能。

CPUの学習配分と追加条件の評価は[共通サンプラー](docs/sampling.md)へ統合。ラベル・平面サイズ・密度条件はセル評価器として追加可能。非平面などの新しい本番条件は未追加。

追加の非平面重点学習は2026-09-25に撤去。`/downsampling/nonplane`と関連設定は廃止。
既存unknown重点学習・通常エイジング・非平面成分出力は継続。[変更内容](../gng_vlut_system/docs/releases/2026-09-25_nonplane_attention.md)。

## CPUのパラメータ反映

`input.voxel_grid_unit: 0.0`はボクセル間引きなし。YAMLの範囲・入力点数上限は引き続き適用。正値ではセル内平均化を適用。

実行中のROS変更に対応する値は`node.learning_num`、`node.interval`、`node.s1_age_max`、`node.clusted_s1_age`、`node.static.age_min`、`node.static.s1_age_max`、`node.eta_decay_rate`、`node.unknown_learning_rate`、`node.s1_reset_range`、`ds.range_max`、`performance.log_interval_ms`。平面クラスタが有効な場合は`plane_cluster.use_node_rho_for_seed_order`も対応。

`node.static.s1_age_max`は長期記憶ノードの未観測・未選択寿命（GNG更新回数、正の整数、既定100）。近傍入力の観測または学習の勝者選択でカウンタをリセットし、削除判定後に1加算。寿命到達時に削除。秒数・学習反復数ではなく、`node.static.age_min: -1`による長期記憶への昇格無効は別設定。新項目の対応はCPUのみ。[対応範囲と検証](../gng_vlut_system/docs/releases/2026-09-23_gng_static_age_parameter.md)。

それ以外のCPU ROSパラメータ変更は再起動必須として拒否。入力範囲・容量・初期学習係数・トピック・機能ON/OFFを変更する場合はYAML編集後にlaunchを再起動。無効値や未対応値を含む一括変更は適用前に拒否。[仕様と検証](../gng_vlut_system/docs/releases/2026-09-23_gng_parameter_application.md)。

## CPU GNGの実行時間ボトルネック

本番CPU版は、最小空きノードIDの探索開始位置を保持し、入力voxelの区間確定と重心計算を一度の走査に統合。既存の入力順・加算順・学習回数・全点照合を保持。これらの[変更と出力一致検証](../benchmarks/gng_production_efficiency_20260924/README.md)の後、基数ソートも本番CPU版へ採用。[複数シードでの評価と制限](../benchmarks/gng_radix_multiseed_20260924/README.md)。

当時計測の処理時間内訳、全ボクセル近傍照合の役割、設定変更比較、二乗メモリ、未検証の改善候補は[ボトルネック調査報告](../gng_vlut_system/docs/designs/gng_runtime_cost_20260923.md)を参照。通常CPU版の条件付き実測であり、現在の設定やSpatialTree実験版の性能とは区別。

## CPUの空間被覆とノード上限

ノード生成候補はボクセル番号順の偏りを避けるため、フレーム番号を種にした順序で全件処理。入力点の追加除外なし。ノード上限到達時の全域被覆・密度は保証されず、YAMLの範囲・間隔・上限が引き続き適用。[原因・回帰検証](../gng_vlut_system/docs/releases/2026-09-23_gng_spatial_coverage.md)。

## CPUクラスタの所属情報と人・車の分類

通常CPU版のクラスタ出力は、実際の所属ノード数と所属配列をROSへ転送。`/topological_map`の`clusters[].nodes`は同じメッセージの`nodes[]`への添字であり、永続ノードIDではない。

`classify.human`・`classify.car`で有効な分類器は、所属30ノード以上などの既存条件を満たすクラスタを入力として使用。Pl・CurveのON/OFFとは別機能。所属数が0で渡されて全件除外される不具合を修正済み。[変更範囲・回帰検証](../gng_vlut_system/docs/releases/2026-09-23_gng_cluster_members.md)。

`clusters[].label_inferred`は当該フレームの推論結果、`clusters[].label`は確認・保持処理後のラベル。通常CPU版では同じクラスの推論を`cluster.human.confirmation_age` / `cluster.car.confirmation_age`の回数だけ確認後、次のGNG更新から確定ラベルへ反映。同一フレームの重複結果の加算なし。

推論途絶時は`cluster.human.hysteresis_age` / `cluster.car.hysteresis_age`フレーム分を保持。保持期間内の短い途絶では確認回数を維持し、期限切れまたは人・車の切替時にはリセット。生成フレーム番号と年齢の取り違え、保持条件の逆転を修正済み。[仕様・検証範囲](../gng_vlut_system/docs/releases/2026-09-23_gng_cluster_labels.md)。

## CPU直結の平面クラスタリング（既定OFF）

`lidar:=at128.yaml`などで選ぶセンサ別YAMLの`ais_gng_node.ros__parameters`で、Pl・Curveの計算ON/OFFを指定可能。`at128.yaml`には両方falseで明記。

```yaml
ais_gng_node:
  ros__parameters:
    plane_clustering: false # PlのCPU直結平面クラスタ計算
    curve_clustering: false # Curveの別ノード曲面計算
```

優先順位は共通設定、センサ別YAML、対応するlaunch引数の順。省略時は`config/plane_cluster_incremental.yaml`と`config/surface_model.yaml`の共通設定を使用。`nonplane_component.*`もCPUセンサ別YAMLを優先。[設定経路と検証](../gng_vlut_system/docs/releases/2026-09-23_gng_clustering_yaml.md)。

`plane_clustering`はCPUノードでも直接指定できる平面計算の切替。共通設定よりセンサ別YAMLを優先。`curve_clustering`はlaunch側で`surface_model.enable`へ変換し、併記時は前者を優先。値は引用符なしの`true`／`false`。旧`plane_cluster.direct_enabled`は廃止のため`plane_clustering`へ置換が必要。

`plane_clustering: false`ではCPU直結の平面クラスタ計算と、その結果に依存する非平面成分抽出・Publisherを停止。通常の自動入力構成では平面可視化・曲面ノードも起動せず、保存ノードの平面購読も無効化。GNG学習・`/topological_map`のノード・エッジ出力は継続。GPU構成では独立平面ノードの起動条件へ適用。設定反映にはlaunchの再起動が必要。

OFF時に`start_plane_cluster:=false`を追加する必要なし。平面計算ONのまま可視化・曲面ノードだけを止める場合は引き続き利用可能。`plane_clusters_input_topic`を明示した外部平面入力・独立再計算は自動起動抑止の対象外。[OFF時の通信口抑止と検証](../gng_vlut_system/docs/releases/2026-09-23_gng_clustering_topics.md)。

## 曲面検出（既定OFF）

`curve_clustering: false`により、曲面検出・追跡・曲面出力を無効化。GNG学習と平面検出は継続。`ais_gng.launch.py`ではセンサ別YAMLを優先し、未指定時は`config/surface_model.yaml`を使用。設定の反映はlaunchの再起動後。共通設定はCPU・GPU・単独の曲面launchに適用。

曲面OFFまたは曲面ノード未起動時はGNG側の`/curved_surface_clusters/update_ms`購読も未生成。Viewerの補助平面購読は発行元の存在中だけ有効。Viewer更新前の既存プロセスにはViewerの再起動も必要。他の独立ノードによる同名トピックの購読・発行は停止対象外。

曲面が必要な場合は`curve_clustering: true`へ変更し、`start_plane_cluster:=false`を外して再起動。通常のCPU構成では平面クラスタを入力とするため、`plane_clustering: true`も必要。外部の平面入力を使用する構成は別。以下の比較方式も有効化後に利用可能。

## モデル当てはめなしの連続面抽出（比較用）

GNGの位置・法線・実エッジだけで滑らかな連結成分をまとめる方式。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml surface_method:=smooth_graph
```

`surface_method:=model` で従来方式へ復帰。省略時はセンサ別YAMLの`surface_model.method`を優先し、未指定時は`config/surface_model.yaml`の設定（既定`model`）を使用。
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
