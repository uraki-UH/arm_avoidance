# 完全表面mesh GNGテンプレート

## 目的

軽自動車、普通自動車、トラック、バイク、自転車、船の3D meshから、内部点を含まない完全表面点群を
生成し、`ais_gng` CPU backendでGNGテンプレートを学習する手順。

完全表面はテンプレート生成用。照合時は欠損を含む観測GNGと比較。
対応形式・引数は[mesh_surface_template.py](../tools/mesh_surface_template.py)の`--help`を参照。利用条件を確認済みのmeshを別途用意。
meshの単位をmへ換算し、既定でXY中心・最下部Z=0へ正規化。元座標を保持する場合は`--keep-mesh-pose`。

## Docker手順

`ais_gng` CPU backendをビルド・source済みのコンテナで実行。

### 1. 完全表面点群の生成

次は普通自動車の例である。カテゴリごとに`--mesh`、`--template-id`、`--display-name`を変更する。

```bash
python3 /ros2_ws/src/gng_vlut_system/tools/mesh_surface_template.py prepare \
  --mesh /datasets/meshes/car.obj \
  --template-id car_surface \
  --display-name "普通自動車" \
  --output-dir /datasets \
  --point-count 50000
```

生成物は次の2件。

- `/datasets/car_surface_object_surface_dataset_v1.json`: 全表面点群、mesh hash、座標正規化、サンプル条件
- `/datasets/car_surface_surface.pcd`: 外部確認用の同一点群

### 2. CPU GNG学習

端末Aで`ais_gng`を起動する。完全表面テンプレートは局所座標で扱うためTFは不要。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch ais_gng ais_gng.launch.py \
  backend:=cpu \
  lidar:=mesh_surface.yaml \
  input_topic:=/mesh_template_surface
```

端末Bで同じ表面点群を10 Hzで連続配信する。GNGの学習完了まで継続する。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
python3 /ros2_ws/src/gng_vlut_system/tools/mesh_surface_template.py publish \
  --dataset /datasets/car_surface_object_surface_dataset_v1.json \
  --topic /mesh_template_surface \
  --rate-hz 10
```

### 3. 学習済みGNGの保存と統合

学習後、端末Cで保存する。`--replace`により同じIDの古いGNG JSONを固定名で置換する。
完全表面点群は先に生成したJSONに保持済みのため、`--with-points`は不要である。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 run ais_gng save_object_gng_dataset car_surface --replace

python3 /ros2_ws/src/gng_vlut_system/tools/mesh_surface_template.py merge-gng \
  --surface-dataset /datasets/car_surface_object_surface_dataset_v1.json \
  --gng-dataset /datasets/car_surface_gng_template.json.gz \
  --output /datasets/car_surface_object_surface_dataset_v1.json \
  --replace
```

`merge-gng`はtemplate IDの不一致、空GNG、空の表面点群をエラーにする。統合後のJSONにはGNG node、edge、
勝者点群誤差共分散、node cluster、plane clusterと、元の完全表面点群が同居する。

### 4. 静的テンプレートの召喚

統合済みデータセットは既存launchでそのまま召喚できる。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system object_template_map_publisher.launch.py \
  dataset_dir:=/datasets \
  dataset_file:=car_surface_object_surface_dataset_v1.json
```

配信topicは`/car_surface/Tmap_static`。`frame_id`は既定で`object_template`。

## カテゴリID

| 対象 | template_id |
|---|---|
| 軽自動車 | `kei_car_surface` |
| 普通自動車 | `car_surface` |
| トラック | `truck_surface` |
| バイク | `motorcycle_surface` |
| 自転車 | `bicycle_surface` |
| 船 | `boat_surface` |

## 生成仕様・制限

三角形を面積比例で選び、面内を一様サンプル。内部点・voxel充填なし。
GNG設定は`mesh_surface.yaml`。入力範囲・voxel幅はmesh寸法に合わせて調整。過大な範囲は内部indexの確保量に影響。
meshの穴・裏返り・非多様体面の自動補正なし。
