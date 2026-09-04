# 完全表面mesh GNGテンプレート

## 目的

軽自動車、普通自動車、トラック、バイク、自転車、船の3D meshから、内部点を含まない完全表面点群を
生成し、`ais_gng` CPU backendでGNGテンプレートを学習する手順。

実LiDAR点群はテンプレート生成には不要である。環境側との照合時だけ、単視点depth点群の欠損を持つ
観測GNGと、ここで作成した完全表面テンプレートGNGを照合する。

## 事前準備

対応するmesh形式は`OBJ`、`STL`、binary little-endian `PLY`。`--mesh`は複数回指定でき、同じ座標系の
複数部品を1つの完全表面テンプレートへ結合する。配布元のライセンスを確認したmeshだけを用意する。mesh本体は
workspaceへ同梱しない。カテゴリごとのID、想定寸法、mesh指定欄は
[`mesh_surface_template_catalog.yaml`](../config/mesh_surface_template_catalog.yaml)にまとめてある。

`unit_scale`はmesh座標をmへ換算する倍率である。mm単位のmeshは`0.001`、cm単位のmeshは`0.01`を指定する。
既定ではmeshのXY中心を原点、最下部をZ=0へ正規化する。元の座標系を維持する場合だけ`--keep-mesh-pose`を付ける。

## Docker手順

以下の`<container>`は`ais_gng`をビルド済みのDockerコンテナ名へ置き換える。初回だけ設定ファイルを
install領域へ反映するため、コンテナ内で再ビルドする。

```bash
docker exec -it <container> bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --packages-select ais_gng
source install/setup.bash
```

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
  dataset_id:=car_surface
```

配信topicは`/car_surface/topological_map_static`。`frame_id`は既定で`object_template`。

## カテゴリID

| 対象 | template_id |
|---|---|
| 軽自動車 | `kei_car_surface` |
| 普通自動車 | `car_surface` |
| トラック | `truck_surface` |
| バイク | `motorcycle_surface` |
| 自転車 | `bicycle_surface` |
| 船 | `boat_surface` |

## 生成仕様

- 三角形選択: 面積比例の乱数選択
- 面内サンプル: 一様な重心座標サンプル
- 生成点: 表面のみ。voxel充填や内部点生成なし
- 既定点数: 50,000点
- 既定frame: `object_template`
- GNG入力topic: `/mesh_template_surface`
- GNG設定: `ais_gng/config/gng_cpu/mesh_surface.yaml`
- 共分散: `node.covariance_enabled: true`

`mesh_surface.yaml`の入力範囲はX/Y `-15..15 m`、Z `-2..8 m`。この範囲を超えるmeshでは、
設定ファイルの範囲と`input.voxel_grid_unit`をセットで見直す。CPU GNGは範囲とvoxel単位から内部indexを
確保するため、必要以上に大きな範囲を指定しない。

meshの穴、裏返り、非多様体面は補正しない。登録前にmesh viewerで表面が意図どおり閉じていることを確認する。
