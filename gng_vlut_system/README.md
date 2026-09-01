# gng_vlut_system

URDFのロボットモデルからGNG (Growing Neural Gas) とVLUT (Voxel Look-Up Table) を
構築するパッケージ。角度空間と座標空間のデュアルスペース学習、および
幾何学的な自己干渉判定が特徴。

## 依存ライブラリ

- **Eigen3**: 解析計算、ベクトル・行列演算。
- **urdfdom** / **urdfdom_headers**: URDFロボットモデルのパース。
- **ROS 2 Humble**: ノード、ビルドシステム。
- **voxel_idx** / **voxel_msgs**: ボクセルID表現とメッセージ定義。

## 1. ビルド

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select voxel_msgs voxel_idx gng_control_msgs gng_vlut_system
source install/setup.bash
```

## 2. オフライン構築

`offline_urdf_trainer` により、URDFからGNGとVLUTを一貫して生成。

```bash
ros2 run gng_vlut_system offline_urdf_trainer --id my_robot_v1 --res 0.02
```

- `--id`: 実験名。この名前で結果ディレクトリを作成。
- `--res`: VLUTのボクセル解像度 (m)。

GNGの学習を行わず、既存のGNGマップからVLUTだけ再構築する場合。

```bash
ros2 run gng_vlut_system offline_urdf_trainer --id my_robot_v1 --vlut-only --res 0.03
```

launch経由の実行は [RUN_GUIDE.md](../RUN_GUIDE.md) を参照。

### 出力ファイル

`gng_results/<id>/` へ生成。

- `gng.bin`: 学習済みGNGマップ。
- `vlut.bin`: 生成されたVLUT。

### GNG後処理

| 実行ファイル | 用途 |
| --- | --- |
| `offline_gng_main5_clean` | 非アクティブノードの完全削除によるファイル軽量化 |
| `offline_gng_main6_island_pruning` | 分離した「島」を除去し最大連結成分のみ保持 |
| `offline_gng_status_updater` | 可操作性などの特徴量の再計算・上書き |

## 3. 実行時のURDF→VLUT

| ノード | 役割 |
| --- | --- |
| `robot_description_player_node` | URDFを`/robot_description`へ配信 |
| `self_recognition_viz_node` | URDFと関節角から自己認識ボクセルを生成しマスクをpublish |
| `voxel_to_vlut_node` | 自己認識ボクセルを`occupied_voxels`/`danger_voxels`へ橋渡し |

### トピック

| トピック | 内容 |
| --- | --- |
| `/joint_states` | 現在のアーム姿勢 |
| `/robot_description` | URDF |
| `/self_recognition/voxel_mask` | 自己認識マスクの送信トピック |
| `/self_mask_viz` | 自己認識マスクのマーカー |
| `/occupied_voxels` | 自己認識ボクセルIDを渡すトピック |
| `/danger_voxels` | occupiedを外側へ膨張したシェル |

## 4. 可視化スクリプト

```bash
python3 scripts/plot_gng_stats.py --file gng_distance_stats.dat
python3 scripts/plot_gng_edge_dist.py <gng.bin>
```
