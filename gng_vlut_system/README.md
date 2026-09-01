# gng_vlut_system

URDFのロボットモデルから、自己認識ボクセル・VLUT (Voxel Look-Up Table)・GNG (Growing Neural Gas)
を生成するオフライン学習パッケージ。角度空間と座標空間のデュアルスペース学習、および
幾何学的な自己干渉判定が特徴。

## 詳細仕様

- [docs/TECHNICAL_SPEC.md](docs/TECHNICAL_SPEC.md)
- [docs/RELEASE_NOTE_TEMPLATE.md](docs/RELEASE_NOTE_TEMPLATE.md)
- [docs/releases/](docs/releases/)

> docs配下には、本ブランチで削除した実行時・Viewer連携機能の記述も履歴として残存。

## 依存ライブラリ

- **Eigen3**: 解析計算、ベクトル・行列演算。
- **urdfdom** / **urdfdom_headers**: URDFロボットモデルのパース。
- **tinyxml2**: XMLのパース（urdfdomが使用）。
- **ROS 2 Humble**: ノード、ビルドシステム、可視化。
- **voxel_idx** / **voxel_msgs**: ボクセルID表現とメッセージ定義。

## 1. ビルド

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select voxel_msgs voxel_idx gng_control_msgs gng_vlut_system
source install/setup.bash
```

## 2. オフライン学習

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

### 後処理ツール

| 実行ファイル | 用途 |
| --- | --- |
| `offline_gng_main5_clean` | 非アクティブノードの完全削除によるファイル軽量化 |
| `offline_gng_main6_island_pruning` | 分離した「島」を除去し最大連結成分のみ保持 |
| `offline_gng_status_updater` | 可操作性などの特徴量の再計算・上書き |
| `visualization_gng_trainer` | 可視化専用の3次元GNGと圧縮永続化形式の生成 |

### 検証ツール

| 実行ファイル | 用途 |
| --- | --- |
| `voxel_spherizer_preview` | URDFリンクの球体近似結果のプレビュー |
| `voxel_link_mask_validator` | リンク単位ボクセルマスクの検証 |
| `voxel_status_test_publisher` | 仮想的な占有・危険ボクセルのpublishによる色変化確認 |

## 3. 主要なノードとトピック

### ノード

| ノード | 役割 |
| --- | --- |
| `robot_description_player_node` | URDFを`/robot_description`へ配信 |
| `initial_joint_state_publisher_node` | 初期姿勢の`JointState`配信 |
| `rviz_robot_visual_marker_node` | RVizへのロボット外形マーカー表示 |
| `self_recognition_viz_node` | URDFから自己認識ボクセルを生成しマスクをpublish |
| `self_recognition_filter_node` | 自己認識ボクセル内外で点群を分離 |
| `self_voxel_filter_node` | 自己認識ボクセルによる点群フィルタ |
| `voxel_spherized_robot_viewer_node` | 球体近似したロボット形状の可視化 |
| `voxel_to_vlut_node` | 自己認識ボクセルを`occupied_voxels`/`danger_voxels`へ橋渡し |

### トピック

| トピック | 内容 |
| --- | --- |
| `/joint_states` | 現在のアーム姿勢 |
| `/self_mask_viz` | 自己認識マスクのマーカー |
| `/self_recognition/voxel_mask` | 自己認識マスクの送信トピック |
| `/self_recognition_points` | 自己認識ボクセル内の点群 |
| `/self_filtered_points` | 自己認識ボクセル外の点群 |
| `/occupied_voxels` | 自己認識ボクセルIDを安全監視へ渡すトピック |
| `/danger_voxels` | occupiedを外側へ膨張したシェル |

## 4. 可視化スクリプト

```bash
python3 scripts/plot_gng_stats.py --file gng_distance_stats.dat
python3 scripts/plot_gng_edge_dist.py <gng.bin>
```
