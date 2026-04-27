# Multi-Layer Geometric GNG Project

このプロジェクトは、Growing Neural Gas (GNG) アルゴリズムを用いたロボットアームの軌道計画および自己干渉回避システムです。
角度空間と座標空間のデュアルスペース学習、および幾何学的な自己干渉判定を特徴としています。

## 依存ライブラリ (Dependencies)

- **Eigen3**: 解析計算、ベクトル・行列演算に使用。
- **urdfdom** / **urdfdom_headers**: URDFロボットモデルのパースに使用。
- **tinyxml2**: XMLのパースに使用（urdfdomで使用）。
- **console_bridge**: ロギングユーティリティ。
- **ROS 2 Humble**: ROS 2関連のノード、ビルドシステム、可視化に使用。

---

# GNG / VLUT オフライン処理 & ROS 2 ガイド

このドキュメントは、GNG (自己増殖型ニューラルガス) と VLUT (Voxel Look-Up Table) のオフライン学習から、ROS 2環境での実行、可視化までをまとめた総合ガイドです。

## 1. ビルド

まず、`gng_vlut_system` パッケージをビルドします。

```bash
# ワークスペースのルートに移動
cd ~/arm_avoidance 

# ROS 2 環境をセットアップ
source /opt/ros/humble/setup.bash

# ビルド
colcon build --symlink-install --packages-select gng_vlut_system

# ビルド後の環境をセットアップ
source install/setup.bash
```

## 2. オフライン学習

`offline_urdf_trainer` を使って、URDF から GNG と VLUT を一貫して生成します。

### 基本的な学習コマンド

```bash
ros2 run gng_vlut_system offline_urdf_trainer --id my_robot_v1 --res 0.02
```
- `--id`: 実験名を指定します。この名前で結果を保存するディレクトリが作成されます。
- `--res`: VLUTのボクセル解像度 (m) を指定します。

### VLUTのみ再構築

GNGの学習は行わず、既存のGNGマップからVLUTの再構築だけを行いたい場合は `--vlut-only` オプションを使用します。

```bash
ros2 run gng_vlut_system offline_urdf_trainer --id my_robot_v1 --vlut-only --res 0.03
```

### 出力ファイル

学習が完了すると、`gng_results/<id>/` ディレクトリに以下のファイルが生成されます。

- `gng.bin`: 学習済みのGNGマップ。
- `vlut.bin`: 生成されたVLUT (Voxel Look-Up Table)。

### 補助ツール (任意)

生成されたGNGマップを整理・検証するための補助ツールが用意されています。

- **島の除去 (`offline_gng_main6_island_pruning`)**: GNGグラフが分離した「島」を持っている場合に、最大の連結成分のみを残します。
- **データクリーンアップ (`offline_gng_main5_clean`)**: 非アクティブなノードを完全に削除し、ファイルを軽量化します。
- **データ一貫性チェック (`inspect_gng_data`)**: GNGマップ内のデータに矛盾がないか検証します。
- **特徴量再計算 (`offline_gng_status_updater`)**: GNGマップに可操作性などの特徴量を再計算・上書きします。

## 3. ROS 2での実行

### 基本的な可視化

RVizでロボットモデルとGNGの安全状態を可視化します。

```bash
ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py
```

### 安全監視の有効化

GNG/VLUTによる安全監視を有効にして、RVizで確認する場合。

```bash
ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py \
  enable_safety_monitor:=true \
  gng_results_config_path:=gng_results/my_robot_v1/config.txt
```
- `gng_results_config_path`: 使用する学習済みモデルの `config.txt` のパスを指定します。

### 実機/Gazeboとの連携

- **実機**の関節角度を使用する場合:
  ```bash
  ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py joint_state_source:=real
  ```
- **Gazebo**でシミュレーションを行う場合:
  ```bash
  # Gazeboサーバーとロボットを起動
  ros2 launch gng_vlut_system spawn_topoarm_gazebo.launch.py
  
  # 別のターミナルでRVizを起動
  ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py
  ```
  GazeboのGUIを表示したい場合は `gazebo_gui:=true` を追加します。

### ToPoFuzzy-Viewerへのブリッジ

学習したGNGマップやアームの姿勢をToPoFuzzy-Viewerに送信します。

- **GNGマップを送信**:
  ```bash
  ros2 launch gng_vlut_system topofuzzy_bridge.launch.py
  ```
- **アーム姿勢を送信**:
  ```bash
  ros2 launch gng_vlut_system topoarm_viewer_bridge.launch.py
  ```

## 4. テストとデバッグ

### ボクセル色変化テスト

仮想的な占有・危険ボクセルをpublishして、RViz上での色の変化（安全:緑, 危険:黄, 衝突:赤）を確認します。

デフォルトでは、ワールド座標 `(x=20, y=20, z=30)` cm を中心に半径10cmの球が生成されます。

```bash
# デフォルト設定で実行
ros2 run gng_vlut_system voxel_status_test_publisher
```

中心座標や半径、オービット（周回）軌道などを変更したい場合は、パラメータで上書きできます。
```bash
# 例: 半径5cmの球を、Z=40cmの高さで、半径25cmの軌道上を20秒で周回させる
ros2 run gng_vlut_system voxel_status_test_publisher --ros-args \
  -p sphere_radius_cm:=5.0 \
  -p sphere_center_world_cm:="[0.0, 0.0, 40.0]" \
  -p sphere_orbit_radius_cm:=25.0 \
  -p sphere_orbit_period_s:=20.0
```

### 可視化スクリプト

学習結果をプロットして分析します。

- **関節角度の分布**:
  ```bash
  python3 scripts/visualize_joint_angles.py
  ```
- **可操作性の分布**:
  ```bash
  python3 scripts/visualize_manipulability.py --file build/manipulability_data.csv
  ```

## 5. 主要なノードとトピック (リファレンス)

### ノード
- `joint_state_mux_node`: RViz用と実機用の `JointState` を `/joint_states` に集約。
- `safety_monitor_node`: GNG/VLUT の安全状態を `/gng_viz` に表示。
- `self_recognition_viz_node`: 自己認識マスクを `/self_mask_viz` に表示。
- `topofuzzy_bridge_node`: `/topological_map` を Viewer 互換形式で publish。
- ...など

### トピック
- `/joint_states`: 現在のアーム姿勢。
- `/gng_viz`: GNGノードとエッジのマーカー。
- `/self_mask_viz`: 自己認識マスクのマーカー。
- `/occupied_voxels`: 占有ボクセル。
- `/danger_voxels`: 危険ボクセル。
- ...など






最新版



ros2 launch gng_vlut_system gng_vlut_monitor.launch.py \robot_name:=topoarm \dir:=gng_results \id:=topoarm_full_v2 \mode:=STATIC \tag:=topoarm

など
