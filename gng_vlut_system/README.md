# Multi-Layer Geometric GNG Project

ロボット姿勢GNG・VLUTによる計画／干渉判定。既定機体はLong。
[起動手順](../MD/RUN_GUIDE.md)・[共通ROS回避](docs/dual_arm_simulation.md#センシング--ros回避--実行先)を参照。

## 前提条件

ROS 2 Humble、Eigen3、urdfdom、tinyxml2、console_bridge。
共通環境は[起動手順](../MD/RUN_GUIDE.md)、依存は[package.xml](package.xml)を参照。

## 1. ビルド

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gng_vlut_system --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 2. オフライン学習

URDFからのGNG・ノードVLUT構築。既定モデルは`topo_dual_arm_max_long`。

### 基本的な学習コマンド

```bash
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py experiment_id:=my_robot_v1
```

設定は[機体YAML](config/topo_dual_arm_max_long.yaml)を参照。

### VLUTのみ再構築

同じ保存先の既存GNGを使用した、VLUTだけの再構築:

```bash
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py experiment_id:=my_robot_v1 vlut_only:=true
```

### 出力ファイル

保存先: `gng.data_directory/<experiment_id>/`。独立腕設定では各プロファイルのサブディレクトリ。

- `gng.bin`: 学習済みのGNGマップ。
- `vlut.bin`: 生成されたVLUT (Voxel Look-Up Table)。

保存形式はGNG v10。旧形式は通常起動では受理せず、以下の移行が必要。
索引・可操作性の内部表現は[実装仕様](docs/TECHNICAL_SPEC.md)を参照。

旧形式v5〜v9の移行。元データは未作成ディレクトリへ退避。
可操作性値は無効化されるため、実行時の再計算または再学習が必要。VLUT再生成は不要。

```bash
python3 gng_vlut_system/tools/migrate_gng_v10.py gng_vlut_system/gng_results \
  --backup-dir log/artifacts/gng_v10_import/originals
```

コンテナ所有の学習データは同じコマンドをコンテナ内の`/ros2_ws/src`で実行。移行後は`gng_vlut_system`を再ビルドし、利用中の関連ノードを再起動してください。新形式の既存ファイルは内容を保持。

### 任意の辺掃引バックエンド

既定: `vlut_backend:=node_vlut`。従来の学習・保存形式・ノード判定の維持。追加方式は明示指定時のみ。

```bash
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py \
  experiment_id:=my_robot_v1 vlut_only:=true vlut_backend:=edge_vlut
```

追加成果物: 各保存先の`edge_vlut_recipe.json`と`edge_vlut.npz`。既存の`gng.bin`・`vlut.bin`とは別保存。上記launchでは従来構築後に追加索引を生成し、追加成果物だけを更新。追加構築の失敗はlaunchエラー。

レシピから追加索引だけの構築、または格子条件の変更:

```bash
ros2 run gng_vlut_system gng_edge_vlut.py \
  --input /ros2_ws/src/gng_vlut_system/gng_results/my_robot_v1 \
  --voxel-size 0.05 --clearance-margin 0.02
```

既存追加索引の上書きは`--allow-replace`指定時のみ。URDF配置の変更は`--urdf`、計画グループの変更は`--avoidance-config`で、実行時と同じ`dual_arm_avoidance_demo`設定を指定。自動構築launchにも`avoidance_config`指定口あり。

判定は`clear`・`blocked`・`unknown`。境界・未構築辺・条件不一致は従来の区間検査へ委任。
全関節端点・形状・計画リンク・固定環境・入力セル半径・要求余裕が実行条件と一致すること。
固定関節・非対象腕の姿勢変更も照合対象。予算・構築オプションは[gng_edge_vlut.py](scripts/gng_edge_vlut.py)の`--help`を参照。

ROS側の選択口: `dual_arm_gng_lidar_demo.launch.py`／`dual_arm_gazebo_demo.launch.py`の`vlut_backend:=edge_vlut edge_vlut_file:=<索引の絶対パス>`、または回避設定YAML内の同名項目。既存ノード安全判定は継続し、確定衝突辺だけ探索から早期除外。`vlut_backend:=node_vlut`で従来方式へ復帰。

検証範囲: Longの小規模学習launch・旧バイナリ一致・19関節形状の7軸／14軸構築・ROSノード選択。ブラウザ独立腕計画・Harmonic入口・Gazebo物理を含む通し回避・実機は未検証または未接続。動的障害物の予測、実機追従誤差、未知領域の安全保証は対象外。初回読込み・環境更新の追加費用があるため、全条件での高速化保証なし。

### 3D占有に基づく領域ノード

任意の試験バックエンド。構築・状態判定・被覆保証の範囲・比較条件は[姿勢領域VLUT仕様](docs/TECHNICAL_SPEC.md#18-姿勢領域vlut)。
既存GNGの学習則・保存形式・既定バックエンドは維持。全可行姿勢の安全証明・実機保証は未達。

### 補助ツール (任意)

島の除去は`offline_gng_main6_island_pruning`、非アクティブノード削除は`offline_gng_main5_clean`、
整合性検査は`inspect_gng_data`、特徴量再計算は`offline_gng_status_updater`。
起動設定は[launch/](launch/)を参照。

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
  enable_safety_monitor:=true
```

### 実機/Gazeboとの連携

- **実機**の関節角度を使用する場合:
  ```bash
  ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py joint_state_source:=real
  ```
- **Gazebo**でシミュレーションを行う場合:
  ```bash
  # Gazeboサーバーとロボットを起動
  ros2 launch gng_vlut_system robot_gazebo_sim.launch.py robot_name:=ToPoDualArm

  # 別のターミナルでRVizを起動
  ros2 launch gng_vlut_system visualize_topoarm_rviz.launch.py
  ```
  Docker内でGUIが不要なら `gui:=false` にできます。

### FuzzBotの移動状態GNG

機体パッケージは`robots/fuzzbot/`、状態学習・遷移グラフは本パッケージへ分離。
旧構想の[effectivity_map_gng](experimental/effectivity_map_gng/)はコメントのみで未実装、通常ビルド対象外。

初期位置・前方を基準とする状態`(x, y, 相対yaw, v, omega)`の学習とViewer表示。設定は[fuzzbot.yaml](config/fuzzbot.yaml)の`mobile_gng`・`motion_graph`。
平坦床・横滑りなしの差動二輪モデル。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=fuzzbot.yaml
```

再学習だけの場合は`enable_rebuild:=true enable_build_only:=true`を追加。
表示用normalは車体前方`(cos yaw, sin yaw, 0)`で、表面法線とは別。速度はモデル値。
表示原点の変更は学習済み状態へ影響なし。yawの表示範囲は相対角、下限が上限を超える場合は±180°をまたぐ区間。

保存先は`gng_results/fuzzbot/state_gng.json`。ROSなしの確認用HTML生成:

```bash
ros2 run gng_vlut_system mobile_gng_preview.py \
  /ros2_ws/src/gng_vlut_system/gng_results/fuzzbot/state_gng.json \
  /ros2_ws/src/log/artifacts/fuzzbot_state_gng/preview.html
```

辺は状態空間の近傍で、走行可能な制御入力の保証なし。
車体・掃引VLUT、環境衝突判定、odom追従、経路探索・実機走行は未接続。
`mobile_state_graph.launch.py`の運動制約付き遷移グラフとは別データ。

### 双腕学習・Viewerの既定モデル

```bash
ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py
```

既定はLong。設定は[topo_dual_arm_max_long.yaml](config/topo_dual_arm_max_long.yaml)を参照。
左右独立モデルの保存先は`gng_results/topo_dual_arm_max_long_independent/`。
manifestはindependent_arms.json、各腕はgng.bin・vlut.bin・model.json。
別機体はparams_fileを変更。データはURDF・固定関節条件と一致させること。

LongのViewer入口は共通の実測変換も起動。USBドライバ・実機駆動は別起動。
外部の実測・TFへ合わせる場合:

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  joint_control_backend:=external state_topic:=/follower/joint_states
```

実測変換の重複起動は避けること。設定は[dynamixel_joint_state_input.yaml](config/dynamixel_joint_state_input.yaml)、
追従接続は[共通手順](docs/dynamixel_sim_control.md#srfの共通追従構成)を参照。

左右の各ノードが安全でも、その組合せの安全は別判定。腰・首・グリッパーは学習時の固定条件。
組合せ検査:

```bash
ros2 service call /topo_dual_arm_max_long/check_arm_pair \
  gng_control_msgs/srv/CheckArmPair '{left_node_id: 0, right_node_id: 0}'
```

IDは配信グラフから選択。start_stateなしは終点のみ、指定時は左右14関節の線形補間に沿う離散検査。
環境障害物の判定・連続時間保証・実機指令は対象外。成功判定はis_validとis_collision_freeの両方。
学習時の衝突検査・キャッシュ条件は[TECHNICAL_SPEC](docs/TECHNICAL_SPEC.md)と[学習実装](launch/offline_urdf_trainer_dual.launch.py)を参照。

### ToPoFuzzy-Viewerへのブリッジ

Longは[上記の共通起動](#双腕学習viewerの既定モデル)を使用。別モデルの指定例:

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

従来の14関節モデルは`enable_independent_arms:=false`と`gng_model_path`・`vlut_path`を指定。
機体・モデル・設定の切替はブリッジ再起動が必要。
グリッパー体積の確認用トピックは`enable_gripper_volume_graph:=true`で有効化。
これらを把持ノードへ入力する場合も発行元が必要。設定は[ToPoDualArm.yaml](config/ToPoDualArm.yaml)。

L0簡略グラフは`visualization_gng.enabled: true`と現行形式の`vis_gng_L0.bin`で同時配信。
緑・黄・赤は集約元の安全・危険・衝突／使用不可の件数割合。成功確率としての解釈は不可。
通常版は状態更新に追従し、安全判定用`label`は保持。旧形式の割合表示には再生成が必要。
仕様は[L0状態割合](docs/releases/2026-09-16_l0_state_ratio_colors.md)を参照。

保存時点の簡略グラフだけを配信する場合:

```bash
ros2 launch gng_vlut_system visualization_gng_static.launch.py \
  model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng_static_L0.bin \
  topic_name:=/ToPoDualArm/Tmap_vis_static_L0 frame_id:=ToPoDualArm/base_link
```

単体版は静的表示。占有による安全状態更新・軌道変換は対象外。
個別配信には`topofuzzy_bridge.launch.py`・`topoarm_viewer_bridge.launch.py`を使用。

### 環境点群からVLUTへの入力

```bash
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max_long.yaml
```

設定は[機体YAML](config/topo_dual_arm_max_long.yaml)を参照。
取得時刻のセンサ→ロボット基準TFが必要。末尾一致によるframe置換・未接続frameのworld扱いは校正の代用にしないこと。

roi_voxelsは自己除去前、self_filter_roi_voxelsは除去後。VLUTへは除去後を入力。
自己マスクは回転・並進後の境界セル不足が残るため、厳密な形状保証とは区別。
ROI範囲は[仕様](docs/TECHNICAL_SPEC.md#17-roi範囲の静的マップbbox追従)、実センサ接続は[MID-360](../hardware_drivers/mid360/README.md)を参照。

### ROI登録とAiS-GNGの自己判定共有

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=topo_dual_arm_max_long.yaml enable_shared_roi_gng:=true
```

設定は[機体YAML](config/topo_dual_arm_max_long.yaml)を参照。
同じ入力のROI・自己判定・CPU GNG登録を共有。通常のais_gng.launch.pyと重複起動しないこと。
共有OFF・粗索引OFFが既定。追加費用があるため、単一ROIでの総時間短縮は保証なし。

共有APIの更新後は関連componentを同じ版へ再ビルド:

```bash
colcon build --packages-select voxel_idx pointcloud_sampling gng_cpu ais_gng gng_vlut_system fuzzy_voxel_grid \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Dallow_external_sampler=ON
```

旧共有型・新共有ライブラリの混在は禁止。

## 4. テストとデバッグ

占有・危険ボクセルの診断配信:

```bash
ros2 run gng_vlut_system voxel_status_test_publisher
```

学習結果の可視化（`gng_vlut_system/`で実行）:

```bash
python3 scripts/visualize_joint_angles.py
python3 scripts/visualize_manipulability.py --file build/manipulability_data.csv
```

自作テスト・ベンチコードはローカル保持・Git管理外、新規checkoutには未同梱。
未配置時の通常ビルドは継続。内部仕様・ノード構成は[TECHNICAL_SPEC](docs/TECHNICAL_SPEC.md)を参照。
