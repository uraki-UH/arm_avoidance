# Multi-Layer Geometric GNG Project

ロボット姿勢GNG・VLUTによる計画／干渉判定。既定機体はLong。
[起動手順](../RUN_GUIDE.md)・[共通ROS回避](docs/dual_arm_simulation.md#センシング--ros回避--実行先)を参照。

## 詳細仕様

- [docs/TECHNICAL_SPEC.md](docs/TECHNICAL_SPEC.md)
- [docs/RELEASE_NOTE_TEMPLATE.md](docs/RELEASE_NOTE_TEMPLATE.md)
- [docs/releases/](docs/releases/)

## 依存ライブラリ (Dependencies)

- **Eigen3**: 解析計算、ベクトル・行列演算に使用。
- **urdfdom** / **urdfdom_headers**: URDFロボットモデルのパースに使用。
- **tinyxml2**: XMLのパースに使用（urdfdomで使用）。
- **console_bridge**: ロギングユーティリティ。
- **ROS 2 Humble**: ROS 2関連のノード、ビルドシステム、可視化に使用。

---

# GNG / VLUT オフライン処理 & ROS 2 ガイド

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

- 構造: GNGの隣接辺ごとの線形関節軌道 → 理想外接球の掃引セル → セルから辺への疎な逆引き。全関節格子の列挙なし。
- 判定: 非交差は`clear`、サンプル球に完全包含されるセルとの接触は`blocked`、境界・未構築は`unknown`。通過判定には自己干渉・固定環境の離隔下界も必要。
- 環境登録: 同一点群スナップショット・格子条件につき1回。占有の追加・消去による接触カウンタ更新。内部離隔の細分はFKだけで、掃引セルの再登録なし。
- 予算: 既定512辺・60 s／レシピ、128サンプル／辺・細分段階、500,000逆引き関係。CLIの`--max-num-edges`・`--max-build-sec`で調整。未処理辺は通過扱いにならず、従来の区間検査へ委任。
- 条件照合: 全関節端点・形状・計画リンク・固定環境・入力セル半径・要求余裕。不一致または境界判定は従来検査。固定関節や非対象腕の姿勢変更も対象。

ROS側の選択口: `dual_arm_gng_lidar_demo.launch.py`／`dual_arm_gazebo_demo.launch.py`の`vlut_backend:=edge_vlut edge_vlut_file:=<索引の絶対パス>`、または回避設定YAML内の同名項目。既存ノード安全判定は継続し、確定衝突辺だけ探索から早期除外。`vlut_backend:=node_vlut`で従来方式へ復帰。

検証範囲: Longの小規模学習launch・旧バイナリ一致・19関節形状の7軸／14軸構築・ROSノード選択。ブラウザ独立腕計画・Harmonic入口・Gazebo物理を含む通し回避・実機は未検証または未接続。動的障害物の予測、実機追従誤差、未知領域の安全保証は対象外。初回読込み・環境更新の追加費用があるため、全条件での高速化保証なし。

### 3D占有に基づく領域ノード

任意の試験バックエンド。構築・状態判定・被覆保証の範囲・比較条件は[姿勢領域VLUT仕様](docs/TECHNICAL_SPEC.md#18-姿勢領域vlut)。
既存GNGの学習則・保存形式・既定バックエンドは維持。全可行姿勢の安全証明・実機保証は未達。

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

FuzzBotの初期位置・前方を原点とする状態 `(x, y, 相対yaw, v, omega)` を学習し、位置・前方向・速度をViewerへ配信します。設定は[config/fuzzbot.yaml](config/fuzzbot.yaml)の `mobile_gng` と `motion_graph`。既定は6万状態・1,000ノード・20万反復、1軌跡は最大16区間×0.5秒です。平坦床・差動二輪・横滑りなしの運動モデルで、車輪速度と加減速制限を適用します。

```bash
# 初回学習・保存済みモデルの読込・FuzzBot本体の同時表示
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=fuzzbot.yaml

# 設定変更後の再学習専用
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=fuzzbot.yaml \
  enable_rebuild:=true enable_build_only:=true

# 相対yaw −15～15°の状態表示
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=fuzzbot.yaml \
  min_display_yaw_deg:=-15.0 max_display_yaw_deg:=15.0
```

ViewerまたはSimulatorの「ROS Scene Layers」で `/fuzzbot/state_gng/map` を選択し、対象レイヤーの `Normals` を有効にします。このトピックのnormalは `(cos yaw, sin yaw, 0)` の車体前方で、環境表面の法線とは異なる属性です。速度は同じ向きにvを掛けたモデル値で、GNG状態領域のcluster velocityへ配信します。`state_gng/data` は5次元状態・単位・学習属性、`state_gng/node_features` は代表姿勢です。

`gng_viewer_bridge.launch.py`は機体YAMLの`mobile_gng`設定に応じて状態GNGを同時起動します。`enable_mobile_gng:=false`で本体表示のみ。双腕設定では既定OFF。

モデルは `gng_results/fuzzbot/state_gng.json`。`model_path` で変更可能です。yawはcos・sinへ埋め込み、学習の距離は位置・向き・速度・角速度の各scaleで換算します。学習中心に近い重複なしの実サンプルを代表状態として保存します。GNG学習は共通C++エンジンと6次元の厳密近傍索引を使用します。

`origin_x`、`origin_y`、`origin_yaw_deg` は表示原点の配置のみで、学習済み状態を変更しません。yaw選択は相対角で、`min_display_yaw_deg` が `max_display_yaw_deg` より大きい場合は±180°をまたぐ区間です。`min_display_speed`／`max_display_speed`、`min_display_angular_speed`／`max_display_angular_speed` で状態を選択可能。これらの表示範囲・原点は `/fuzzbot/mobile_gng` のROSパラメータでも変更可能です。選択後の辺は表示ノード配列へ参照を付け替えます。

ROSなしの確認用HTMLは次のコマンドで生成できます。

```bash
ros2 run gng_vlut_system mobile_gng_preview.py \
  /ros2_ws/src/gng_vlut_system/gng_results/fuzzbot/state_gng.json \
  /ros2_ws/src/log/artifacts/fuzzbot_state_gng/preview.html
```

学習済み辺は状態空間のGNG近傍で、走行可能な制御入力を保証する辺ではありません。車体・掃引占有のVLUT、環境衝突判定、odomへの自動追従、経路探索と実機走行は未接続です。既存の `mobile_state_graph.launch.py` が生成する運動制約付き遷移グラフは別データとして保持します。

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

学習したGNGマップやアームの姿勢をToPoFuzzy-Viewerに送信します。

既定のロボット名・設定は `topo_dual_arm_max_long` です。次の指定はファイル名だけでも利用でき、
`params_file` を省略した場合も同じ設定です。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=topo_dual_arm_max_long.yaml
```

左右別モデルは`dir`・`id`で出力ディレクトリを指定して表示します。
`gng_results/topo_dual_arm_max_long_independent/independent_arms.json`から左右のファイルを読み込みます。
従来の14関節モデルを表示する場合は`enable_independent_arms:=false`と、
`gng_model_path`・`vlut_path`を指定します。以前の検査済みモデルは
`log/artifacts/gng_self_collision_fix_20261001/long/model/`に保持しています。
名前・モデル・設定の切替には、起動中のブリッジの終了と新しい設定での再起動が必要です。

別モデルのToPoDualArmは次の指定です。左右グリッパーの体積確認用トピック
（`L_grip_V_Tmap`など、左右の`V`・`minV`・`baseV`・`sweptV`）は
`ToPoDualArm.yaml`で既定オフです。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

体積を可視化して確認する場合は、同じコマンドに
`enable_gripper_volume_graph:=true`を追加します。
`enable_gripper_volume_graph:=false`で発行ノードの起動を無効化できます。
引数を省略した場合は、YAMLの`gripper_volume_graph.enabled`に従います。
切替はlaunchの再起動時に反映されます。

簡略版L0は`/ToPoDualArm/Tmap_vis_L0`です。`visualization_gng.enabled: true`と
現行形式の`vis_gng_L0.bin`があれば、上記ブリッジの起動時に元の`Tmap_static`と同時配信されます。
モデルを再生成した場合はブリッジの再起動が必要です。

L0のノード色は、集約元の安全（緑）・危険（黄）・衝突／使用不可（赤）の件数割合で表示します。
ノード詳細には件数と割合を表示し、既存の安全判定用`label`は変更しません。
通常ブリッジ版は元姿勢の状態更新に追従し、単体版は保存時点の割合です。
割合は学習済み姿勢の構成比であり、成功確率ではありません。
旧形式の単体モデルは件数未収録のため従来色へ戻ります。割合表示にはモデル再生成が必要です。
今回のメッセージ拡張を反映する際は、関連ROSノードの再ビルド・再起動とViewerの再読み込みが必要です。
[変更点と検証](docs/releases/2026-09-16_l0_state_ratio_colors.md)を参照してください。

メッセージ定義を更新済みの環境で、静的な簡略版だけを追加する場合は次を使用します。

```bash
ros2 launch gng_vlut_system visualization_gng_static.launch.py \
  model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng_static_L0.bin \
  topic_name:=/ToPoDualArm/Tmap_vis_static_L0 \
  frame_id:=ToPoDualArm/base_link
```

Viewerでは`/ToPoDualArm/Tmap_vis_static_L0`を選択します。単体版は保存時点の静的グラフで、
占有による安全状態の更新や軌道変換はありません。

`grasp_voxel_matcher_node`でこれらのトピックを入力として使う構成では、
発行をオンにするか、別のlaunchから発行ノードを起動してください。

- **GNGマップを送信**:
  ```bash
  ros2 launch gng_vlut_system topofuzzy_bridge.launch.py
  ```
- **アーム姿勢を送信**:
  ```bash
  ros2 launch gng_vlut_system topoarm_viewer_bridge.launch.py
  ```

### 環境点群からVLUTへの入力

```bash
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max_long.yaml
```

設定は[機体YAML](config/topo_dual_arm_max_long.yaml)を参照。
取得時刻のセンサ→ロボット基準TFが必要。末尾一致によるframe置換・未接続frameのworld扱いは校正の代用にしないこと。

roi_voxelsは自己除去前、self_filter_roi_voxelsは除去後。VLUTへは除去後を入力。
自己マスクは回転・並進後の境界セル不足が残るため、厳密な形状保証とは区別。
ROI範囲は[仕様](docs/TECHNICAL_SPEC.md#17-roi範囲の静的マップbbox追従)、実センサ接続は[MID-360](../hardware/mid360/README.md)を参照。

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

自作テスト・ベンチコードはローカル保持・Git管理外、新規checkoutには未同梱。
未配置時の通常ビルドは継続。画面内QA・診断ノードは通常機能として維持。

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
- `/self_recognition/voxel_mask`: 自己認識マスクの送信トピック。
- `/occupied_voxels`: self_recognition のボクセルIDを安全監視へ渡すトピック。
- `/danger_voxels`: occupied を外側に膨張したシェルを渡すトピック。
- ...など

最新版

ros2 launch gng_vlut_system gng_vlut_monitor.launch.py \robot_name:=topoarm \dir:=gng_results \id:=topoarm_full_v2 \mode:=STATIC \tag:=topoarm

など
