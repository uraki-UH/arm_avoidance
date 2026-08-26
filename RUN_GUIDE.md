# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド
cd uraki_ws
docker compose down && docker compose up --build  && docker compose up -d
docker compose exec gng_cpu bash
## frontendの起動
cd ~/uraki_ws
docker compose exec gng_cpu bash -lc 'cd /ros2_ws/src/ToPoFuzzy-Viewer/frontend && npm run build'

docker compose --profile manual up  frontend

chrome://restart

##  backendの起動
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py

## ロボットおよび対応する学習済みGNGの起動
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

## ロボットを座標変換
python3 test_tf_publisher.py --world-frame world --frame-id ToPoDualArm/base_link --x 0.35 --y 0.15 --z -0.3 --yaw 3.2


## ダミー把持姿勢をPoseArrayで流す
ros2 launch gng_vlut_system grasp_pose_dummy_publisher.launch.py \
  frame_id:=world \
  candidate_count:=1

## 把持候補姿勢、軌道、動く
ros2 launch gng_vlut_system grasp_goal_planning.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  enable_motion:=false


## HTML起動
python3 -m http.server 8000
http://localhost:8000/ToPo-FUZZY_Manipulation_v1.html


## 点群から占有ボクセルに変換
ros2 launch gng_vlut_system point_to_voxel.launch.py \
  input_topic:=/semantic_points \
  output_topic:=/topo_voxel_ids

### `/dataset/points`をToPoDualArmのVLUTへ反映

`/dataset/points`のheader frameは`world`とし、点群のframeを強制的に
`ToPoDualArm/base_link`として扱わない。点群時刻のTF
`world`→`ToPoDualArm/base_link`でロボット座標系へ変換した後、2 cm voxel IDを
VLUT入力へ渡す構成。

```bash
# ROI voxel化、world index、occupied/danger更新の起動
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

このlaunchはViewer gatewayとロボットを起動しないため、`gng_viewer_bridge.launch.py`を別terminalで起動する。
ボクセル解像度はVLUT headerから取得するため、通常は手動指定しない。

`environment_voxelization.world_index`では、index構築とROI抽出経路を別々に選択できる。

| `enable_build` | `enable_roi_query` | 動作 |
| --- | --- | --- |
| `false` | `false` | indexを構築せず、各robot座標系へ直接ROI voxel化 |
| `true` | `false` | world indexはViewer確認用に構築し、ROI voxel化は直接方式 |
| `true` | `true` | world indexを構築し、bucket AABB抽出後にROI voxel化 |

`enable_build: false`と`enable_roi_query: true`は使用不可。既定値は両方`true`。

### ToPo Fuzzy Viewerでのworld index確認

Viewer gatewayを別途起動後、Viewerのtopic一覧から次の`voxel_msgs/Voxel`を有効にする。

- `/ToPoDualArm/roi_voxel_ids`: `ToPoDualArm/base_link`座標系のVLUT入力ROI voxel
- `/ToPoDualArm/world_index_buckets`: `world`座標系の非空world bucket voxel

`world`から`ToPoDualArm/base_link`へのTFと、両topicのViewerレイヤーが必要。
移動ロボットでは`enable_static_tf: false`、固定設置で外部TFがない場合だけ静的TFを設定する。

### 複数robotの共有world index

`config/world_index.yaml`の`consumers`へrobotごとの設定を列挙する。

```bash
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/world_index.yaml
```

robot本体とViewer gatewayはrobotごとに別terminalで起動する。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm_A
```

2台目以降も`params_file`と`robot_name`を変更して同様に起動する。

danger判定方式は`ToPoDualArm.yaml`の`environment_voxelization.danger_source`で選択する。

- `environment_inflation`: 現在の既定値。`danger_inflation`だけ環境voxelを膨張してdangerへ送る方式
- `vlut_distance`: 環境側膨張を0にし、VLUT relationの距離値を`vlut_danger_dist`で判定する方式

現行VLUTは距離値を持たないため、`vlut_distance`は距離付きVLUT生成後に使用する。

```yaml
environment_voxelization:
  danger_source: "vlut_distance"
  vlut_danger_dist: 0.025
```

起動後は、`/dataset/points`、`/ToPoDualArm/roi_voxel_ids`、
`/ToPoDualArm/occupied_voxels`が順に更新されることを確認する。

```bash
ros2 topic hz /dataset/points
ros2 topic echo --once /ToPoDualArm/roi_voxel_ids
ros2 topic echo --once /ToPoDualArm/occupied_voxels
```

別ロボットでROIを変更する場合は、TCPサンプリング範囲と余裕を指定する。

```bash
ros2 launch gng_vlut_system point_to_voxel.launch.py \
  target_frame_id:=<robot_base_frame> \
  min_reachability_x:=<min_x> max_reachability_x:=<max_x> \
  min_reachability_y:=<min_y> max_reachability_y:=<max_y> \
  min_reachability_z:=<min_z> max_reachability_z:=<max_z> \
  reachability_margin_x:=<margin_x> \
  reachability_margin_y:=<margin_y> \
  reachability_margin_z:=<margin_z>
```

marginには把持物の張り出し、推定誤差、安全余裕、必要な台車移動範囲を含める。
全点を対象にする場合は`enable_reachability_filter:=false`を指定する。

### depth画素handle付きpersistent world indexの比較

固定カメラのraw depthからpersistent world indexを構築し、全再構築方式と比較する。
実機では`camera_world_*`へ外部パラメータを設定する。

```bash
# ターミナル1: persistent indexの実行設定
ros2 run gng_vlut_system depth_world_index_benchmark_node --ros-args \
  --params-file /ros2_ws/src/gng_vlut_system/config/depth_world_index_benchmark.yaml

# ターミナル2: raw depthとcamera_infoを同時に配信
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41 \
  --topics \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info
```

全再構築比較は`enable_comparison_benchmark:=true`、ログ抑止は`enable_runtime_log:=false`。
条件と計測結果は
[`2026-08-25_reachability_filtered_environment_voxelization.md`](gng_vlut_system/docs/releases/2026-08-25_reachability_filtered_environment_voxelization.md)を参照。

#### ROI voxelとworld indexの視覚確認
- `/depth_world_index/debug/roi_voxels` 
- `/depth_world_index/debug/world_buckets_voxels` 

##　ボクセルからGNGのoccupied_voxels / danger_voxelsに橋渡し
ros2 launch gng_vlut_system voxel_to_vlut.launch.py \
  robot_name:=ToPoDualArm \
  input_topic:=/topo_voxel_ids \
  danger_inflation:=0.08

## AISGNG実行
ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=d435.yaml
ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=topo_points.yaml

ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=graspnet.yaml


## RVizでロボットを表示
ros2 launch gng_vlut_system visualize_robot_rviz.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm


==============================================================

## 把持ノードから接続構造を追って、指定距離以内のノードを抽出する (今の所未使用)
ros2 launch ais_gng topological_query.launch.py \
  input_topic:=/topological_map/merged \
  relation_mode:=graph_edges \
  semantic_label:=1 \
  max_euclidean_distance:=0.5 \
  max_hops:=-1

##　dynamixel handlerの起動（使えない可能性が高い）
ros2 launch dynamixel_handler dynamixel_handler_launch.xml
USB の番号が変わる環境では、こちらのラッパーの方が安定。
ros2 launch topoarm_bringup dynamixel_handler_auto.launch.py

##　dynamixelの/dynamixel/state/present　トピックをjoint_statesに変換
ros2 launch dynamixel_joint_state_bridge dynamixel_joint_state_bridge.launch.py namespace:=/ToPoDualArm

ros2 launch dynamixel_joint_state_bridge \
  dynamixel_joint_state_bridge.launch.py \
  namespace:=/ToPoDualArm

realsense
ros2 run dynamixel_joint_state_bridge dynamixel_joint_state_bridge_node \
  --ros-args \
  -r __ns:=/ToPoDualArm \
  --params-file /ros2_ws/src/dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge.yaml \
  -p output_topic:=viewer_joint_states

##　自己認識ボクセルの起動
ros2 launch gng_vlut_system self_recognition_viz.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  root_link:=right_link2 \
  leaf_link:=right_link8 \
  mask_topic:=/ToPoDualArm/right_arm_voxel

##　自己認識ボクセルをoccupied_voxels / danger_voxelsに橋渡し
ros2 launch gng_vlut_system voxel_to_vlut.launch.py \
  robot_name:=ToPoDualArm \
  input_topic:=/ToPoDualArm/right_arm_voxel \
  danger_inflation:=0.05
  (output_voxel_size:=0.02)

## 自己認識ボクセル内外の点群に分けてパブリッシュ
ros2 run gng_vlut_system self_recognition_filter_node
  # self-recognition voxel内の点群: /self_recognition_points
  # self-recognition voxel外の点群: /self_filtered_points

# joint_statesのダミー
(topoarmの場合)
python3 dummy_joint_pub.py --robot topoarm

## realsenseのrosbag + 点群座標変換
# ターミナル1: raw点群を /camera/camera/depth/color/points_raw へリマップして再生
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ \
  --topics /camera/camera/depth/color/points \
  --remap /camera/camera/depth/color/points:=/camera/camera/depth/color/points_raw \
  --loop

# ターミナル2: 変換後の点群を元のトピック名でpublish
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py \
  input_topic:=/camera/camera/depth/color/points_raw \
  output_topic:=/camera/camera/depth/color/points

### 深度画像ベースの動体ノード削除判定ベンチ

Mapと同時刻の深度画像を30フレーム照合し、解像度別の処理時間を比較する。

```bash
# ターミナル1: 生点群は変換用に退避し、深度画像と内部パラメータも同時に再生する
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ \
  --topics \
    /camera/camera/depth/color/points \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info \
  --remap /camera/camera/depth/color/points:=/visibility/raw_points \
  --loop

# ターミナル2: GNG 入力用の点群を base_link へ変換する
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py \
  input_topic:=/visibility/raw_points \
  output_topic:=/camera/camera/depth/color/points

# ターミナル3: GNG
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml

# ターミナル4: 深度画像の削除証拠判定を計測する
ros2 run fuzzy_voxel_grid depth_visibility_benchmark
```

このbagでは等倍参照が最速。低解像度化はノード内poolingではなくカメラ側で行う。

### 変換済み点群を新しいrosbagへ1周分だけ保存
# 先に上の変換ノードを起動し、次にrecordを開始してから、最後にbagを--loopなしで再生する。

# ターミナル3: 変換済み点群と可視性判定用の生depthをrecord
ros2 bag record \
  -o /rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed \
  /camera/camera/depth/color/points \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info

# ターミナル1: record開始後にraw bagを1回だけ再生
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ \
  --topics \
    /camera/camera/depth/color/points \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info \
  --remap /camera/camera/depth/color/points:=/camera/camera/depth/color/points_raw

# 再生終了後、作成後は変換御殿軍を次で再生。
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed --loop

## GNGの学習の実行
  ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true \gng_profile_names:=left_arm 
  (initial_collision_only:=true):初期姿勢での衝突リンクの組み合わせを検証

## 衝突urdfの球化
ros2 launch gng_vlut_system voxel_spherized_robot_viewer.launch.py  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml


ros2 launch gng_vlut_system topological_map_avoidance.launch.py   params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml   trial_mode:=true   trial_safe_only:=true ( trial_return_home:=true
)

ros2 launch gng_vlut_system target_joint_state_executor.launch.py   robot_name:=ToPoDualArm   target_topic:=target_joint_states   state_topic:=joint_states   command_topic:=joint_commands   max_joint_velocity:=0.6   publish_hz:=20.0


python3 test_tf_once_publisher.py   --world-frame world   --frame-id ToPoDualArm/base_link   --x 0.35   --y 0.15   --z -0.3 --yaw 3.2  --hold-seconds 1.0   --publish-hz 20


同じ座標系でそのまま通す場合は、`voxel_to_vlut` の `target_frame_id` は指定しません。
このとき、入力 voxel の frame をそのまま使って再エンコードします。


python3 -m pip install --user torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cpu


## GNGノードを把持候補用ボクセルへ変換

`/topological_map`と`/downsampling/grasp_support`を照合し、点群支持のある物体候補をボクセル化する。
`SAFE_TERRAIN`、`HUMAN`、`CAR`は候補から除外する。

```bash
ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  pointcloud_topic:=/downsampling/grasp_support \
  output_topic:=/topo_voxel_ids \
  grid_size:=0.01
```

主な補助出力は`<output_topic>/delta`、`<output_topic>/isolated`、`<output_topic>/edge_inferred`、
`<output_topic>/triangle_inferred`、`<output_topic>/summary`。詳細設定は
`ais_gng_cpu/src/ais_gng/config/topological_grid.yaml`、処理仕様は
[`TECHNICAL_SPEC.md`](gng_vlut_system/docs/TECHNICAL_SPEC.md)を参照。

## GNGエッジから増分平面クラスタを作る

GNG edgeとノード法線から平面クラスタを増分生成する。

```bash
ros2 launch ais_gng plane_cluster_incremental.launch.py \
  input_topic:=/topological_map
```

確認用Markerは`/topological_planar_clusters_incremental/markers/obb`と
`/topological_planar_clusters_incremental/markers/nodes`。

## 把持ボクセルテンプレート（左グリッパ、POC）

`/topo_voxel_ids`をグリッパ体積と照合し、TCP姿勢候補とMarkerを出力する。

```bash
ros2 launch grasping_system grasp_voxel_template.launch.py
```

主な出力は`/grasp_pose_cand_cells`、`/grasp_pose_cands`、`/grasp_pose_markers`、
`/grasp_pose_cands/summary`。この段階ではIKと実ロボット到達性を評価しない。

深度画像、CameraInfo、同時刻TFがある入力でだけ深度可視性を有効化する。

```bash
ros2 launch grasping_system grasp_voxel_template.launch.py \
  enable_depth_visibility:=true
```

照合条件、掃引禁止体積、深度可視性、summary項目の詳細は
[`TECHNICAL_SPEC.md`](gng_vlut_system/docs/TECHNICAL_SPEC.md)を参照。

## realsense 
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=true


## Gazeboに召喚
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true

## Gazeboでベースをワールドに固定したい場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  spawn_z:=0.0 \
  fixed_base_link:=base_footprint

## Gazeboのピックアンドプレース用worldを使う場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  spawn_z:=0.0 \
  fixed_base_link:=base_footprint \
  world:=/ros2_ws/src/gng_vlut_system/worlds/pick_and_place.world

このworldには作業台、動的な立方体・円柱・直方体、配置用トレイ2個が含まれる。
現在の`dual_arm_robot.urdf`には`gazebo_ros2_control`がないため、Gazebo内の関節と
グリッパを指令して実際に把持するには、別途Gazebo用controller接続が必要。

## GazeboでTF追従させたい場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  follow_tf_frame:=ToPoDualArm/base_footprint






必要に応じて
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 

echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

alias sh='source /opt/ros/humble/setup.bash'
alias sw='source /ros2_ws/install/setup.bash'



### このリポジトリでの前提
- `Dockerfile` に `ros-humble-rosbridge-server` が入っています
- `docker-compose.yaml` に `rosbridge` サービスがあります
- そのため、通常は `docker compose up -d` で足ります

### 素の ROS2 環境で入れる場合
```bash
sudo apt install ros-humble-rosbridge-server
```

### 起動手順
```bash
# 1) rosbridge を起動
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

このlaunchはWebSocketに加えて`rosapi`を起動する。単体HTMLは`rosapi`から
`*_grip_V_topological_map`、`*_grip_minV_topological_map`、`*_grip_baseV_topological_map`、
`*_grip_sweptV_topological_map`
topicを自動発見するため、`ros2 run
rosbridge_server rosbridge_websocket`だけではなく上記launchを使用する。

```bash
python3 -m http.server 8000
```

```text
# 3) ブラウザで開く
http://localhost:8000/ToPo-FUZZY_Manipulation_v1.html
```


## 左腕をtopological_map_avoidanceで動かす
ros2 launch gng_vlut_system topological_map_avoidance.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  trial_mode:=true \
  trial_safe_only:=true
