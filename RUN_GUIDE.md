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
ros2 launch gng_vlut_system pointcloud_voxel_bridge.launch.py \
  input_topic:=/semantic_points \
  output_topic:=/topo_voxel_ids

##　ボクセルからGNGのoccupied_voxels / danger_voxelsに橋渡し
ros2 launch gng_vlut_system voxel_to_vlut_bridge.launch.py \
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

## グリッド所属を半セルずらしで出す（未使用）
ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  output_topic:=/topological_grid_voxels_shifted \
  grid_size:=0.01 \
  origin_shift_half:=true

##　dynamixel handlerの起動（使えない可能性が高い）
ros2 launch dynamixel_handler dynamixel_handler_launch.xml
USB の番号が変わる環境では、こちらのラッパーの方が安定します。
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
ros2 launch gng_vlut_system voxel_to_vlut_bridge.launch.py \
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
# bagのraw点群は内部トピックへ退避し、変換後の点群が元のトピック名を引き継ぐ。
# 入出力を同じトピックにすると変換ノードが自己購読するため、直接同名にはしない。

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

`/camera/camera/depth/image_rect_raw` を直接読むため、点群から深度バッファを再構築しない。
GNG は従来どおり変換済み点群を入力にする。ベンチマークは map と同時刻の深度画像を30フレーム照合し、
等倍・実行時1/2・実行時1/4 min-pooling を比較して自動終了する。

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

この bag でのC++計測では、等倍の深度画像を直接参照する方式が p50 約1.30 ms で最速だった。
実行時の1/2・1/4 min-pooling は各フレームで全画素を走査するため、p50 約7.38 ms / 6.74 ms となり遅い。
解像度を下げるなら、このノード内でpoolingするのではなく、カメラ側で低解像度深度画像を出す。

### 変換済み点群を新しいrosbagへ1周分だけ保存
# 先に上の変換ノードを起動し、次にrecordを開始してから、最後にbagを--loopなしで再生する。

# ターミナル3: 変換後の元トピック名をrecord
ros2 bag record \
  -o /rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed \
  /camera/camera/depth/color/points

# ターミナル1: record開始後にraw bagを1回だけ再生
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ \
  --topics /camera/camera/depth/color/points \
  --remap /camera/camera/depth/color/points:=/camera/camera/depth/color/points_raw

# 再生終了後、ターミナル3でCtrl-Cしてrecordを終了する。
# 作成後は変換ノードなしで次を再生できる。
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


同じ座標系でそのまま通す場合は、`voxel_to_vlut_bridge` の `target_frame_id` は指定しません。
このとき、入力 voxel の frame をそのまま使って再エンコードします。


python3 -m pip install --user torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cpu




GNGノードをラベル付きボクセルに変換する。
既定では、同一frame・同一timestampの `/topological_map` と `/downsampling/unknown` を照合し、
`SAFE_TERRAIN`、`HUMAN`、`CAR`以外のノードと現在点群が同じセルにある場合を候補にする。
各セルは直近100更新の非除外label出現回数と点群input更新回数を別々に保持する。
`DEFAULT`、`WALL`、`UNKNOWN_OBJECT`は物体候補占有として合算する。
隣接候補は各3回、26近傍に候補がない孤立セルは各5回を履歴内で満たしてからpublishする。
このlabel最低カウントは新規追加と削除後の再追加にだけ使用し、確定済みセルの削除条件には使用しない。
連続観測である必要はない。新規追加時だけ現在点群を必須とし、確定後に非除外labelがセルから消えても
既定10更新は旧セルを維持する。隣接セルへ移動したノードが追加条件を満たすまでの欠落をこの猶予で防ぐ。
孤立判定は現在点群ではなく非除外GNGセル同士の近傍で行う。確定後の非孤立セルは点群input履歴が
最低カウント未満になっても削除しない。孤立セルも`DEFAULT`、`WALL`、`UNKNOWN_OBJECT`のいずれかの
ノードが現在セルに残る間は、点群やlabel種別が変化しても赤い非把持セルとして保持する。
孤立セルの点群消失を削除条件に使うのは、非除外GNGノードもセルから消えた場合だけとする。
非孤立セルは、確定時に一意だったGNGノードの`(id, frame)`が現在Mapにも存在し、確定位置からの移動が
既定`0.02 m`以内なら保持する。同一Map内で重複する`(id, frame)`は追跡に使用しない。
確定済みボクセルに所属するGNGノード同士がedge接続されている場合、既定ではボクセル対を1本へ集約し、
最大`0.10 m`までの間を10 mmセルで補間する。26近傍がない直接観測セルは把持候補から除外して
`<output_topic>/isolated`へ分離する。Topo Fuzzy Viewerではこのtopicを選択すると既定で赤表示される。
`output_topic`は非孤立の直接観測と補間セルの和集合、
`<output_topic>/edge_inferred`は補間セルだけを保持する。補間セルを次の補間端点には使用しない。
さらに3ノード間のedgeがすべて存在する3-cycleを三角形候補とし、辺長、面積、細長さ、ノード法線、
任意の点群支持率を通過した面と交差するセルを追加する。`<output_topic>/triangle_inferred`は面由来だけを保持する。

ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  pointcloud_topic:=/downsampling/unknown \
  output_topic:=/topo_voxel_ids \
  summary_topic:=/topo_voxel_ids/summary \
  grid_size:=0.01 \
  point_support_mode:=auto \
  point_support_radius_m:=0.02 \
  unknown_shape_filter_enabled:=true \
  shape_neighborhood_hops:=2 \
  shape_minimum_neighbors:=3 \
  shape_residual_weight:=0.7 \
  shape_mad_multiplier:=3.0 \
  shape_seed_expansion_scale:=2.0 \
  neighbor_radius_m:=0.02 \
  inferred_require_input_points:=true \
  node_identity_history_migration_enabled:=true \
  node_identity_retention_enabled:=false \
  history_reset_on_time_regression:=false \
  history_reset_node_count_ratio:=0.5 \
  point_activity_update_enabled:=true \
  point_activity_cell_size:=0.02 \
  point_activity_ema_alpha:=0.2 \
  point_activity_top_fraction:=0.1 \
  point_activity_occupancy_weight:=0.7 \
  point_activity_warmup_updates:=5 \
  point_activity_minimum_update_interval:=1 \
  point_activity_maximum_update_interval:=10

`point_support_mode:=auto`は、AIS-GNGノードに`inpcl_ids`がある場合は入力点との直接対応を
点群支持に使い、対応がないMapでは`point_support_radius_m`以内の点群支持へフォールバックする。
`unknown_shape_filter_enabled:=true`は、`UNKNOWN_OBJECT`ノードの1〜2 hop近傍について、
局所平面残差をそのノードのGNG edge長中央値で正規化し、法線変化と合成する。
全候補の中央値/MADに対して相対的に逸脱するノードだけをseedとし、局所edge長で正規化した
graph距離内へ領域を拡張する。固定のmm半径や突出量は使用しない。後方の連続平面はseedを
持たないため除外され、物体の角や段差に接続した平坦面はseed周辺として保持される。
summaryの`shape_candidate_node_count`、`shape_seed_node_count`、
`shape_retained_node_count`、`shape_rejected_node_count`で抑制量を確認できる。
`neighbor_radius_m`はグリッドセル数ではなく実距離で近傍を判定する。
`node_identity_history_migration_enabled:=true`では、一意な`(node.id, node.frame)`が
`node_identity_max_displacement`以内で移動したとき、時間履歴を移動先セルへ引き継ぐ。
これらの設定により、`grid_size`を小さくした際のセル境界による発火切れを抑制する。
rosbagを`--loop`再生してもGNGノードIDが継続する構成では、時刻巻き戻りだけで履歴を消すと
毎周回出力が途切れるため、`history_reset_on_time_regression:=false`を使用する。
GNGノード数が急減して実際にリセットされた場合は`history_reset_node_count_ratio`で履歴を消去する。
履歴を移動先へ引き継ぐ場合、旧セルを二重保持しないよう
`node_identity_retention_enabled:=false`を使用する。

`point_activity_update_enabled:=true`では、点群占有頻度と点密度のEMAから活動度を計算し、
重いノード・edge・triangle処理の実行間隔を1〜10入力の間で連続的に変更する。
起動直後の5入力は必ず処理し、時間履歴を遅延なく確立する。
新しい占有領域や消失領域があれば毎入力に近づき、静止点群では最大10入力ごとになる。
活動度は出力ボクセルとは独立した既定20 mmの物理セルで計算するため、`grid_size`を5 mmへ
小さくしても点のセル境界揺れと統計セル数が過剰に増えにくい。全更新を省略した入力では
最後にpublishしたVoxelを維持し、最大間隔に達すると必ず再計算する。
summaryの`point_activity_score`、`point_activity_desired_update_interval`、
`point_activity_processed_update_count`、`point_activity_skipped_update_count`で動作を確認できる。

# 補間由来だけを確認
ros2 topic echo /topo_voxel_ids/edge_inferred
ros2 topic echo /topo_voxel_ids/triangle_inferred

# 表示専用で、把持候補には含まれない孤立セルを確認
ros2 topic echo /topo_voxel_ids/isolated

候補labelや点群支持条件を変更する場合は、次のように指定する。

ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  pointcloud_topic:=/downsampling/unknown \
  output_topic:=/topo_voxel_ids \
  summary_topic:=/topo_voxel_ids/summary \
  grid_size:=0.01 \
  excluded_labels:="SAFE_TERRAIN,HUMAN,CAR" \
  require_input_points:=true \
  point_support_mode:=auto \
  point_support_radius_m:=0.02 \
  unknown_shape_filter_enabled:=true \
  shape_neighborhood_hops:=2 \
  shape_minimum_neighbors:=3 \
  shape_residual_weight:=0.7 \
  shape_mad_multiplier:=3.0 \
  shape_seed_expansion_scale:=2.0 \
  neighbor_radius_m:=0.02 \
  inferred_require_input_points:=true \
  history_window_size:=100 \
  minimum_label_history_count:=3 \
  minimum_point_input_history_count:=3 \
  isolated_minimum_label_history_count:=5 \
  isolated_minimum_point_input_history_count:=5 \
  maximum_missing_label_updates:=2 \
  node_identity_retention_enabled:=false \
  node_identity_max_displacement:=0.02 \
  node_identity_history_migration_enabled:=true \
  history_reset_on_time_regression:=false \
  history_reset_node_count_ratio:=0.5 \
  point_activity_update_enabled:=true \
  point_activity_cell_size:=0.02 \
  point_activity_ema_alpha:=0.2 \
  point_activity_top_fraction:=0.1 \
  point_activity_occupancy_weight:=0.7 \
  point_activity_warmup_updates:=5 \
  point_activity_minimum_update_interval:=1 \
  point_activity_maximum_update_interval:=10 \
  edge_inference_enabled:=true \
  edge_max_length:=0.10 \
  triangle_inference_enabled:=true \
  triangle_max_edge_length:=0.05 \
  triangle_min_area:=0.000001 \
  triangle_min_aspect_ratio:=0.05 \
  triangle_max_normal_angle_deg:=45.0 \
  triangle_min_point_support_ratio:=0.0

保持期限を超えたセルは通常削除されますが、削除によって占有セルの低いZ側と高いZ側が
26近傍グラフ上で分断されるセルは保持されます。水平方向だけの分断は削除を妨げません。


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
`*_grip_V_topological_map`、`*_grip_minV_topological_map`、`*_grip_baseV_topological_map`
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
