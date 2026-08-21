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




GNGノードを、把持候補の前段となるラベル付きボクセルへ変換する。
`HUMAN`、`CAR`を除外し、現在の点群支持があるセルを対象にする。`SAFE_TERRAIN`は候補としては除外するが、
床際物体の履歴を切らないため内部の構造証拠としては保持する。`auto`支持は各セルごとに
GNGの`inpcl_ids`と現在点群の近傍支持の大きい方を使うため、IDが一部のノードで欠けても安定した点群を失わない。セルの有効／無効は、
点群支持の在席率と切替率から求める時間安定度で決める。現在の点群支持は周囲27セルの中央値で正規化し、
点群・ノード密度や`grid_size`が変わっても固定個数閾値への依存を避ける。EMAはMap headerの時刻差から
更新するため、点群レートにも依存しない。単発の欠落は残し、繰り返すON/OFFは切替率が上がるため抑制する。
時間安定度は `在席率 × (1 - 切替率)` であり、rayを使わずに一時欠落と反復フリッカへの扱いを分ける。
さらにネイティブ深度画像が同時刻にあるときは、各ボクセル中心を1回だけ射影して3×3画素の中央値を比較する。
画角外と手前物体による遮蔽では履歴を保持し、対応画素の深度がセルより奥にある（その場所が空いた）場合だけ
負の証拠として減衰させる。全rayの走査はしない。深度または座標変換が得られないフレームは従来の時間減衰に戻る。
`depth_visibility_*` と `camera_to_map.*` は通常触らず、プロファイル YAML にだけ置いてある。TFを優先し、
録画に `base_link`→カメラTFがない場合だけ既存の点群変換キャリブレーションをフォールバックとして使う。
深度がない場合も、過去セルのGNG近傍が複数残り、局所的に静止または移動方向が不一致なら、その既知セルだけを保持する。
近傍が局所edge間隔に対して大きく、同方向に並進したときだけ、移動物体の旧位置として通常の時間減衰へ渡す。
これは前後フレームの既存GNG edgeとノード位置だけを1回走査する。新規セルの膨張、点群kNN・PCA、全ボクセル走査は行わない。
単ノード法線方向の動きは補助オプションであり、既定では無効である。
`SAFE_TERRAIN`は新規候補にはしないが、内部ではGNG構造・在席の証拠として残す。
そのため床際の物体で`UNKNOWN_OBJECT`と`SAFE_TERRAIN`が交互になっても、terrainを観測欠損としては扱わない。
ただし単発unknownは候補化せず、同一フレームのGNG edge連結成分が**4ノードかつ4つの点群支持済み出力セル**を
占有したときだけ物体証拠として履歴へ記録する。`unknown_component_event_count`、
`unknown_component_event_node_count`、`unknown_component_event_voxel_count`で確認できる。
直接観測セルの連結性は26近傍セルではなく、両端が安定したGNG edgeで判定する。細かい`grid_size`で
途中セルが抜けても、そのedge上だけを補間して連結を維持する。edge長の除外も固定距離ではなく、
各端点の周辺GNG edge長に対するロバストな外れ値で決める。GNG edgeを持たない直接観測セルだけを
`<output_topic>/isolated`へ分離する。
`output_topic`は非孤立の直接観測と補間セルの和集合で、`edge_inferred`と`triangle_inferred`は補間由来だけを出す。

ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  pointcloud_topic:=/downsampling/unknown \
  output_topic:=/topo_voxel_ids \
  grid_size:=0.02

通常起動で指定できる値は上の4つだけである。詳細設定は
`ais_gng_cpu/src/ais_gng/config/topological_grid.yaml`に集約した。
別プロファイルを試すときだけ、`params_file:=/absolute/path/profile.yaml`を追加する。

主な調整点は`temporal_time_constant_sec`（在席率・切替率の時間定数）、`temporal_activation_score`（発火）、
`temporal_retention_score`（消失）である。
`retention_score < activation_score`のヒステリシスを保つ。`unknown_shape_filter_enabled`は既定で無効で、
形状の逸脱だけを物体判定にしない。summaryにはこれらの設定値と活動度を出す。
深度が有効なときは `depth_visibility_free_count`、`depth_visibility_occluded_count`、
`depth_visibility_out_of_view_count` もsummaryに出る。移動物体の削除証拠として見るのは `free_count` だけである。
局所構造の判定数は `local_structure_static_node_count`、`local_structure_moving_node_count`、
`local_structure_ambiguous_node_count` に、これで保持された旧セル数は `retained_by_local_structure` に出る。
GNG法線方向の補助スコアは `normal_drift_mean_score` と `normal_drift_maximum_score` に出る。
`edge_max_length: 0.0` は局所GNG edge長から自動で外れedgeを除く設定である。出力の`grid_size`は
把持テンプレートの必要解像度で選び、物体連結性のために大きくする必要はない。

`point_activity_update_enabled:=true`では、点群占有頻度と点密度から重い更新の実行間隔を連続的に変える。
静止点群では更新を間引き、新しい占有や消失が多いと毎入力へ近づく。出力ボクセルとは別の物理セルで
統計を取るため、`grid_size`を小さくしても活動度の統計セル数が過剰に増えにくい。

# 補間由来だけを確認
ros2 topic echo /topo_voxel_ids/edge_inferred
ros2 topic echo /topo_voxel_ids/triangle_inferred

# 表示専用で、把持候補には含まれない孤立セルを確認
ros2 topic echo /topo_voxel_ids/isolated

候補labelや点群支持、補間条件を変える場合も、launch 引数を増やさず
`topological_grid.yaml`をコピーしたプロファイルを作り、`params_file`で指定する。


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
