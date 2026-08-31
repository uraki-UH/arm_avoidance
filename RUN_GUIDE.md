# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド
cd ~/uraki_ws
docker compose down && docker compose up --build  && docker compose up -d
docker compose exec gng_cpu bash
## frontendの起動
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


## 把持候補姿勢、軌道、動く
ros2 launch gng_vlut_system grasp_goal_planning.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  enable_motion:=false


## HTML起動
python3 -m http.server 8000
http://localhost:8000/ToPo-FUZZY_Manipulation_v1.html


### 点群をToPoDualArmのVLUTへ反映
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml


## AISGNG実行
ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=topo_points.yaml

ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=graspnet.yaml

## HTML全点群からCPU GNGテンプレートを保存

点群も保存
source /ros2_ws/install/setup.bash
ros2 run ais_gng save_object_gng_dataset mug_complete  --replace --with-points

--replaceをつけると同名で保存していたやつ削除

保存先は`/datasets/設定名_<UTC日時>_<連番>_gng_template.json.gz`。

同名テンプレートを置換し、過去の同名保存と対応する点群・深度・色情報を削除する場合。


置換保存先は`/datasets/mug_complete_gng_template.json.gz`。

保存済みテンプレートは、保存名の接頭名だけで静的トピックへ配信。

source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system object_template_map_publisher.launch.py \
  dataset_file:=mug_complete

## 環境GNGとの照合後に物体テンプレートを配信

照合が連続フレームで確定した場合だけ、`/<template_id>/topological_map_static`へ
事前登録GNGを配信する。

```bash
source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system object_template_matching.launch.py \
  dataset_file:=mug_complete
```

環境側GNG topicは既定で`/topological_map`。変更する場合は
`environment_topological_map_topic:=/topological_map/merged`を追加する。
姿勢許容、特徴量のファジー評価、確定条件は
`/ros2_ws/src/gng_vlut_system/config/object_template_matching.yaml`で設定する。
`max_contradiction_point_ratio`は、仮説の隣接構造で説明できない点群支持量の許容率とする。
この値を超える候補は破棄し、次点yaw候補を評価する。全候補が破棄された場合は
`topological_map_static`を配信しない。

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

# URDF準拠のダミー関節状態
ros2 launch gng_vlut_system dummy_joint_pub.launch.py \
  urdf_path:=/ros2_ws/src/<robot_package>/<robot>.urdf

## realsenseのrosbag + 点群座標変換
# ターミナル1: raw点群を /camera/camera/depth/color/points_raw へ　リマップして再生
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ \
  --topics /camera/camera/depth/color/points \
  --remap /camera/camera/depth/color/points:=/camera/camera/depth/color/points_raw \
  --loop

# ターミナル2: 変換後の点群を元のトピック名でpublish
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py \
  input_topic:=/camera/camera/depth/color/points_raw \
  output_topic:=/camera/camera/depth/color/points


## GNGの学習の実行
  ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true \gng_profile_names:=left_arm 
  (initial_collision_only:=true):初期姿勢での衝突リンクの組み合わせを検証

## 衝突urdfの球化
ros2 launch gng_vlut_system voxel_spherized_robot_viewer.launch.py  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml


ros2 launch gng_vlut_system topological_map_avoidance.launch.py   params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml   trial_mode:=true   trial_safe_only:=true ( trial_return_home:=true
)

ros2 launch gng_vlut_system target_joint_state_executor.launch.py   robot_name:=ToPoDualArm   target_topic:=target_joint_states   state_topic:=joint_states   command_topic:=joint_commands   max_joint_velocity:=0.6   publish_hz:=20.0


python3 test_tf_publisher.py --static --world-frame world --frame-id ToPoDualArm/base_link --x 0.35 --y 0.15 --z -0.3 --yaw 3.2 --hold-seconds 1.0


python3 -m pip install --user torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cpu


## GNGノードを把持候補用ボクセルへ変換
`/topological_map`と`/downsampling/grasp_support`を照合し、点群支持のある物体候補をボクセル化する。
`SAFE_TERRAIN`、`HUMAN`、`CAR`は候補から除外。

```bash
ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  pointcloud_topic:=/downsampling/grasp_support \
  output_topic:=/topo_voxel_ids \
  grid_size:=0.02
```

## 把持ボクセル照合（左グリッパ、POC）
ボクセルとグリッパ体積、平面クラスタなどを使って  tcp候補を得る

ros2 launch grasping_system grasp_voxel_matcher.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

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

現在の`dual_arm_robot.urdf`には`gazebo_ros2_control`がないため、Gazebo内の関節と
グリッパを指令して実際に把持するには、別途Gazebo用controller接続が必要。

## GazeboでTF追従させたい場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  follow_tf_frame:=ToPoDualArm/base_footprint






必要に応じて
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

alias sh='source /opt/ros/humble/setup.bash'
alias sw='source /ros2_ws/install/setup.bash'


### dockerではなく素の ROS2 環境で入れる場合
sudo apt install ros-humble-rosbridge-server

### 起動手順
# 1) rosbridge を起動
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090

このlaunchはWebSocketに加えて`rosapi`を起動する。単体HTMLは`rosapi`から
`*_grip_V_topological_map`、`*_grip_minV_topological_map`、`*_grip_baseV_topological_map`、
`*_grip_sweptV_topological_map`
topicを自動発見するため、`ros2 run
rosbridge_server rosbridge_websocket`だけではなく上記launchを使用する。

## 左腕をtopological_map_avoidanceで動かす
ros2 launch gng_vlut_system topological_map_avoidance.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  trial_mode:=true \
  trial_safe_only:=true


すべてのプロセスをキル　frontendを除く
./scripts/stop_ros2_stack.sh

docker compose --profile manual up -d --build frontend


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

## GNGエッジから差分方式で平面クラスタを作る
ros2 launch ais_gng plane_cluster_incremental.launch.py \
  input_topic:=/topological_map

今はais_gng_実行で生成できるようにしている

## 点群から占有ボクセルに変換
ros2 launch gng_vlut_system point_to_voxel.launch.py \
  input_topic:=/semantic_points \
  output_topic:=/topo_voxel_ids



### depth画素handle付きpersistent world indexの比較

固定カメラのraw depthからpersistent world indexを構築し、全再構築方式と比較する。
実機では`camera_world_*`へ外部パラメータを設定する。


# ターミナル2: raw depthとcamera_infoを同時に配信
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41 \
  --topics \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info


## ダミー把持姿勢をPoseArrayで流す
ros2 launch gng_vlut_system grasp_pose_dummy_publisher.launch.py \
  frame_id:=world \
  candidate_count:=1


ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed --loop

ros2 launch graspnet_ros2 play_scene.launch.py scene_id:=3 camera:=realsense start:=10 end:=20 h
z:=20.0
