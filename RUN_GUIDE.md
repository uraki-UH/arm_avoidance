# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド

docker compose up --build
docker compose down
dokcer compose up -d

## frontendの起動

cd ~/uraki_ws

docker compose exec gng_cpu bash -lc 'cd /ros2_ws/src/ToPoFuzzy-Viewer/frontend && npm run build'

docker compose --profile manual up --build frontend
docker compose --profile manual up  frontend

## 左腕をtopological_map_avoidanceで動かしつつ、右腕を適当に揺らす
ros2 launch gng_vlut_system topological_map_avoidance.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  trial_mode:=true \
  trial_safe_only:=true \
  right_arm_oscillation_enabled:=true \
  trial_return_home:=true

## docker の起動
cd uraki_ws && docker compose up -d

cd uraki_ws && docker compose exec gng_cpu bash

##  backendの起動
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py

## ロボットおよび対応する学習済みGNGの起動
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
   \topic_name:=/topological_map_static

##　dynamixel handlerの起動
ros2 launch dynamixel_handler dynamixel_handler_launch.xml

USB の番号が変わる環境では、こちらのラッパーの方が安定します。
ros2 launch topoarm_bringup dynamixel_handler_auto.launch.py

##　dynamixelの/dynamixel/state/present　トピックをjoint_statesに変換
ros2 launch dynamixel_joint_state_bridge dynamixel_joint_state_bridge.launch.py namespace:=/ToPoDualArm

##　自己認識ボクセルの起動
ros2 launch gng_vlut_system self_recognition_viz.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  root_link:=right_link2 \
  leaf_link:=right_link8 \
  mask_topic:=/ToPoDualArm/right_arm_voxel

##　自己認識ボクセルをoccupied_voxels / danger_voxelsに橋渡し
ros2 launch gng_vlut_system self_recognition_voxel_bridge.launch.py \
  robot_name:=ToPoDualArm \
  input_topic:=/ToPoDualArm/right_arm_voxel \
  danger_inflation:=0.05


必要に応じて
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 

echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

alias sh='source /opt/ros/humble/setup.bash'
alias sw='source /ros2_ws/install/setup.bash'


## 自己認識ボクセル内外の点群に分けてパブリッシュ
ros2 run gng_vlut_system self_recognition_filter_node
  # self-recognition voxel内の点群: /self_recognition_points
  # self-recognition voxel外の点群: /self_filtered_points


## tf のダミー
python3 test_tf_publisher.py　(--ros-args -p frame_id:=topoarm/base_link)

# joint_statesのダミー
python3 dummy_joint_pub.py 

(topoarmの場合)
python3 dummy_joint_pub.py --robot topoarm

## realsenseのrosbag
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ --topics /camera/camera/depth/color/points --loop


## 点群の座標変換版トピック
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py 

## realsenseにおけるGNG
ros2 launch ais_gng camera_depth_points.launch.py target_frame_id:=world


## `/topo_points` を入力にして GNG を回す
ros2 launch ais_gng ais_gng.launch.py \
  backend:=cpu \
  lidar:=topo_points.yaml

`topo_points.yaml` のトップレベルキーは `ais_gng_node:` にしてください。ノード名と一致しないと YAML が読まれません。


## GNGの学習の実行
  ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true \gng_profile_names:=left_arm 

  (initial_collision_only:=true):初期姿勢での衝突リンクの組み合わせを検証


ros2 launch gng_vlut_system self_recognition_voxel_bridge.launch.py \
  robot_name:=ToPoDualArm \
  input_topic:=/ToPoDualArm/left_arm_voxel \
  danger_inflation:=0.02 \
  output_voxel_size:=0.02

ros2 launch gng_vlut_system voxel_spherized_robot_viewer.launch.py  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml


ros2 launch gng_vlut_system topological_map_avoidance.launch.py   params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml   trial_mode:=true   trial_safe_only:=true


ros2 launch gng_vlut_system topological_map_avoidance.launch.py   params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml   trial_mode:=true   trial_safe_only:=true   trial_return_home:=true


ros2 launch gng_vlut_system target_joint_state_executor.launch.py   robot_name:=ToPoDualArm   target_topic:=target_joint_states   state_topic:=joint_states   command_topic:=joint_commands   max_joint_velocity:=0.6   publish_hz:=50.0

tf位置調整
 python3 test_tf_once_publisher.py   --world-frame world   --frame-id topoarm/base_link   --x 0.0   --y 0.5   --z -0.3 --yaw 1.5  --hold-seconds 1.0   --publish-hz 20


HTML起動
python3 -m http.server 8000

http://localhost:8000/ToPo-FUZZY_Manipulation_v1.html


ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=topo_points.yaml

