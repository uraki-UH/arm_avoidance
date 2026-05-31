# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド

docker compose up --build
docker compose down
dokcer compose up -d

## frontendの起動
cd ~/uraki_ws
docker compose --profile manual up  frontend

## docker の起動
cd uraki_ws && docker compose up -d

cd uraki_ws && docker compose exec gng_cpu bash

##  backendの起動
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py

## ロボットおよび対応する学習済みGNGの起動
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  topic_name:=/topological_map_static

##　dynamixel handlerの起動
ros2 launch dynamixel_handler dynamixel_handler_launch.xml

USB の番号が変わる環境では、こちらのラッパーの方が安定します。
ros2 launch topoarm_bringup dynamixel_handler_auto.launch.py

##　dynamixelの/dynamixel/state/present　トピックをjoint_statesに変換
ros2 launch dynamixel_joint_state_bridge dynamixel_joint_state_bridge.launch.py namespace:=/ToPoDualArm

##　自己認識ボクセルの起動
ros2 launch gng_vlut_system self_recognition_viz.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  root_link:=left_link2 \
  leaf_link:=left_link8 \
  mask_topic:=/self_recognition/left_arm_voxel_mask

##　自己認識ボクセルをoccupied_voxels / danger_voxelsに橋渡し
ros2 launch gng_vlut_system self_recognition_voxel_bridge.launch.py \
  input_topic:=/self_recognition/left_arm_voxel_mask \
  occupied_voxels_topic:=/occupied_voxels \
  danger_voxels_topic:=/danger_voxels \
  danger_inflation:=0.05


必要に応じて
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 



## 自己認識ボクセル内外の点群に分けてパブリッシュ
ros2 run gng_vlut_system self_recognition_filter_node
  # self-recognition voxel内の点群: /self_recognition_points
  # self-recognition voxel外の点群: /self_filtered_points


## tf のダミー
python3 test_tf_publisher.py　(--ros-args -p frame_id:=topoarm/base_link)

# joint_statesのダミー
python3 dummy_joint_pub.py 

## realsenseのrosbag
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ --topics /camera/camera/depth/color/points --loop

## 点群の座標変換版トピック
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py 

## realsenseにおけるGNG
ros2 launch ais_gng camera_depth_points.launch.py target_frame_id:=world


## GNGの学習の実行
  ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true 
  \gng_profile_names:=left_arm 

  (initial_collision_only:=true):初期姿勢での衝突リンクの組み合わせを検証
