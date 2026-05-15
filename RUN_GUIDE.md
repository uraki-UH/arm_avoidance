# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド

## tf のダミー
python3 test_tf_publisher.py　(--ros-args -p frame_id:=topoarm/base_link)

## 
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py   topic_name:=/topological_map_static   robot_base_frame:=base_link   gng_frame_id:=base_link

## realsenseのrosbag
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ --topics /camera/camera/depth/color/points --loop

## 点群の座標変換版トピック
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py 

## realsenseにおけるGNG
ros2 launch ais_gng camera_depth_points.launch.py target_frame_id:=world


ros2 run gng_vlut_system self_recognition_filter_node
  # self-recognition voxel内の点群: /self_recognition_points
  # self-recognition voxel外の点群: /self_filtered_points


# ToPoDualArmの場合
python3 dummy_joint_pub.py --robot topo_dual_arm


## ロボットおよび対応するGNGの召喚
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  topic_name:=/topological_map_static



## GNGの学習の実行
  ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true

  (initial_collision_only:=true)


## frontendの起動
docker compose --profile manual up  frontend

##  backendの起動
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py

##　dynamixel handlerの起動
ros2 launch dynamixel_handler dynamixel_handler_launch.xml

##　dynamixelの/dynamixel/state/present　トピックをjoint_statesに変換
ros2 launch dynamixel_joint_state_bridge dynamixel_joint_state_bridge.launch.py namespace:=/ToPoDualArm



##　自己認識ボクセルの起動
ros2 launch gng_vlut_system self_recognition_viz.launch.py   
 \params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml 