# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド
これで一発でfrontendが立ち上がる(-dをつけるとバックグラウウンドで起動し続けて面倒)
docker compose --profile manual up  frontend

以下で終了できる-dを押していても終了はできる
docker compose stop frontend

## ロスバグの再生例
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ --topics /camera/camera/depth/color/points  --loop

## 1. バックエンドのビルドと起動
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash 
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py
```

## 2. ブリッジノードの起動
`static_only` では、`gng.bin` を静的グラフとしてビューアへ中継します。
この構成では `enable_safety_monitor:=false` にして、`/topological_map` は出しません。
```bash
source install/setup.bash && ros2 launch gng_vlut_system gng_vlut_runtime.launch.py robot_name:=topoarm enable_joint_state_publisher:=true enable_safety_monitor:=false 

ros2 launch gng_vlut_system gng_viewer_bridge.launch.py topic_name:=/topological_map_static robot_base_frame:=base_link gng_frame_id:=base_link 
```

## 3. フロントエンドの起動
Viteデバッグサーバを立ち上げます。ブラウザで可視化画面が表示されます。
```bash
cd ToPoFuzzy-Viewer/frontend && npm install && npm run dev
```

## 4. リアルタイム位置合わせ（キャリブレーション）
センサーとロボットの位置がズレている場合、以下の手順で GUI から調整できます。

python3 test_joint_state_publisher.py 
オプションジョイント名をつけたい場合
--prefix topoarm_

python3 test_tf_publisher.py
(--ros-args -p frame_id:=topoarm/base_link)

ros2 launch gng_vlut_system gng_viewer_bridge.launch.py   topic_name:=/topological_map_static   robot_base_frame:=base_link   gng_frame_id:=base_link

realsenseのrosbag  （dynamicの方もtfに対応しているがこれは処理が重くなる原因なのでやめたほうがいいかも）

ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ --topics /camera/camera/depth/color/points --loop

点群の座標変換版トピック
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py 

realsenseにおけるGNG
ros2 launch ais_gng camera_depth_points.launch.py target_frame_id:=world

robotの自己認識ボクセル
ros2 launch gng_vlut_system self_recognition_viz.launch.py 
(marker_frame_id:=world display_mode:=link_local)

座標変換
ros2 run gng_vlut_system self_recognition_filter_node


学習の実行　(vlut_only:=trueも可能)
ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
  use_voxel_collision:=true \
  initial_collision_only:=true


ToPoDualArmの場合
python3 dummy_joint_pub.py --robot topo_dual_arm


ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  topic_name:=/topological_map_static


  ros2 launch gng_vlut_system self_recognition_viz.launch.py   \params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml 


  ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true
  

dynamixel
ros2 launch dynamixel_handler dynamixel_handler_launch.xml

ros2 launch dynamixel_joint_state_bridge dynamixel_joint_state_bridge.launch.py namespace:=/ToPoDualArm
