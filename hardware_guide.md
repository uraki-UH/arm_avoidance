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



## realsense 
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=true

点群の各点とdepth画像の画素を対応付ける方式

ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  pointcloud.ordered_pc:=true \
  pointcloud.allow_no_texture_points:=true \
  align_depth.enable:=false


ros2 launch realsense2_camera rs_launch.py   align_depth.enable:=true   pointcloud.enable:=true   pointcloud.ordered_pc:=true


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
