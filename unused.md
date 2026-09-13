
## 把持ノードから接続構造を追って、指定距離以内のノードを抽出する (今の所未使用)
ros2 launch ais_gng topological_query.launch.py \
  input_topic:=/topological_map/merged \
  relation_mode:=graph_edges \
  semantic_label:=1 \
  max_euclidean_distance:=0.5 \
  max_hops:=-1


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





基本未使用

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