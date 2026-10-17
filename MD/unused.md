# 未使用の参考コマンド

## 把持ノードから接続構造を追って、指定距離以内のノードを抽出する (今の所未使用)

ros2 launch ais_gng topological_query.launch.py \
  input_topic:=/topological_map/merged \
  relation_mode:=graph_edges \
  semantic_label:=1 \
  max_euclidean_distance:=0.5 \
  max_hops:=-1

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

## 単体HTMLのrosbridge

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

トピック自動発見にはrosapiも必要なため、WebSocketノード単体ではなく上記launchを使用。
ToPoFuzzy ViewerのWS v2バックエンドとは別経路。
