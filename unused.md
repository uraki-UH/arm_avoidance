
## 把持ノードから接続構造を追って、指定距離以内のノードを抽出する (今の所未使用)
ros2 launch ais_gng topological_query.launch.py \
  input_topic:=/topological_map/merged \
  relation_mode:=graph_edges \
  semantic_label:=1 \
  max_euclidean_distance:=0.5 \
  max_hops:=-1