uint8 DEFAULT = 0
uint8 SAFE_TERRAIN = 1
uint8 WALL = 2
uint8 UNKNOWN_OBJECT = 3
uint8 HUMAN = 4
uint8 CAR = 5

uint8 SEMANTIC_DEFAULT = 0
uint8 SEMANTIC_HANDLE = 1
# 把持候補の位置到達性。環境分類labelとは独立の意味ラベル
uint8 SEMANTIC_GRASP_UNKNOWN = 2
uint8 SEMANTIC_GRASP_INSIDE = 3
uint8 SEMANTIC_GRASP_OUTSIDE = 4

std_msgs/Header header
uint32 frame_number
ais_gng_msgs/TopologicalNode[] nodes
uint16[] edges
ais_gng_msgs/TopologicalCluster[] clusters
