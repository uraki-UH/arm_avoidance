uint8 DEFAULT = 0
uint8 SAFE_TERRAIN = 1
uint8 WALL = 2
uint8 UNKNOWN_OBJECT = 3
uint8 HUMAN = 4
uint8 CAR = 5

std_msgs/Header header
ais_gng_msgs/TopologicalNode[] nodes
uint16[] edges
ais_gng_msgs/TopologicalCluster[] clusters
# Field required by FuzzyClassifier
uint32 frame_number