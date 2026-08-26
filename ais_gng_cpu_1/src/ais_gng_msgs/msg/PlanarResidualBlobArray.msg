# 平面クラスタに属さないノードの連結塊の一覧。
std_msgs/Header header
uint32 frame_number

# 全塊のノード添字を連結したもの。各塊は node_begin / node_num で区間を指す。
uint32[] node_indices

ais_gng_msgs/PlanarResidualBlob[] blobs
