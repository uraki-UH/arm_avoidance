# 入力座標系と観測時刻。配信元の生存期間内で単調増加する候補集合ID
std_msgs/Header header
uint64 update_id
GraspCandidate[] candidates

# 到達セルの評価座標系・時刻・寸法・原点。計画GNGとのセル対応用
std_msgs/Header evaluation_header
float64 voxel_size
geometry_msgs/Point voxel_origin
