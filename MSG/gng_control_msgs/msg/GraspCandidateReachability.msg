# 入力PoseArray内の添字と元の把持姿勢
int32 candidate_idx
geometry_msgs/Pose pose
float32 shape_score

# 評価座標系での把持姿勢と位置到達性。方向の到達性は評価対象外
geometry_msgs/Pose evaluated_pose
bool has_evaluation
bool is_in_reachability
bool can_plan

# 未評価理由、領域外によるスキップ、計画対象などの状態
string reason
# 元の把持候補に対応する計画対象GNGノードID
int32[] goal_node_ids
