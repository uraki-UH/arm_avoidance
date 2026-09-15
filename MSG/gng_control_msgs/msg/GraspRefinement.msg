# 元候補集合内のIDと、IK初期値のアームGNGノードID
uint32 source_candidate_id
int32 seed_node_id -1
geometry_msgs/Pose source_pose
geometry_msgs/Pose refined_pose
# 観測した対向接触点。has_contact_pairによる有効性判定
geometry_msgs/Point left_contact
geometry_msgs/Point right_contact
bool has_contact_pair
# 局所点群を閉じ軸へ投影した観測幅 [m]。欠損面を含む物体全幅の保証なし、未計算はNaN
float64 observed_width
# 接触時の内幅と進入時の内幅 [m]。未計算はNaN
float64 contact_width
float64 opening_width
uint32 left_support_num
uint32 right_support_num
# 近似指・基部・進入掃引領域と観測点との検査。未観測空間の安全保証なし
bool has_gripper_check
bool has_observed_collision
# IK成立と関節角。全身衝突・経路検証とは独立
bool has_joint_solution
sensor_msgs/JointState joint_state
# 出力関節角の順運動学による実TCP姿勢。refined_poseは目標姿勢
geometry_msgs/Pose joint_pose
float64 position_error
float64 orientation_error_deg
# 現行の並列評価ではfalse。既存の経路成立・feasibleの継承なし
bool has_arm_path_check
string reason
