uint16 id
geometry_msgs/Point32 pos
geometry_msgs/Point32 normal
float32 rho
uint8 label
uint8 semantic_label
float32 semantic_reliability
bool is_goal
uint32 frame
uint32[] inpcl_ids
uint32 winner_point_count
float32[9] winner_point_covariance
uint32 NONPLANE_COMPONENT_NONE = 4294967295
uint32 nonplane_component_id
# GNG側で判定した低次数の境界候補。真の境界・遮蔽の確定情報ではなく一次候補。
bool is_boundary_candidate
# 境界候補付近の観測証拠。独立ビットの併存、0は不明。物体の端の確定情報ではない。
uint8 BOUNDARY_OCCLUSION = 1
uint8 BOUNDARY_FREE_SPACE = 2
uint8 BOUNDARY_FIELD_OF_VIEW = 4
uint8 boundary_evidence
