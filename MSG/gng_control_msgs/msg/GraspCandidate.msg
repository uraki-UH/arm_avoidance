# 候補集合内のID、入力座標系でのTCP姿勢、形状スコア
uint32 id
geometry_msgs/Pose pose
float32 shape_score

# TCP位置の到達範囲。姿勢到達性・衝突回避・把持成功の保証なし
uint8 UNKNOWN=0
uint8 INSIDE=1
uint8 OUTSIDE=2
uint8 state
