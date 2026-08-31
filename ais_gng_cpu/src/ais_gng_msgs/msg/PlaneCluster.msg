# 既存のGNGノードグラフから得る、連結した平面パッチ。
# `node_indices` は、このメッセージを生成した TopologicalMap.nodes 配列の
# 添字を指す。永続的なグローバルノードIDを前提としないため、地図内の添字として
# 扱う。
uint32 id
uint8 source_label
uint32[] node_indices

geometry_msgs/Point32 centroid
geometry_msgs/Vector3 normal
geometry_msgs/Vector3 tangent_u
geometry_msgs/Vector3 tangent_v

# 所属GNGノード位置の母共分散[m^2]。行優先の3x3行列:
# [xx, xy, xz, yx, yy, yz, zx, zy, zz]
# GNGノード自身の入力残差共分散ではなく、クラスタの空間的な広がりを表す。
float32[9] position_covariance

# 推定した凸包の外形は出力しない。平面クラスタは凹形状や未観測の穴を含み得るため、
# 利用側は `boundary` を塗りつぶし済みの衝突面として扱ってはならない。
# 将来、観測境界を追跡して格納するために予約している。
geometry_msgs/Point32[] boundary

# 連続する2点は、この構造的な連結成分で実際に観測されたGNGエッジである。
# 凸包を推定せず、凹部と穴を保持する。
geometry_msgs/Point32[] support_edges

# グラフだけから得たクラスタでは、凸包による面積推定を意図的に行わない。
# 観測面積の推定器を導入するまで、この値は0とする。
float32 area
float32 extent_u
float32 extent_v
float32 local_spacing
float32 planarity
float32 residual_ratio
