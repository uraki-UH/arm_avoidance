# 平面クラスタのどれにも属さないGNGノードの連結塊。
#
# 面から突出した物体の手がかりとして使う。衝突判定では、平面クラスタだけを
# 衝突面として扱うとこの塊を見落とすため、突出量の情報を併せて持たせる。
#
# ノード添字は、このメッセージを生成した TopologicalMap.nodes 配列に対するもの。


# 塊の識別子。フレーム間の追跡は行わないため、同一フレーム内でのみ一意。
uint32 id

# この塊のノードは PlanarResidualBlobArray.node_indices の
# [node_begin, node_begin + node_num) に入る。
#
# 塊ごとに配列を持たせると1フレームあたり数十回のヒープ確保が走り、処理時間の
# テールが跳ねる(実測: p90 3.6ms -> 8.5ms)。共有配列への区間参照に変えることで
# 確保を1回に抑える。
uint32 node_begin
uint32 node_num

# 最も多くのGNGエッジで接している平面クラスタのID。接触なしの場合は0。
uint32 adjacent_cluster_id
# その平面クラスタへ伸びるGNGエッジの本数。
uint32 adjacent_edge_num
# 接している平面クラスタの法線。adjacent_cluster_id が0の場合は零ベクトル。
geometry_msgs/Vector3 adjacent_normal

geometry_msgs/Point32 centroid

# 接する平面からのノード距離の最大値[m]。突出の高さ。
float32 max_plane_dist
# 同じ距離の中央値[m]。
#
# 最大値だけでは、面の境界に沿って伸びた薄い帯と、面から浮いた物体を区別できない。
# 帯は中央値がほぼ0に留まり、物体は中央値も最大値に近い値になる。
float32 median_plane_dist

# 塊の最大差し渡し[m]。重心からの最遠ノード距離の2倍。
float32 extent
