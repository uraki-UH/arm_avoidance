#pragma once

#include <ais_gng_msgs/msg/planar_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <memory>

namespace fuzzrobo::topological_plane::incremental
{

// 平面クラスタ生成のパラメータ。
//
// 距離のしきい値はすべて、そのノードの局所ノード間隔(隣接ノードまでの平均距離)で
// 正規化した比で指定する。GNGの密度が場所によって変わっても同じ値が使えるようにする。
struct ClusterOptions
{
  // 種候補を平面クラスタとして確定するための最小ノード数。
  std::size_t min_cluster_nodes = 7;

  // 未所属ノードを取り込むときの、平面までの正規化距離の上限。
  // |平面法線・(ノード位置 - クラスタ重心)| / max(クラスタ間隔, ノード間隔) で計算する。
  // 共分散の固有値比ではない。
  double growth_residual_ratio = 0.70;

  // 同じ距離比で、すでに所属しているノードを保持し続ける上限。
  // 取り込みより緩くすることでヒステリシスを作り、境界ノードの往復を防ぐ。
  double retention_residual_ratio = 1.40;

  // ノード法線とクラスタ平面法線の許容角度[deg]。法線の正負は区別しない。
  double normal_alignment_deg = 60.0;

  // クラスタとして確定するために要求する平面性。sqrt(第2固有値 / 第3固有値)であり、
  // 厚みではなく「線分状でないこと」を見る。
  double min_cluster_planarity = 0.45;

  // クラスタとして確定するために要求する、正規化残差(厚み / 間隔)の上限。
  double max_normalized_cluster_residual = 0.70;

  // 成長の途中で要求する平面性の下限。確定時のしきい値より緩くする。
  //
  // 共分散の第2固有値が第3固有値に対して小さすぎる領域、つまり鎖状に伸びた塊を、
  // 育ちきる前に止めるための条件。確定時だけ見ていると、鎖が最後まで伸びてから
  // 棄却され、同じノードを毎フレーム試し直すことになる。
  double min_growth_planarity = 0.25;

  // 取り込み・移動・併合に共通して要求するGNG接続証拠の本数。
  // 1本だけの偶然の接続による所属の漏れ出しや面の橋渡しを防ぐ。
  std::size_t connection_requirement = 2;

  // 新しいクラスタを育てる途中で要求する、生成中クラスタ内の隣接ノード数。
  // 生成直後は競合相手がいないため小さくてよい。
  std::size_t birth_neighbor_requirement = 1;

  // 移動を認めるために必要な、正規化距離の改善量。振動を止めるための余裕。
  //
  // 環境が静止していても所属が流動的なのは、境界で判定が拮抗し、GNG法線の揺れ
  // (実測で毎フレーム約4度)で行き先が入れ替わるためである。この値が効く。
  // 実測(同一フレーム列100枚): 0.05 で移動9202・往復45.1%、0.30 で移動5799・往復37.8%。
  double migration_improvement_margin = 0.30;

  // 大クラスタ側への任意移動機能のON/OFF。面の統合はクラスタ併合を優先。
  bool enable_larger_cluster_migration_bias = false;

  // 大クラスタ側への任意移動で許容するscore悪化量のしきい値。
  double migration_size_bias_th = 0.10;

  // 大きい側とみなすために必要なノード数の比。
  //
  // 僅差で「大きい方」を決めると、サイズが拮抗した2クラスタ間で毎フレーム大小が
  // 入れ替わり、かえって往復が増える。はっきり大きい相手にだけ寄せる。
  double migration_size_bias_ratio = 2.0;

  // ノードを供給しないクラスタの大きさ。min_cluster_nodes にこの数を足した以下なら、
  // 移動でメンバーを失わない。
  //
  // 小さいクラスタが境界から少しずつ吸われて消えるのを防ぐ。まとまるべきなら
  // クラスタ併合として一括で行われるべきで、削り取られて消えるのは別物である。
  std::size_t donor_protection_buffer = 3;

  // 複数接続ノードに対する保持用距離上限緩和機能のON/OFF。
  bool enable_multi_edge_dist_relaxation = true;

  // 保守点検(移動・取り込み)を1フレームで回す上限回数。
  std::size_t maintenance_iter = 2;

  // 併合を認めるために、結合後の残差が元のクラスタから悪化してよい倍率。
  //
  // 残差の絶対値だけで見ると、小さなクラスタ同士は何をつないでも通ってしまう。
  // 「つないでも当てはめが悪くならない」ことを併せて要求する。
  double merge_residual_growth_ratio = 1.3;

  // 上の倍率を適用する下限。元の当てはめが十分よい場合に、僅かな悪化まで
  // 拒否しないための余裕。
  double merge_residual_growth_min_th = 0.15;

  // 新しいクラスタを出力に載せるまでに、生き残る必要があるフレーム数。
  //
  // 生まれてすぐ消えるクラスタを表示しないための条件。実測では毎フレーム約4.5個が
  // 生まれ約4.2個が消えており、その多くが数フレームしか保たない。確認できるまで
  // 出力しないことで、画面上に「できては消える」塊が現れなくなる。0 なら即座に出す。
  std::size_t birth_confirm_frames = 3;

  // クラスタを実際に分割するまでに、接続が切れた状態が続く必要があるフレーム数。
  //
  // 境界ノードの解放で接続は一時的によく切れる。そのたびに分割すると、大きな
  // クラスタが割れて別IDになり、次のフレームで戻るという出入りを繰り返す。
  // 0 なら即座に分割する。
  std::size_t split_confirm_frames = 3;

  // 条件を満たさないまま許容するフレーム数。超えるとクラスタを破棄する。
  // 条件を満たさないまま許容するフレーム数。
  // 不健全なクラスタは縮んで持ち直そうとするが、実測ではその収束に数フレーム掛かる。
  // 2 では間に合わず満サイズのまま淘汰されていた(消滅238件 -> 5 で182件)。
  std::size_t weak_frame_allowance = 5;
};

// 1フレーム分の処理内訳。定常状態に入れば変化量はすべて0に落ち着く。
struct ClusterStatistics
{
  std::size_t valid_node_count = 0;
  std::size_t usable_node_count = 0;
  std::size_t cluster_count = 0;
  std::size_t clustered_node_count = 0;

  std::size_t released_node_count = 0;
  std::size_t migrated_node_count = 0;
  std::size_t absorbed_node_count = 0;
  std::size_t born_cluster_count = 0;
  std::size_t merged_cluster_count = 0;
  std::size_t split_cluster_count = 0;
  std::size_t removed_cluster_count = 0;
  std::size_t maintenance_iter_num = 0;

  // 隣接クラスタ対が併合判定のどこで止まったかを示す診断値。
  std::size_t merge_adjacent_pair_count = 0;
  std::size_t merge_insufficient_edge_pair_count = 0;
  std::size_t merge_invalid_fit_pair_count = 0;
  std::size_t merge_planarity_rejected_pair_count = 0;
  std::size_t merge_absolute_residual_rejected_pair_count = 0;
  std::size_t merge_residual_growth_rejected_pair_count = 0;

  // 鎖状(第2固有値が第1固有値に対して小さすぎる)として捨てた領域の数。
  std::size_t chain_rejected_count = 0;

  std::size_t clustered_default_node_count = 0;
  std::size_t clustered_terrain_node_count = 0;
  std::size_t clustered_wall_node_count = 0;
  std::size_t clustered_unknown_node_count = 0;
  std::size_t clustered_human_node_count = 0;
  std::size_t clustered_car_node_count = 0;
  std::size_t clustered_other_node_count = 0;
};

struct ClusterResult
{
  ais_gng_msgs::msg::PlanarClusterArray clusters;
  ClusterStatistics statistics;
};

// GNGの位相地図から平面クラスタを生成する、増分方式の実装。
//
// フレームごとに全体を作り直さず、GNGノードID単位の所属を持ち越して差分だけ直す。
// 1フレームの処理はノード数Nとエッジ数Eに対して O(N + E) と、クラスタ数ぶんの
// 3x3固有値分解で済む。優先度付きキューも、クラスタ同士の総当たりも使わない。
//
// 所属が動くのは次の場合だけで、それ以外のノードは前フレームの所属を保つ。
//   - 所属クラスタの平面から離れすぎた                             -> 解放
//   - 未所属で、あるクラスタに所属済みの隣接が規定数以上あり条件を満たす -> 取り込み
//   - 別クラスタに所属済みの隣接が規定数以上あり、明確により適合する    -> 移動
class Clusterizer
{
public:
  explicit Clusterizer(ClusterOptions options = ClusterOptions{});
  ~Clusterizer();

  Clusterizer(const Clusterizer &) = delete;
  Clusterizer &operator=(const Clusterizer &) = delete;

  // 1フレーム分の地図を取り込み、更新後の平面クラスタを返す。
  ClusterResult update(const ais_gng_msgs::msg::TopologicalMap &map);

  // 保持している所属をすべて捨てる。地図の系列が切り替わったときに使う。
  void reset();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // fuzzrobo::topological_plane::incremental 名前空間
