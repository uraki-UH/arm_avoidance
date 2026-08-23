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
  // 出力クラスタに要求する最小ノード数。
  std::size_t min_cluster_nodes = 7;

  // 未所属ノードを取り込むときの、平面までの正規化距離の上限。
  double growth_residual_ratio = 0.70;

  // すでに所属しているノードを保持し続ける、平面までの正規化距離の上限。
  // 取り込みより緩くすることでヒステリシスを作り、境界ノードの往復を防ぐ。
  double retention_residual_ratio = 1.40;

  // ノード法線とクラスタ平面法線の整合下限(内積の絶対値)。
  double normal_alignment_cos = 0.50;

  // クラスタとして確定するために要求する平面性。sqrt(第2固有値 / 第3固有値)であり、
  // 厚みではなく「線分状でないこと」を見る。
  double min_cluster_planarity = 0.45;

  // クラスタとして確定するために要求する、正規化残差(厚み / 間隔)の上限。
  double max_normalized_cluster_residual = 0.70;

  // 成長の途中で要求する平面性の下限。確定時のしきい値より緩くする。
  //
  // 共分散の第2固有値が第1固有値に対して小さすぎる領域、つまり鎖状に伸びた塊を、
  // 育ちきる前に止めるための条件。確定時だけ見ていると、鎖が最後まで伸びてから
  // 棄却され、同じノードを毎フレーム試し直すことになる。
  double min_growth_planarity = 0.25;

  // 未所属ノードを既存クラスタへ取り込むために要求する、
  // 「そのクラスタにすでに所属している隣接ノード」の数。
  std::size_t absorb_neighbor_requirement = 2;

  // 所属済みノードを別クラスタへ移すために要求する、
  // 「移動先クラスタにすでに所属している隣接ノード」の数。
  // 1本のエッジだけで所属が漏れ出すのを防ぐ条件。
  std::size_t migration_neighbor_requirement = 2;

  // 新しいクラスタを育てる途中で要求する、生成中クラスタ内の隣接ノード数。
  // 生成直後は競合相手がいないため小さくてよい。
  std::size_t birth_neighbor_requirement = 1;

  // 移動を認めるために必要な、正規化距離の改善量。振動を止めるための余裕。
  double migration_improvement_margin = 0.05;

  // 同一クラスタのノードから複数のエッジが伸びているノードは、平面までの距離を
  // 問わずそのクラスタの所属とする。
  //
  // 所属の根拠を「平面までの絶対距離」ではなく「接続と法線一致」に置く。実機では
  // 机の面が局所ノード間隔の 0.3〜0.4 ぶんうねっており、距離だけで切ると面が途中で
  // 分断された。距離は候補が複数あるときの優先順位にだけ使う。
  bool coplanar_multi_edge_overrides_distance = true;

  // 保守点検(移動・取り込み)を1フレームで回す上限回数。
  std::size_t maintenance_iterations = 2;

  // 2つのクラスタを併合判定にかけるために必要な、両者を直接つなぐGNGエッジ本数。
  std::size_t merge_edge_requirement = 2;

  // 併合の候補にするための法線一致の下限。
  //
  // 併合の可否は「結合したら1枚の平面として成立するか」で決めるので、ここは
  // 高い判定を回す前のふるいにすぎない。厳しくしすぎると、実際には同じ面である
  // 対まで判定に到達しなくなる。実測では 0.95 だと真に合体すべき対の 7% しか
  // 到達できなかった。
  double merge_normal_alignment_cos = 0.80;

  // 併合を認めるために、結合後の残差が元のクラスタから悪化してよい倍率。
  //
  // 残差の絶対値だけで見ると、小さなクラスタ同士は何をつないでも通ってしまう。
  // 「つないでも当てはめが悪くならない」ことを併せて要求する。
  double merge_residual_growth_ratio = 1.3;

  // 上の倍率を適用する下限。元の当てはめが十分よい場合に、僅かな悪化まで
  // 拒否しないための余裕。
  double merge_residual_growth_floor = 0.15;

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
  std::size_t weak_frame_allowance = 2;
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
  std::size_t maintenance_iterations_used = 0;

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
