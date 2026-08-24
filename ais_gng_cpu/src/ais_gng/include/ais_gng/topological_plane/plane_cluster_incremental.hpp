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

  // growth/retention_residual_ratio の正規化に使う、実効ノード間隔の上限[m]。
  //
  // fitScoreは生の距離を局所間隔(cluster.spacing, node.spacing のうち大きい方)で
  // 割った比率であり、間隔が粗い領域ほど「同じ比率でも許容される実距離」が
  // 際限なく大きくなる。卓上の小物体(数cm程度の高さ)が、間隔の粗い床などの上で
  // ノイズとして紛れ込み、合体と分離を繰り返す原因になっていた。実測(密集領域
  // 150フレーム): 局所間隔の中央値2.2cm・p90 4.1cmに対し、実際に不安定だった
  // クラスタは3.3〜3.5cmと粗い側に偏っていた。分母をこの値で頭打ちにすることで、
  // 間隔が粗い場所でも許容距離が青天井にならないようにする。
  double max_effective_spacing = 0.03;

  // ノード法線に掛ける指数移動平均(EMA)の混合率。1.0ならフィルタなし(生の値を
  // そのまま使う)。小さいほど平滑化が強く、追従が遅くなる。
  //
  // GNG法線のフレーム間角度変化は、平均こそ約4度だが分布の裾が非常に重く、
  // p99.9で約86度に達する(実測)。これは境界・稜線ノードの瞬間的な推定誤差が
  // 主因で、フィルタで大きく縮む(alpha=0.3でp99.9が約22度、alpha=0.05で約3度)。
  // 把持候補の評価・選定にこのクラスタを使う用途を考え、追従の速さを優先して
  // 0.30を既定にする。ここでノイズを減らすのが根治療法で、しきい値を緩める
  // (normal_alignment_deg等)のは対症療法にすぎない。
  double normal_filter_alpha = 0.30;

  // 種の生成・成長で要求する、ノード法線とクラスタ平面法線の許容角度[deg]。
  // 法線の正負は区別しない。
  //
  // 「これは同じ面の種として妥当か」を判定する場面でだけ厳しく見る。ここは
  // retention_normal_alignment_deg とは別に厳しいままにする。
  double normal_alignment_deg = 60.0;

  // 既に所属しているノードの保持・取り込み・移動で要求する、法線の許容角度[deg]。
  //
  // 種の判定用より緩くする。フィルタ導入後もこの緩和は保険として残す。実測
  // (同一フレーム列150枚、フィルタなし): 70度のままだと所属変化が273.6件/
  // フレームだったが、85度まで緩めると112.5件/フレーム(-59%)に減り、往復率は
  // 悪化しなかった(23.9% -> 22.5%)。直交面(90度差)は85度でも合成データ上は
  // 分離を維持することを確認済み。
  double retention_normal_alignment_deg = 85.0;

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

  // ノードの取り込み・移動で要求するGNG接続証拠の本数。
  // 1本だけの偶然の接続による所属の漏れ出しを防ぐ。ノード1個の判定はその1点が
  // 受け入れ先の平面に合うかしか見ない(統計的な頑健さがない)ため、緩めるとポロポロ
  // 誤って移る。
  std::size_t connection_requirement = 2;

  // クラスタ併合の候補と認めるために要求する、クラスタ間の直接GNGエッジ本数。
  //
  // 併合は数百点対数百点の結合フィット(merge_min_planarity・
  // merge_residual_growth_ratio等)で最終判定するため、統計的に頑健で、1本の
  // 偶然のエッジが混ざっても弾かれやすい。実測(同一フレーム列150枚、
  // merge_residual_growth_ratio=1.1固定): connection_requirementを2→1相当に
  // 下げると併合成立+22%(81→99件)。一方、この値を併合専用に分離せず
  // connection_requirementそのものを1に下げた場合はノード単体の取り込みが+55%
  // (2014→3115件)・移動が+107%(208→430件)と、狙いより副作用の方が大きく出た。
  // そのため併合だけを独立して緩める専用値として分離してある。
  std::size_t merge_connection_requirement = 1;

  // 新しいクラスタを育てる途中で要求する、生成中クラスタ内の隣接ノード数。
  // 生成直後は競合相手がいないため小さくてよい。
  std::size_t birth_neighbor_requirement = 1;

  // 移動を認めるために必要な、正規化距離の改善量。振動を止めるための余裕。
  //
  // 環境が静止していても所属が流動的なのは、境界で判定が拮抗し、GNG法線の揺れ
  // (実測で毎フレーム約4度)で行き先が入れ替わるためである。この値が効く。
  // 実測(同一フレーム列100枚): 0.05 で移動9202・往復45.1%、0.30 で移動5799・往復37.8%。
  double migration_improvement_margin = 0.30;

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

  // 併合を認めるために要求する、結合後の平面性の下限。確定用の
  // min_cluster_planarity より緩くする。
  //
  // 実測: 直交する床+壁を結合すると平面性は0.620まで上がり、確定用の0.45を
  // 普通に満たしてしまう一方、残差比は1.208(要件0.70)まで悪化して弾かれた。
  // つまり誤併合を防いでいるのは残差条件だけで、平面性条件は防止に寄与していない。
  // 一方で、細長い帯状の領域を追加すると平面性だけが一時的に下がり、絶対残差は
  // 十分小さい(実測0.025〜0.036)のに併合できず、統合されるべきクラスタが
  // 何十フレームも境界を奪い合いながら足踏みする原因になっていた。
  double merge_min_planarity = 0.25;

  // 併合を認めるために、結合後の残差が元のクラスタから悪化してよい倍率。
  //
  // 残差の絶対値だけで見ると、小さなクラスタ同士は何をつないでも通ってしまう。
  // 「つないでも当てはめが悪くならない」ことを併せて要求する。
  double merge_residual_growth_ratio = 1.3;

  // 上の倍率を適用する下限。元の当てはめが十分よい場合に、僅かな悪化まで
  // 拒否しないための余裕。
  double merge_residual_growth_min_th = 0.15;

  // 併合を認めるために要求する、メンバー数が少ない側の点群単体を結合後の平面へ
  // 当てはめたときのRMS残差比の上限。
  //
  // union_fit(結合後の共分散)は全メンバーの平均統計なので、大きいクラスタに
  // 小さいクラスタを混ぜても、小さい側の実際のズレ(例: 卓上の小物体が数cm浮いて
  // いる)が平均に埋もれて見えなくなる。少数側の点群を union の平面に対して
  // 個別に評価することで、この抜け道を塞ぐ。growth_residual_ratioと同じ
  // 「候補点は平面にこれだけ近くなければならない」という基準を、少数側の
  // 点群全体に適用するイメージ。
  double merge_smaller_side_residual_ratio = 0.70;

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
  std::size_t merge_smaller_side_rejected_pair_count = 0;

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
