#pragma once

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <memory>

namespace fuzzrobo::topological_plane::incremental
{

// 平面クラスタ生成のパラメータ。
//
// 距離しきい値は局所エッジ長で正規化した比。位置・間隔の一様拡大縮小への追従。
struct ClusterOptions
{
  // 種候補を平面クラスタとして確定するための最小ノード数。
  std::size_t min_cluster_nodes = 7;

  // 取り込み用の平面距離比。分母は対象ノードの隣接エッジ長中央値。
  double growth_residual_ratio = 0.15;

  // 同じ距離比で、すでに所属しているノードを保持し続ける上限。
  // 取り込みより緩くすることでヒステリシスを作り、境界ノードの往復を防ぐ。
  double retention_residual_ratio = 0.30;

  // 距離比の分母への任意の絶対上限[m]。0は上限なしのスケール追従。
  double max_effective_spacing = 0.0;

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

  // GNGが計算済みの rho (近傍法線との角度不一致)を、新規クラスタの
  // 種候補順序に使う。平面クラスタ側で同じ近傍法線内積を再計算しない。
  // 古いbagでは rho が未設定の場合があるため、汎用Clusterizerの既定はfalse。
  bool use_node_rho_for_seed_order = false;

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

  // 面内縦横比 sqrt(第2固有値 / 第3固有値)。面幅条件との選択判定。
  double min_cluster_planarity = 0.45;

  // 面内短軸の標準偏差 / 平均局所ノード間隔。縦横比に代わる面幅の判定値。
  // 十分な幅のある長い路面の許容と、一本鎖や幅の乏しい帯の除外用。
  double min_plane_width_ratio = 1.0;

  // 生成・統合時のRMS残差比。分母は所属ノードの局所間隔平均（任意の絶対上限付き）。
  double max_normalized_cluster_residual = 0.15;

  // 成長中の面内縦横比。min_plane_width_ratioによる面幅条件との選択判定。
  double min_growth_planarity = 0.25;

  // ノードの取り込み・移動で要求するGNG接続証拠の本数。
  // 1本だけの偶然の接続による所属の漏れ出しを防ぐ。ノード1個の判定はその1点が
  // 受け入れ先の平面に合うかしか見ない(統計的な頑健さがない)ため、緩めるとポロポロ
  // 誤って移る。
  std::size_t connection_requirement = 2;

  // 未所属ノード限定の面内接続による取り込み救済。既所属間の移動には非適用。
  bool enable_coplanar_absorption = true;
  // 候補から伸びる全エッジと平面との角度[deg]。面外形状の誤吸収防止用。
  double max_absorption_edge_angle_deg_th = 20.0;
  // 平面への接続長 / 両端の局所間隔の小さい側。長い橋エッジの除外用。
  double max_absorption_edge_ratio_th = 2.5;

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

  // 通常2本接続を要求する設定での、小断片に限定した1本接続の救済。
  bool enable_fragment_merge = true;
  // 救済対象となる小さい側の所属ノード数。
  std::size_t max_fragment_nodes = 30;
  // 接続長 / 両端の局所間隔の小さい側。長い橋エッジの除外用。
  double max_fragment_edge_ratio_th = 2.5;
  // 各側・接触部・統合後平面の正規化RMS。通常併合より強い適合条件。
  double max_fragment_residual_ratio_th = 0.10;
  // 同じ永続クラスタ対で幾何条件を満たした連続入力フレーム数。
  std::size_t min_fragment_merge_frames = 3;

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

  // 取り込み・移動時の複数接続ノードに対する距離緩和。既所属の保持判定とは独立。
  bool enable_multi_edge_dist_relaxation = true;

  // 保守点検(移動・取り込み)を1フレームで回す上限回数。
  std::size_t maintenance_iter = 2;

  // 統合後全体の面内縦横比。min_plane_width_ratioによる面幅条件との選択判定。
  double merge_min_planarity = 0.25;

  // 併合を認めるために、結合後の残差が元のクラスタから悪化してよい倍率。
  //
  // 残差の絶対値だけで見ると、小さなクラスタ同士は何をつないでも通ってしまう。
  // 「つないでも当てはめが悪くならない」ことを併せて要求する。
  double merge_residual_growth_ratio = 1.3;

  // 上の倍率を適用する下限。元の当てはめが十分よい場合に、僅かな悪化まで
  // 拒否しないための余裕。
  double merge_residual_growth_min_th = 0.15;

  // 各側全体から統合平面、および接続端点から相手平面へのRMS距離比。
  // 分母は各評価点群の局所間隔平均。小面の法線誤差の遠方外挿の回避。
  // C++識別子は互換維持。ROS設定名はmax_merge_side_residual_ratio。
  double merge_smaller_side_residual_ratio = 0.15;

  // 新しいクラスタを出力に載せるまでに、生き残る必要があるフレーム数。
  //
  // 生まれてすぐ消えるクラスタを表示しないための条件。実測では毎フレーム約4.5個が
  // 生まれ約4.2個が消えており、その多くが数フレームしか保たない。確認できるまで
  // 出力しないことで、画面上に「できては消える」塊が現れなくなる。0 なら即座に出す。
  std::size_t birth_confirm_frames = 3;

  // 分割条件の連続成立を許容するフレーム数。0は即時分割。
  std::size_t split_confirm_frames = 3;

  // 非連結成分の面外エッジ方向による分割確認。falseは従来の接続切断のみの判定。
  bool enable_directional_split = true;
  // 元平面とエッジとの角度[deg]。面外接続の判定用。
  double min_split_edge_angle_deg_th = 30.0;
  // 面外接続を持つノード数と、分断候補成分内の同ノード割合。
  std::size_t min_split_conflict_nodes = 2;
  double min_split_conflict_ratio_th = 0.25;
  // 全エッジ消失時に前回の法線・局所間隔で所属判定を継続するフレーム数。
  std::size_t max_isolated_frames = 5;

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
  // 通常の距離・接続条件では取り込めなかった面内候補の救済数。
  std::size_t num_coplanar_absorbed_nodes = 0;
  std::size_t born_cluster_count = 0;
  std::size_t merged_cluster_count = 0;
  std::size_t split_cluster_count = 0;
  std::size_t removed_cluster_count = 0;
  std::size_t maintenance_iter_num = 0;

  // 全域木の維持によって連結探索を省略したクラスタ数。
  std::size_t num_connectivity_reused_clusters = 0;
  // 連結成分の再探索で訪問したノード数。
  std::size_t num_connectivity_scanned_nodes = 0;
  // 面外根拠不足・連続確認待ちの非連結成分数と、全エッジ消失の猶予対象ノード数。
  std::size_t num_split_retained_components = 0;
  std::size_t num_split_pending_components = 0;
  std::size_t num_isolated_retained_nodes = 0;

  // 隣接クラスタ対が併合判定のどこで止まったかを示す診断値。
  std::size_t merge_adjacent_pair_count = 0;
  std::size_t merge_insufficient_edge_pair_count = 0;
  std::size_t merge_invalid_fit_pair_count = 0;
  std::size_t merge_planarity_rejected_pair_count = 0;
  std::size_t merge_absolute_residual_rejected_pair_count = 0;
  std::size_t merge_residual_growth_rejected_pair_count = 0;
  std::size_t merge_smaller_side_rejected_pair_count = 0;
  // 1本接続の小断片救済による統合数と連続確認待ち対数。
  std::size_t num_fragment_merged_clusters = 0;
  std::size_t num_fragment_pending_pairs = 0;

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
  ais_gng_msgs::msg::PlaneClusterArray clusters;
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

  // 次フレームから新規クラスタの種順序にGNG rhoを使うかを切り替える。
  // 既存クラスタの所属状態はリセットしない。
  void setUseNodeRhoForSeedOrder(bool enabled);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // fuzzrobo::topological_plane::incremental 名前空間
