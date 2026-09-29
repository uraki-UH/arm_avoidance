#pragma once

#include <ais_gng_msgs/msg/plane_cluster_array.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstddef>
#include <cstdint>
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
  double min_plane_aspect_ratio = 0.45;

  // 面内短軸の標準偏差 / 平均局所ノード間隔。縦横比に代わる面幅の判定値。
  // 十分な幅のある長い路面の許容と、一本鎖や幅の乏しい帯の除外用。
  double min_plane_width_ratio = 1.0;

  // 生成・統合時のRMS残差比。分母は所属ノードの局所間隔平均（任意の絶対上限付き）。
  double max_normalized_cluster_residual = 0.15;

  // ノードの取り込み・移動で要求するGNG接続証拠の本数。
  // 1本だけの偶然の接続による所属の漏れ出しを防ぐ。ノード1個の判定はその1点が
  // 受け入れ先の平面に合うかしか見ない(統計的な頑健さがない)ため、緩めるとポロポロ
  // 誤って移る。
  std::size_t connection_requirement = 2;

  // 未所属ノード限定の面内接続による取り込み救済。既所属間の移動には非適用。
  bool enable_coplanar_absorption = true;
  // 候補から伸びる全エッジと平面との角度[deg]。面外形状の誤吸収防止用。
  double max_absorption_edge_angle_deg_th = 20.0;
  // 接続長 / 両端の局所間隔の小さい側。取り込み・断片統合共通の橋エッジ除外用。
  double max_connection_edge_ratio_th = 2.5;

  // クラスタ併合候補の直接GNGエッジ要求本数。単一点の取り込み条件から独立。
  // 統合後の面幅・厚みと各側の残差による最終判定。
  std::size_t merge_connection_requirement = 1;

  // 通常2本接続を要求する設定での、短い1本接続の連続適合による救済。
  bool enable_fragment_merge = true;
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
  // 不健全なクラスタは縮んで持ち直そうとするが、実測ではその収束に数フレーム掛かる。
  // 2 では間に合わず満サイズのまま淘汰されていた(消滅238件 -> 5 で182件)。
  std::size_t weak_frame_allowance = 5;

  // 試作の差分統計。位置・間隔・所属の変化分のみの加減算。
  bool enable_delta_statistics = true;
  // 試作の保持証明キャッシュ。入力不変ブロックの安全な保持判定省略。
  bool enable_block_retention = false;
  // 平面所属エッジの座標複製。表示・元グラフを受け取らない利用側向けの互換出力。
  bool enable_support_edges = false;
  // 比較用の判定平面固定・所属1パス更新。収束処理の次フレームへの継続。
  bool enable_temporal_update = false;
  // 移籍・生成の候補分散周期[入力フレーム]。1は毎回、逸脱解放・未所属点の取り込みは毎回。
  std::size_t num_acquisition_phases = 5U;
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
  // 保持証明による個別判定の省略ノード数。
  std::size_t num_retention_reused_nodes = 0;

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

// 平面計算に必要な属性のみの入力値。ROS・GNGの所有配列から読出し。
struct node_input
{
  struct coordinate {float x, y, z;};
  std::uint16_t id;
  std::uint8_t label;
  float rho;
  coordinate pos, normal;
  // 同一IDの別ノードを区別するGNG生成フレーム。
  std::uint32_t frame = 0U;
};

// update呼出し中だけ有効な借用ビュー。接続値はノード配列添字の対、IDではない。
struct graph_view
{
  const void *nodes = nullptr;
  std::size_t num_nodes = 0U;
  const std::uint16_t *edges = nullptr;
  std::size_t num_edge_values = 0U;
  node_input (*read_node)(const void *, std::size_t) = nullptr;
};

// 元配列の所有権・並び順・精度を維持したアダプター。ノード配列の中間コピーなし。
template<typename node_type>
graph_view make_graph_view(const node_type *nodes, const std::size_t num_nodes,
  const std::uint16_t *edges, const std::size_t num_edge_values)
{
  return {nodes, num_nodes, edges, num_edge_values,
    [](const void *data, const std::size_t idx) -> node_input {
      const auto &node = static_cast<const node_type *>(data)[idx];
      return {node.id, node.label, node.rho,
        {node.pos.x, node.pos.y, node.pos.z}, {node.normal.x, node.normal.y, node.normal.z}, node.frame};
    }};
}

// GNGの位相地図から平面クラスタを生成する、増分方式の実装。
//
// GNGノードIDと生成フレームによる所属の持ち越し。試作オプションで統計の差分加減算へ切替。
// 主走査はノード数Nとエッジ数Eに対してO(N + E)、未所属U点の種順序はO(U log U)。
// 加えて領域成長・平面の3x3固有値分解。優先度付きキュー・全クラスタ対総当たりなし。
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
  // 内部グラフの直接入力。ビューと元配列の寿命は呼出し終了まで、結果は自己所有。
  ClusterResult update(const graph_view &map, const std_msgs::msg::Header &header,
    std::uint32_t frame_number);

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
