#ifndef SPATIAL_TREE_MOVING_BSP_TREE_HPP
#define SPATIAL_TREE_MOVING_BSP_TREE_HPP

/**
 * @file MovingBSPTree.hpp
 * @brief 動く点群（GNG のノードなど）向けの、占有数で分割・併合する軸平行 BSP 木。
 *        10次元程度で使うことを想定している（2^Dim 分岐の AdaptiveTree は
 *        Dim が大きいと子の数が爆発するため）。
 *
 * 設計:
 *   - 各ノードは二分の軸平行 split（x[axis] < threshold → 左, それ以外 → 右）。
 *   - 全ノードが祖先 split の積を 2*Dim 個の境界にまとめた box [lo, hi) を持つ。
 *     根は全空間（番兵値 lowest/max）なので、ワールド範囲の指定は要らない。
 *   - box の各境界には、その境界を作った祖先 split へのポインタ（owner）を持つ。
 *   - 移動時:
 *       1) 新座標が今の葉の box 内なら O(Dim) の比較だけで終わる。
 *       2) 外に出たら、破った境界の owner（最も浅いもの）へ直接ジャンプする。
 *          そこの box にも入っていなければ、同じことをもう一度やる。
 *          box に入った最初のノードが旧葉と新葉の LCA になる。
 *       3) LCA の反対側の部分木を新座標で降りる。
 *   - 一度作った split の閾値は動かさない。古くなった split は
 *     部分木ごと併合（collapse）または再構築（rebuild）して作り直す。
 *     これにより、子孫の box を書き換える処理が発生しない。
 *
 * 探索:
 *   - 深さ優先で、近い側の子から降りる。遠い側の子は、cell 領域までの距離
 *     （祖先 split から O(1) で逐次更新）と、締まった bbox までの距離の
 *     大きい方を下界にして枝刈りする。
 *   - 葉は座標を 8点ずつ軸ごとに並べて持ち（SoA）、要素ポインタも同じブロックに
 *     入れる。8点の距離は SIMD でまとめて計算する。10次元ではノード1つを辿る
 *     コストが点1つの距離計算の 20倍程度あるので、葉は次元に応じて大きめにする
 *     （MovingBSPParams::tunedLeafSize、既定は自動）。
 *   - 遠い側の子は、まず O(1) の領域距離で判定し、通ったときだけノードを読む。
 *   - 上位2件までの探索では、葉のブロックから上位2点だけを候補に入れる
 *     （低次元で有効。kTopTwoScan）。
 *   - 32次元以上では、葉の走査を 8軸ごとに区切り、部分和がすでに今の k 番目
 *     より遠いブロックは残りの軸を計算しない（厳密なまま）。分散の大きい軸が
 *     先頭に来る座標系ほど効くので、高次元データは PCA 座標に回してから
 *     入れるとよい（直交変換なので距離は変わらない）。
 *   - approx_eps > 0 で (1+eps) 近似探索になる。点が実質的に高次元に
 *     広がるデータほど効く。ただし内在次元が数十を超えると距離が集中し、
 *     (1+eps) の保証がほとんど何も絞らなくなって正解率が大きく落ちる。
 *
 * 部分木の要素数:
 *   併合・再構築の判定に部分木の要素数を使うため、境界を跨いだ移動では
 *   旧葉から LCA までの祖先の要素数を1ずつ減らす（親ポインタを辿る）。
 *   owner ジャンプで省けるのは「どの祖先まで戻るか」を探す box 判定であり、
 *   この整数の減算ではない。
 *
 * 約束事:
 *   - 葉は要素の座標を複製して持つ（LeafStore）。探索はこの複製だけを
 *     読む。したがって座標は必ず updatePosition 経由で変えること。
 *     Traits::setPosition で直接書き換えると複製が古いまま残る。
 *     そのため lazy_neighbors を要求するポリシーとは組み合わせられない。
 *   - HysteresisPolicy::enabled のとき、要素は葉の box から各軸 margin だけ
 *     はみ出して残ることがある。探索側にも同じ以上の search_margin を渡すと、
 *     探索は「各軸 search_margin だけ膨らませた box」までの距離で枝刈りする
 *     ので取りこぼさない。小さい search_margin を渡すと取りこぼし得る。
 *     NoHysteresis なら常に「要素は自分の葉の box 内」が成り立ち、探索は厳密。
 */

#include "Policy.hpp"
#include "SpatialTree.hpp"
#include "Traits.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace SpatialTree {

namespace detail {
// ポリシーが lazy_neighbors = true を宣言しているか
template <typename P, typename = void>
struct policy_requests_lazy : std::false_type {};
template <typename P>
struct policy_requests_lazy<P, std::void_t<decltype(P::lazy_neighbors)>>
    : std::integral_constant<bool, P::lazy_neighbors> {};
} // namespace detail

/**
 * @brief MovingBSPTree の分割・併合・再構築の基準。
 */
template <typename Scalar> struct MovingBSPParams {
  // 0 以下なら次元から自動で決める（下の tunedLeafSize を参照）。
  // 葉の要素数がこれを超えたら分割する
  int max_leaf_size = 0;
  int merge_threshold = 0;  // 0 以下なら葉の容量の 1/3。これ以下なら1枚の葉へ併合
  int max_depth = 64;       // 安全上の上限（重複点が大量にある場合など）

  // 片側の子の要素数が親の balance_alpha 倍を超えたら、その部分木を
  // 中央値 split で作り直す（scapegoat 木と同じ考え方）。1.0 以上で無効。
  double balance_alpha = 0.8;
  int min_rebuild_size = 0; // 0 以下なら葉の容量の 2倍。これ未満は再構築しない

  // false にすると、境界を跨いだとき親を1段ずつ上って LCA を探す（比較用）
  bool use_owner_jump = true;

  // 締まった bbox（部分木の点が実際にある範囲）でも枝刈りする。
  // 維持のため、移動のたびに O(Dim) の min/max が増える。
  //   -1: 次元から自動  0: 使わない  1: 使う
  // GNG のように点が毎回動く用途では、10次元以下だと維持コストの方が
  // 大きく、使わない方が速い（実測）。静的な点群なら 1 が有利なこともある
  int use_bbox = -1;

  // 分割位置  0: 中央値付近の隙間の中点  1: 広がりの中点（sliding midpoint）
  // 1 は低次元の多様体に乗ったデータで数%速く、高次元に広がるデータでは遅い。
  // 1 は偏った木を作るので、使うときは balance_alpha = 1.0（再構築なし）にする
  int split_rule = 0;

  // 近似探索。0 なら厳密。> 0 なら、下界が「今の k 番目の距離 / (1+eps)」を
  // 超える部分木を捨てる。返る i 番目の近傍の距離は真の i 番目の (1+eps) 倍以内
  double approx_eps = 0.0;

  MovingBSPParams() = default;

  // AdaptiveTree 用のパラメータからの変換（GNG などに差し替えて使う場合）
  explicit MovingBSPParams(const SpatialTreeParams<Scalar> &p)
      : max_leaf_size(p.max_nodes_per_cell),
        merge_threshold(p.min_nodes_for_merge), max_depth(p.max_depth) {}

  // 次元ごとの葉の容量（GNG の学習ループでの実測から）
  static int tunedLeafSize(int dim) {
    if (dim <= 3)
      return 32;
    if (dim <= 7)
      return 48;
    return 64;
  }
};

/**
 * @brief 構造の挙動を観測するためのカウンタ（ベンチマーク・検証用）。
 */
struct MovingBSPStats {
  std::uint64_t updates = 0;         // updatePosition の呼び出し回数
  std::uint64_t stayed_in_leaf = 0;  // box 判定だけで終わった回数
  std::uint64_t crossings = 0;       // 葉を移った回数
  std::uint64_t levels_climbed = 0;  // Σ (旧葉の深さ - LCA の深さ)
  std::uint64_t jump_hops = 0;       // LCA 探索で辿ったノード数（ジャンプ or 1段上り）
  std::uint64_t splits = 0;
  std::uint64_t collapses = 0;
  std::uint64_t rebuilds = 0;
  std::uint64_t rebuilt_elements = 0;
};

template <typename T, typename Scalar = DefaultScalar, int Dim = 2,
          typename Traits = SpatialTraits<T, Scalar, Dim>,
          typename HysteresisPolicy = NoHysteresis>
class MovingBSPTree {
  static_assert(!detail::policy_requests_lazy<HysteresisPolicy>::value,
                "MovingBSPTree は葉に座標の複製を持つため、近傍ノードの座標を"
                "木を通さずに書き換える lazy_neighbors ポリシーとは併用できない");

public:
  using PointT = Point<Scalar, Dim>;

  // 根の「無限遠」境界。-ffast-math では infinity が未定義動作になるので
  // 有限の最大値を番兵に使う（この値ちょうどの座標は根の外扱いになり、
  // 根まで戻って降り直すだけで、壊れはしない）
  static constexpr Scalar kUnboundedLo = std::numeric_limits<Scalar>::lowest();
  static constexpr Scalar kUnboundedHi = std::numeric_limits<Scalar>::max();

  // 分割・併合で要素を一時的に持ち運ぶための組
  struct Entry {
    PointT pos;
    T *element;
  };

  // 葉の要素。座標は kLanes 点ずつ軸ごとに並べて持ち（SoA ブロック）、
  // 探索では kLanes 点の距離を軸ごとにまとめて（SIMD で）計算する。
  // 要素 i の座標は blocks[i / kLanes].c[d][i % kLanes]。
  static constexpr int kLanes = 8;
  struct alignas(32) Block {
    Scalar c[Dim][kLanes];
    T *el[kLanes];
  };

  struct LeafStore {
    // 要素ポインタも座標と同じブロックに持つ。移動の反映（座標の書き換えと
    // 要素の照合）が1つの領域で済み、葉ごとの確保も1回になる
    std::vector<Block> blocks;
    int n = 0;

    int size() const { return n; }
    bool empty() const { return n == 0; }
    T *elem(int i) const { return blocks[i / kLanes].el[i % kLanes]; }
    Scalar coord(int i, int d) const { return blocks[i / kLanes].c[d][i % kLanes]; }
    PointT pos(int i) const {
      PointT p;
      const Block &b = blocks[i / kLanes];
      for (int d = 0; d < Dim; ++d)
        p[d] = b.c[d][i % kLanes];
      return p;
    }
    void setPos(int i, const PointT &p) {
      Block &b = blocks[i / kLanes];
      for (int d = 0; d < Dim; ++d)
        b.c[d][i % kLanes] = p[d];
    }
    void push(T *el, const PointT &p) {
      const int i = n;
      if (i % kLanes == 0)
        blocks.push_back(Block{}); // 未使用レーンも有限値（0）にしておく
      blocks[i / kLanes].el[i % kLanes] = el;
      setPos(i, p);
      ++n;
    }
    // i 番目を末尾の要素で埋めて1つ減らす
    void swapPop(int i) {
      const int last = n - 1;
      if (i != last) {
        blocks[i / kLanes].el[i % kLanes] = elem(last);
        setPos(i, pos(last));
      }
      --n;
      if (last % kLanes == 0)
        blocks.pop_back();
    }
    void clear() {
      blocks.clear();
      n = 0;
    }
    void release() {
      std::vector<Block>().swap(blocks);
      n = 0;
    }
    void reserve(int cap) { blocks.reserve((cap + kLanes - 1) / kLanes); }
  };

  // 部分木の点が実際にある範囲（締まった bbox）。移動では広げるだけで、
  // 点の出入りがあったときに締め直す。常に「部分木の全点を含む」が成り立つ
  struct BBox {
    std::array<Scalar, Dim> mn;
    std::array<Scalar, Dim> mx;

    void clear() {
      mn.fill(kUnboundedHi);
      mx.fill(kUnboundedLo);
    }

    // p を含むよう広げる。既に含んでいれば false
    bool expand(const PointT &p) {
      unsigned grew = 0;
      for (int d = 0; d < Dim; ++d) {
        grew |= unsigned(p[d] < mn[d]) | unsigned(p[d] > mx[d]);
        mn[d] = std::min(mn[d], p[d]);
        mx[d] = std::max(mx[d], p[d]);
      }
      return grew != 0;
    }

    Scalar distSq(const PointT &p) const {
      Scalar d_sq = 0;
      for (int d = 0; d < Dim; ++d) {
        Scalar e = std::max<Scalar>(0, std::max(mn[d] - p[d], p[d] - mx[d]));
        d_sq += e * e;
      }
      return d_sq;
    }
  };

  struct Cell {
    // 探索で読むものを先頭に寄せる
    Cell *child[2] = {nullptr, nullptr}; // [0]: x[axis] < threshold, [1]: それ以外
    Scalar threshold = 0;
    int axis = -1;
    int count = 0; // 部分木の要素数
    int depth = 0;
    LeafStore items; // 葉のみ
    BBox box; // use_bbox のときだけ維持する

    // 領域 [lo, hi)（半開区間）。根から継いだ境界は kUnboundedLo / kUnboundedHi
    std::array<Scalar, Dim> lo;
    std::array<Scalar, Dim> hi;
    Cell *parent = nullptr;
    // lo[d] / hi[d] を作った祖先 split。nullptr は無限遠（根から継いだ境界）
    std::array<Cell *, Dim> lo_owner;
    std::array<Cell *, Dim> hi_owner;

    bool isLeaf() const { return child[0] == nullptr; }

    bool contains(const PointT &p) const {
      // 分岐なしで全軸を見る（Dim=10 でもベクトル化される）
      unsigned in = 1;
      for (int d = 0; d < Dim; ++d)
        in &= unsigned(lo[d] <= p[d]) & unsigned(p[d] < hi[d]);
      return in != 0;
    }

    bool containsWithMargin(const PointT &p, Scalar margin) const {
      unsigned in = 1;
      for (int d = 0; d < Dim; ++d)
        in &= unsigned(lo[d] - margin <= p[d]) & unsigned(p[d] < hi[d] + margin);
      return in != 0;
    }

    template <typename Func> void visitCells(Func visitor, int dep) const {
      visitor(*this, dep);
      if (!isLeaf()) {
        child[0]->visitCells(visitor, dep + 1);
        child[1]->visitCells(visitor, dep + 1);
      }
    }
  };

  MovingBSPTree(const MovingBSPParams<Scalar> &params = MovingBSPParams<Scalar>())
      : params_(sanitize(params)) {
    root_ = allocCell();
    root_->box.clear();
    for (int d = 0; d < Dim; ++d) {
      root_->lo[d] = kUnboundedLo;
      root_->hi[d] = kUnboundedHi;
      root_->lo_owner[d] = nullptr;
      root_->hi_owner[d] = nullptr;
    }
  }

  // AdaptiveTree とコンストラクタの形を揃えるためのもの。
  // 根は無限領域なので full_extents は使わない。
  MovingBSPTree(const PointT & /*full_extents*/,
                const SpatialTreeParams<Scalar> &params)
      : MovingBSPTree(MovingBSPParams<Scalar>(params)) {}

  MovingBSPTree(const PointT & /*full_extents*/,
                const MovingBSPParams<Scalar> &params)
      : MovingBSPTree(params) {}

  MovingBSPTree(const MovingBSPTree &) = delete;
  MovingBSPTree &operator=(const MovingBSPTree &) = delete;

  void *add(T *element) {
    total_elements_++;
    Cell *leaf = insertFrom(root_, element, Traits::getPosition(element));
    return leaf;
  }

  void remove(T *element) {
    Cell *leaf = handleOf(element);
    if (!leaf) {
      // ハンドルを失っている要素は根から探して消す
      leaf = findLeafContaining(element);
      if (!leaf)
        return;
    }
    if (!eraseEntry(leaf, element))
      return;
    total_elements_--;
    Traits::setHandle(element, nullptr);
    Traits::setIndex(element, -1);
    Cell *scapegoat = nullptr;
    decrementUpTo(leaf, nullptr, scapegoat);
    if (scapegoat)
      rebuild(scapegoat);
  }

  void *updatePosition(T *element, const PointT &next_pos, Scalar margin = 0) {
    stats_.updates++;
    Cell *leaf = handleOf(element);
    if (!leaf) {
      Traits::setPosition(element, next_pos);
      return add(element);
    }

    // 1) 葉の box 内に留まるなら O(Dim) で終わり
    bool stays;
    if constexpr (HysteresisPolicy::enabled) {
      stays = leaf->containsWithMargin(next_pos, margin);
    } else {
      (void)margin;
      stays = leaf->contains(next_pos);
    }
    if (stays) {
      stats_.stayed_in_leaf++;
      Traits::setPosition(element, next_pos);
      updateCachedPosition(leaf, element, next_pos);
      if (params_.use_bbox && leaf->box.expand(next_pos))
        expandAncestors(leaf->parent, next_pos);
      return leaf;
    }

    // 2) 旧葉と新葉の LCA を求める
    Cell *lca = params_.use_owner_jump
                    ? lcaByOwnerJump(leaf, next_pos, stats_.jump_hops)
                    : lcaByClimb(leaf, next_pos, stats_.jump_hops);
#ifndef NDEBUG
    {
      std::uint64_t unused = 0;
      assert(lca == lcaByClimb(leaf, next_pos, unused));
    }
#endif
    if (lca->isLeaf()) {
      // 根が葉で、座標が NaN 等のためどの box 判定も成立しない。その葉に留める
      Traits::setPosition(element, next_pos);
      updateCachedPosition(leaf, element, next_pos);
      return leaf;
    }
    stats_.crossings++;
    stats_.levels_climbed += static_cast<std::uint64_t>(leaf->depth - lca->depth);

    // 3) 旧葉から外し、LCA の手前までの要素数を減らす
    if (!eraseEntry(leaf, element)) {
      // 葉に見つからない（ハンドル不整合）。根から入れ直す
      Traits::setPosition(element, next_pos);
      Traits::setHandle(element, nullptr);
      total_elements_--;
      return add(element);
    }
    Cell *scapegoat_old = nullptr;
    decrementUpTo(leaf, lca, scapegoat_old);

    // 4) LCA から新座標で降りる。LCA 自身の要素数は差し引き0
    Traits::setPosition(element, next_pos);
    lca->count--;
    Cell *scapegoat_new = nullptr;
    insertFrom(lca, element, next_pos, &scapegoat_new);
    expandAncestors(lca->parent, next_pos);

    // 5) 偏った部分木を作り直す。LCA が偏っていればそれで両側とも覆える
    if (isUnbalanced(lca)) {
      rebuild(lca);
    } else {
      if (scapegoat_old)
        rebuild(scapegoat_old);
      if (scapegoat_new)
        rebuild(scapegoat_new);
    }
    return handleOf(element);
  }

  /**
   * @brief N個の最近傍を検索（動的確保なしバージョン）。
   */
  template <std::size_t MaxN>
  int findNBest(const PointT &p, int n,
                std::array<SearchResult<T, Scalar, Dim>, MaxN> &results,
                Scalar search_margin = 0) const {
    if (n <= 0)
      return 0;
    int n_to_find = std::min<int>(n, MaxN);
    Candidate buffer[MaxN];
    SearchState state(p, n_to_find, buffer, search_margin);
#ifdef SPATIALTREE_BSP_SEARCH_STATS
    search_stats.queries++;
#endif
    runSearch(state);
    for (int i = 0; i < state.found; ++i)
      results[i] = {buffer[i].element, buffer[i].cell, buffer[i].dist_sq};
    return state.found;
  }

  std::vector<SearchResult<T, Scalar, Dim>>
  findNBest(const PointT &p, int n, Scalar search_margin = 0) const {
    if (n <= 0)
      return {};
    std::vector<Candidate> buffer(n);
    SearchState state(p, n, buffer.data(), search_margin);
#ifdef SPATIALTREE_BSP_SEARCH_STATS
    search_stats.queries++;
#endif
    runSearch(state);
    std::vector<SearchResult<T, Scalar, Dim>> res;
    res.reserve(state.found);
    for (int i = 0; i < state.found; ++i)
      res.push_back({buffer[i].element, buffer[i].cell, buffer[i].dist_sq});
    return res;
  }

#ifdef SPATIALTREE_BSP_SEARCH_STATS
  // 探索の中身の計測用（このマクロを定義したときだけ数える）
  struct SearchStats {
    std::uint64_t queries = 0, internal_visited = 0, leaves_scanned = 0,
                  points_scanned = 0;
  };
  mutable SearchStats search_stats;
#endif

  int getTotalNodes() const { return total_elements_; }
  const MovingBSPStats &getStats() const { return stats_; }
  void resetStats() { stats_ = MovingBSPStats(); }
  const MovingBSPParams<Scalar> &getParams() const { return params_; }

  template <typename Func> void visitCells(Func visitor) const {
    root_->visitCells(visitor, 0);
  }

  /**
   * @brief 木の不変条件を全て検査する（テスト用、O(要素数 + ノード数 × Dim)）。
   * @return 問題がなければ空文字列、あれば最初に見つかった問題の説明。
   */
  std::string checkInvariants() const {
    std::string err;
    int total = checkCell(root_, nullptr, err);
    if (err.empty() && total != total_elements_) {
      std::ostringstream os;
      os << "total_elements_=" << total_elements_ << " but tree holds " << total;
      err = os.str();
    }
    return err;
  }

private:
  struct Candidate {
    Scalar dist_sq;
    T *element;
    const void *cell;
  };

  // 上位 n 件を距離の昇順で持つ
  struct SearchState {
    const PointT &p;
    int n;
    Candidate *best;
    int found = 0;
    Scalar worst_dist_sq = std::numeric_limits<Scalar>::max();
    Scalar search_margin;
    Scalar prune_scale = 1; // 1 / (1+eps)^2
    Scalar prune_dist_sq = std::numeric_limits<Scalar>::max();

    SearchState(const PointT &p_, int n_, Candidate *buf, Scalar margin)
        : p(p_), n(n_), best(buf), search_margin(margin) {}

    // 下界 lb の部分木を見る必要があるか
    bool worthVisiting(Scalar lb) const { return found < n || lb <= prune_dist_sq; }

    void offer(Scalar d_sq, T *el, const void *cell) {
      int i;
      if (found < n) {
        i = found++;
      } else {
        i = n - 1; // 最悪の候補を上書き
      }
      while (i > 0 && best[i - 1].dist_sq > d_sq) {
        best[i] = best[i - 1];
        --i;
      }
      best[i] = {d_sq, el, cell};
      if (found == n) {
        worst_dist_sq = best[n - 1].dist_sq;
        prune_dist_sq = worst_dist_sq * prune_scale;
      }
    }
  };

  // rd: クエリから cell の box（各軸 search_margin だけ膨らませたもの）までの
  // 二乗距離。off[d]: その軸方向の成分。左右の子は1軸しか違わないので、
  // 遠い側の距離は O(1) で更新できる。要素は葉の box から各軸 margin 以内に
  // あるので、rd は厳密な下界になる。
  void runSearch(SearchState &s) const {
    if (params_.approx_eps > 0)
      s.prune_scale = static_cast<Scalar>(
          1.0 / ((1.0 + params_.approx_eps) * (1.0 + params_.approx_eps)));
    std::array<Scalar, Dim> off;
    off.fill(0);
    search(root_, s, 0, off);
  }

  static constexpr int kPartialChunk = 8;

  // 葉のブロックから上位2点だけを選んで登録する経路を使うか。
  // 低次元では1ブロックから複数の点が候補になるので得だが、次元が上がると
  // 候補は高々1点になり、上位2点を選ぶ手間が無駄になる（実測で切り替え）
#ifdef BSP_TOPTWO_EXPERIMENT
  static constexpr bool kTopTwoScan = BSP_TOPTWO_EXPERIMENT;
#else
  static constexpr bool kTopTwoScan = Dim <= 4;
#endif
  // 20次元では打ち切りの確認の方が高くつき（超球面で約20%遅い）、
  // 40次元以上で効き始める（実測、Apple M3）
  static constexpr int kPartialMinDim = 32;

  // 部分距離で打ち切りながら acc を埋める。最後まで計算したら true
  static bool accumulatePartial(const Block &blk, const SearchState &s,
                                Scalar (&acc)[kLanes]) {
    int d = 0;
    for (; d + kPartialChunk <= Dim; d += kPartialChunk) {
      for (int dd = d; dd < d + kPartialChunk; ++dd) {
        const Scalar q = s.p[dd];
        for (int l = 0; l < kLanes; ++l) {
          Scalar diff = blk.c[dd][l] - q;
          acc[l] += diff * diff;
        }
      }
      Scalar nearest = acc[0];
      for (int l = 1; l < kLanes; ++l)
        nearest = std::min(nearest, acc[l]);
      if (!(nearest < s.worst_dist_sq))
        return false;
    }
    for (; d < Dim; ++d) {
      const Scalar q = s.p[d];
      for (int l = 0; l < kLanes; ++l) {
        Scalar diff = blk.c[d][l] - q;
        acc[l] += diff * diff;
      }
    }
    return true;
  }

  void scanLeaf(const Cell *c, SearchState &s) const {
    const int m = c->items.size();
#ifdef SPATIALTREE_BSP_SEARCH_STATS
    search_stats.leaves_scanned++;
    search_stats.points_scanned += m;
#endif
    const Block *blocks = c->items.blocks.data();
    for (int base = 0; base < m; base += kLanes) {
      const Block &blk = blocks[base / kLanes];
      Scalar acc[kLanes] = {};
      if constexpr (Dim >= kPartialMinDim) {
        // 高次元では kPartialChunk 軸ごとに部分和の最小を確かめ、ブロック内の
        // どの点も今の k 番目より遠いと分かった時点で残りの軸を計算しない。
        // 部分和は真の距離の下界なので結果は変わらない。分散の大きい軸が
        // 先頭に来る座標系（PCA など）ほど早く打ち切れる
        if (!accumulatePartial(blk, s, acc))
          continue;
      } else {
        for (int d = 0; d < Dim; ++d) {
          const Scalar q = s.p[d];
          for (int l = 0; l < kLanes; ++l) {
            Scalar diff = blk.c[d][l] - q;
            acc[l] += diff * diff;
          }
        }
      }
      Scalar nearest = acc[0];
      for (int l = 1; l < kLanes; ++l)
        nearest = std::min(nearest, acc[l]);
      if (!(nearest < s.worst_dist_sq))
        continue; // ほとんどのブロックはここで終わる
      const int lanes = std::min(kLanes, m - base);
      if (kTopTwoScan && s.n <= 2) {
        // 上位 n (<= 2) 件を求めるときは、ブロック内で最も近い 2点しか
        // 候補になり得ない。その2点だけを登録する（同距離なら若いレーンが先で、
        // 1点ずつ登録する場合と結果は同じ）
        int i0 = -1, i1 = -1;
        Scalar v0 = std::numeric_limits<Scalar>::max(), v1 = v0;
        for (int l = 0; l < lanes; ++l) {
          const Scalar v = acc[l];
          if (v < v0) {
            v1 = v0;
            i1 = i0;
            v0 = v;
            i0 = l;
          } else if (v < v1) {
            v1 = v;
            i1 = l;
          }
        }
        if (i0 >= 0 && v0 < s.worst_dist_sq)
          s.offer(v0, blk.el[i0], c);
        if (s.n == 2 && i1 >= 0 && v1 < s.worst_dist_sq)
          s.offer(v1, blk.el[i1], c);
      } else {
        for (int l = 0; l < lanes; ++l)
          if (acc[l] < s.worst_dist_sq)
            s.offer(acc[l], blk.el[l], c);
      }
    }
  }

  void search(const Cell *c, SearchState &s, Scalar rd,
              std::array<Scalar, Dim> &off) const {
    if (c->isLeaf()) {
      scanLeaf(c, s);
      return;
    }
#ifdef SPATIALTREE_BSP_SEARCH_STATS
    search_stats.internal_visited++;
#endif
    const int a = c->axis;
    const Scalar diff = s.p[a] - c->threshold;
    const int near_side = diff >= 0 ? 1 : 0;
    const Cell *nc = c->child[near_side];
    const Cell *fc = c->child[1 - near_side];
    const Scalar old_off = off[a];
    const Scalar far_off = std::max<Scalar>(0, std::abs(diff) - s.search_margin);
    const Scalar far_rd = rd - old_off * old_off + far_off * far_off;

    if (params_.use_bbox) {
      // bbox までの距離も下界なので、rd と大きい方で枝刈りする。
      // 空の部分木は bbox が空なので飛ばす
      if (nc->count && s.worthVisiting(std::max(rd, nc->box.distSq(s.p))))
        search(nc, s, rd, off);
      // 遠い側は、まず O(1) の far_rd で判定し、通ったときだけ子のノードを
      // 読んで bbox 距離を計算する（枝刈りされる子のメモリに触れない）
      if (s.worthVisiting(far_rd) && fc->count &&
          s.worthVisiting(std::max(far_rd, fc->box.distSq(s.p)))) {
        off[a] = far_off;
        search(fc, s, far_rd, off);
        off[a] = old_off;
      }
      return;
    }

    search(nc, s, rd, off);
    if (s.worthVisiting(far_rd)) {
      off[a] = far_off;
      search(fc, s, far_rd, off);
      off[a] = old_off;
    }
  }

  static MovingBSPParams<Scalar> sanitize(MovingBSPParams<Scalar> p) {
    // 未指定（0 以下）の項目を次元から決める
    if (p.max_leaf_size <= 0)
      p.max_leaf_size = MovingBSPParams<Scalar>::tunedLeafSize(Dim);
    if (p.merge_threshold <= 0)
      p.merge_threshold = p.max_leaf_size / 3;
    if (p.min_rebuild_size <= 0)
      p.min_rebuild_size = 2 * p.max_leaf_size;
    if (p.use_bbox < 0)
      p.use_bbox = Dim > 10 ? 1 : 0;
    if (p.max_leaf_size < 2)
      p.max_leaf_size = 2;
    // 併合直後に再分割しないよう、併合閾値は葉の容量の半分までに抑える
    if (p.merge_threshold > p.max_leaf_size / 2)
      p.merge_threshold = p.max_leaf_size / 2;
    if (p.merge_threshold < 0)
      p.merge_threshold = 0;
    // 再構築対象は併合対象より必ず大きくする（両者が同じ経路で競合しないため）
    if (p.min_rebuild_size <= p.merge_threshold)
      p.min_rebuild_size = p.merge_threshold + 1;
    if (p.max_depth < 1)
      p.max_depth = 1;

    return p;
  }

  // ---------- ノードの確保 ----------
  // deque は末尾追加でアドレスが変わらないので、ポインタで繋げる
  Cell *allocCell() {
    if (!free_cells_.empty()) {
      Cell *c = free_cells_.back();
      free_cells_.pop_back();
      return c;
    }
    pool_.emplace_back();
    return &pool_.back();
  }

  void freeCell(Cell *c) {
    c->items.clear();
    c->child[0] = c->child[1] = nullptr;
    c->parent = nullptr;
    c->axis = -1;
    c->count = 0;
    free_cells_.push_back(c);
  }

  void freeSubtree(Cell *c) {
    if (!c->isLeaf()) {
      freeSubtree(c->child[0]);
      freeSubtree(c->child[1]);
    }
    freeCell(c);
  }

  static Cell *handleOf(const T *element) {
    return static_cast<Cell *>(const_cast<void *>(Traits::getHandle(element)));
  }

  // ---------- LCA の探索 ----------

  // 破った境界の owner のうち最も浅いものへ跳ぶ。そこの box にも
  // 入っていなければ繰り返す。box に入った最初のノードが LCA。
  Cell *lcaByOwnerJump(Cell *c, const PointT &p, std::uint64_t &hops) const {
    while (c != root_) {
      Cell *next = nullptr;
      bool violated = false;
      bool to_root = false;
      for (int d = 0; d < Dim; ++d) {
        if (!(c->lo[d] <= p[d])) {
          violated = true;
          Cell *o = c->lo_owner[d];
          if (!o)
            to_root = true; // 無限遠の境界を破った（NaN 等）
          else if (!next || o->depth < next->depth)
            next = o;
        }
        if (!(p[d] < c->hi[d])) {
          violated = true;
          Cell *o = c->hi_owner[d];
          if (!o)
            to_root = true;
          else if (!next || o->depth < next->depth)
            next = o;
        }
      }
      if (!violated)
        break;
      hops++;
      c = (to_root || !next) ? root_ : next;
    }
    return c;
  }

  // 比較用: 親を1段ずつ上り、box に入る最初の祖先を探す
  Cell *lcaByClimb(Cell *c, const PointT &p, std::uint64_t &hops) const {
    while (c != root_ && !c->contains(p)) {
      hops++;
      c = c->parent;
    }
    return c;
  }

  // ---------- 要素の出し入れ ----------

  // 葉の中での要素の位置。インデックスが壊れていれば線形に探す（安全策）
  static int indexInLeaf(const Cell *leaf, const T *element) {
    const LeafStore &es = leaf->items;
    int idx = Traits::getIndex(element);
    if (idx >= 0 && idx < es.size() && es.elem(idx) == element)
      return idx;
    for (int i = 0; i < es.size(); ++i)
      if (es.elem(i) == element)
        return i;
    return -1;
  }

  static void updateCachedPosition(Cell *leaf, T *element, const PointT &p) {
    int idx = indexInLeaf(leaf, element);
    if (idx >= 0)
      leaf->items.setPos(idx, p);
  }

  bool eraseEntry(Cell *leaf, T *element) {
    int idx = indexInLeaf(leaf, element);
    if (idx < 0)
      return false;
    leaf->items.swapPop(idx);
    if (idx < leaf->items.size())
      Traits::setIndex(leaf->items.elem(idx), idx);
    if (params_.use_bbox)
      refreshLeafBBox(leaf);
    return true;
  }

  // 点が出ていった葉の bbox を締め直す
  void refreshLeafBBox(Cell *leaf) {
    leaf->box.clear();
    for (int i = 0; i < leaf->items.size(); ++i)
      leaf->box.expand(leaf->items.pos(i));
  }

  // 内部ノードの bbox を子の和集合で締め直す
  static void refreshInternalBBox(Cell *c) {
    for (int d = 0; d < Dim; ++d) {
      c->box.mn[d] = std::min(c->child[0]->box.mn[d], c->child[1]->box.mn[d]);
      c->box.mx[d] = std::max(c->child[0]->box.mx[d], c->child[1]->box.mx[d]);
    }
  }

  // c から上へ、p を含むまで bbox を広げる。親の bbox は子の bbox を
  // 含むので、既に含んでいる祖先に当たればそこから上も含んでいる
  void expandAncestors(Cell *c, const PointT &p) {
    if (!params_.use_bbox)
      return;
    while (c && c->box.expand(p))
      c = c->parent;
  }

  void pushEntry(Cell *leaf, T *element, const PointT &p) {
    Traits::setIndex(element, leaf->items.size());
    Traits::setHandle(element, leaf);
    leaf->items.push(element, p);
    if (params_.use_bbox)
      leaf->box.expand(p);
  }

  // c から p の入る葉まで降りて挿入する。通過したノードの要素数を +1 する。
  // scapegoat が渡されれば、偏りが閾値を超えた最も浅いノードを返す。
  Cell *insertFrom(Cell *c, T *element, const PointT &p,
                   Cell **scapegoat = nullptr) {
    while (!c->isLeaf()) {
      c->count++;
      if (params_.use_bbox)
        c->box.expand(p);
      Cell *nx = c->child[p[c->axis] >= c->threshold ? 1 : 0];
      if (scapegoat && !*scapegoat && c->count >= params_.min_rebuild_size &&
          static_cast<double>(nx->count + 1) >
              params_.balance_alpha * static_cast<double>(c->count))
        *scapegoat = c;
      c = nx;
    }
    c->count++;
    pushEntry(c, element, p);
    if (c->items.size() > params_.max_leaf_size)
      split(c);
    return handleOf(element);
  }

  // 葉 leaf から stop の手前まで（stop == nullptr なら根まで）要素数を -1 する。
  // 併合できる最も浅いノードを併合し、偏りの閾値を超えた最も浅いノードを返す。
  void decrementUpTo(Cell *leaf, Cell *stop, Cell *&scapegoat) {
    Cell *collapse_at = nullptr;
    for (Cell *c = leaf; c && c != stop; c = c->parent) {
      c->count--;
      if (c->isLeaf())
        continue;
      if (params_.use_bbox)
        refreshInternalBBox(c);
      if (c->count <= params_.merge_threshold)
        collapse_at = c;
      else if (isUnbalanced(c))
        scapegoat = c;
    }
    // 部分木の要素数は上ほど大きいので、scapegoat は必ず collapse_at より上にある
    if (collapse_at)
      collapse(collapse_at);
  }

  bool isUnbalanced(const Cell *c) const {
    if (c->isLeaf() || c->count < params_.min_rebuild_size)
      return false;
    int heavier = std::max(c->child[0]->count, c->child[1]->count);
    return static_cast<double>(heavier) >
           params_.balance_alpha * static_cast<double>(c->count);
  }

  // ---------- 分割・併合・再構築 ----------

  // 葉を中央値付近の隙間で二分する。子が容量を超えれば再帰的に分割する。
  void split(Cell *leaf) {
    const int m = leaf->items.size();
    if (m <= params_.max_leaf_size || leaf->depth >= params_.max_depth)
      return;

    int axis;
    Scalar threshold;
    if (!chooseSplit(leaf, axis, threshold))
      return; // 全点が同一座標など、分けられない

    // leaf を内部ノードに切り替える前に要素を取り出す
    std::vector<Entry> old;
    old.reserve(m);
    gatherEntries(leaf, old);
    leaf->items.release();

    Cell *kids[2] = {allocCell(), allocCell()};
    for (int s = 0; s < 2; ++s) {
      Cell *k = kids[s];
      k->lo = leaf->lo;
      k->hi = leaf->hi;
      k->lo_owner = leaf->lo_owner;
      k->hi_owner = leaf->hi_owner;
      k->parent = leaf;
      k->child[0] = k->child[1] = nullptr;
      k->axis = -1;
      k->depth = leaf->depth + 1;
      k->count = 0;
      k->items.clear();
      k->box.clear();
      k->items.reserve(params_.max_leaf_size + 1);
    }
    kids[0]->hi[axis] = threshold;
    kids[0]->hi_owner[axis] = leaf;
    kids[1]->lo[axis] = threshold;
    kids[1]->lo_owner[axis] = leaf;

    leaf->axis = axis;
    leaf->threshold = threshold;
    leaf->child[0] = kids[0];
    leaf->child[1] = kids[1];

    for (const Entry &e : old) {
      Cell *k = kids[e.pos[axis] >= threshold ? 1 : 0];
      k->count++;
      pushEntry(k, e.element, e.pos);
    }
    stats_.splits++;

    for (Cell *k : kids)
      if (k->items.size() > params_.max_leaf_size)
        split(k);
  }

  // 広がりが最大の軸を選び、中央値の直前の隙間の中点で切る。
  // 点から境界までの距離が最大になるので、微小移動で跨ぎにくい。
  bool chooseSplit(const Cell *leaf, int &axis_out, Scalar &threshold_out) {
    const LeafStore &es = leaf->items;
    const int m = es.size();
    std::array<Scalar, Dim> mn, mx;
    mn.fill(std::numeric_limits<Scalar>::max());
    mx.fill(std::numeric_limits<Scalar>::lowest());
    for (int i = 0; i < m; ++i) {
      for (int d = 0; d < Dim; ++d) {
        mn[d] = std::min(mn[d], es.coord(i, d));
        mx[d] = std::max(mx[d], es.coord(i, d));
      }
    }
    std::array<int, Dim> order;
    for (int d = 0; d < Dim; ++d)
      order[d] = d;
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return mx[a] - mn[a] > mx[b] - mn[b]; });

    scratch_.resize(m);
    for (int oi = 0; oi < Dim; ++oi) {
      const int a = order[oi];
      if (!(mx[a] > mn[a]))
        return false; // 残りの軸は全て幅0（全点同一）
      for (int i = 0; i < m; ++i)
        scratch_[i] = es.coord(i, a);

      // 閾値は box の内側に限る。ヒステリシス有効時は点が box から
      // はみ出していることがあり、その隙間で切ると子の box が潰れる
      auto accept = [&](Scalar lower, Scalar upper) {
        if (!(lower < upper))
          return false;
        Scalar t = midpoint(lower, upper);
        if (!(leaf->lo[a] < t && t < leaf->hi[a]))
          return false;
        threshold_out = t;
        axis_out = a;
        return true;
      };
      if (params_.split_rule == 1 && accept(mn[a], mx[a]))
        return true;
      const int k = m / 2;
      std::nth_element(scratch_.begin(), scratch_.begin() + k, scratch_.end());
      if (accept(*std::max_element(scratch_.begin(), scratch_.begin() + k),
                 scratch_[k]))
        return true;
      // 中央値付近に重複がある（か box 外）。全体を並べて中央に最も近い隙間を探す
      std::sort(scratch_.begin(), scratch_.end());
      for (int off = 0; off < m; ++off) {
        for (int i : {k - off, k + off + 1}) {
          if (i >= 1 && i < m && accept(scratch_[i - 1], scratch_[i]))
            return true;
        }
      }
    }
    return false;
  }

  // lo < t <= hi を保証する中点（丸めで lo に落ちたら hi を使う）
  static Scalar midpoint(Scalar lo, Scalar hi) {
    Scalar t = lo + (hi - lo) / 2;
    if (!(t > lo))
      t = hi;
    return t;
  }

  void gatherEntries(Cell *c, std::vector<Entry> &out) {
    if (c->isLeaf()) {
      for (int i = 0; i < c->items.size(); ++i)
        out.push_back(Entry{c->items.pos(i), c->items.elem(i)});
      return;
    }
    gatherEntries(c->child[0], out);
    gatherEntries(c->child[1], out);
  }

  // 部分木を1枚の葉に戻す。c の box と owner は変わらないので、
  // c より上の構造にも、他の部分木の owner にも影響しない。
  void collapse(Cell *c) {
    if (c->isLeaf())
      return;
    flatten(c);
    stats_.collapses++;
  }

  // 部分木を中央値 split で作り直す。c の box は変えない。
  void rebuild(Cell *c) {
    if (c->isLeaf())
      return;
    stats_.rebuilds++;
    stats_.rebuilt_elements += static_cast<std::uint64_t>(c->count);
    flatten(c);
    split(c);
  }

  void flatten(Cell *c) {
    std::vector<Entry> all;
    all.reserve(c->count);
    gatherEntries(c, all);
    freeSubtree(c->child[0]);
    freeSubtree(c->child[1]);
    c->child[0] = c->child[1] = nullptr;
    c->axis = -1;
    c->items.clear();
    c->box.clear();
    for (const Entry &e : all)
      pushEntry(c, e.element, e.pos);
    c->count = static_cast<int>(all.size());
  }

  Cell *findLeafContaining(const T *element) {
    Cell *found = nullptr;
    visitLeaves(root_, [&](Cell *leaf) {
      if (found)
        return;
      for (int i = 0; i < leaf->items.size(); ++i)
        if (leaf->items.elem(i) == element)
          found = leaf;
    });
    return found;
  }

  template <typename Func> void visitLeaves(Cell *c, Func f) {
    if (c->isLeaf()) {
      f(c);
      return;
    }
    visitLeaves(c->child[0], f);
    visitLeaves(c->child[1], f);
  }

  // ---------- 検証 ----------

  static bool isAncestor(const Cell *anc, const Cell *c) {
    for (const Cell *x = c->parent; x; x = x->parent)
      if (x == anc)
        return true;
    return false;
  }

  int checkCell(const Cell *c, const Cell *parent, std::string &err) const {
    auto fail = [&](const std::string &msg) {
      if (err.empty()) {
        std::ostringstream os;
        os << "cell depth=" << c->depth << ": " << msg;
        err = os.str();
      }
    };
    if (c->parent != parent)
      fail("parent pointer mismatch");
    if (parent && c->depth != parent->depth + 1)
      fail("depth mismatch");

    for (int d = 0; d < Dim; ++d) {
      const Cell *lo_o = c->lo_owner[d];
      const Cell *hi_o = c->hi_owner[d];
      if (!lo_o) {
        if (c->lo[d] != kUnboundedLo)
          fail("finite lo without owner");
      } else if (!isAncestor(lo_o, c) || lo_o->axis != d ||
                 lo_o->threshold != c->lo[d]) {
        fail("lo owner is not the ancestor split that made this bound");
      }
      if (!hi_o) {
        if (c->hi[d] != kUnboundedHi)
          fail("finite hi without owner");
      } else if (!isAncestor(hi_o, c) || hi_o->axis != d ||
                 hi_o->threshold != c->hi[d]) {
        fail("hi owner is not the ancestor split that made this bound");
      }
    }

    if (c->isLeaf()) {
      if (c->child[1])
        fail("half-leaf");
      for (int i = 0; i < c->items.size(); ++i) {
        const Entry e{c->items.pos(i), c->items.elem(i)};
        if (handleOf(e.element) != c)
          fail("element handle does not point to its leaf");
        if (Traits::getIndex(e.element) != i)
          fail("element index_in_cell mismatch");
        const PointT &actual = Traits::getPosition(e.element);
        for (int d = 0; d < Dim; ++d)
          if (actual[d] != e.pos[d])
            fail("cached position differs from element position");
        if constexpr (!HysteresisPolicy::enabled) {
          if (!c->contains(e.pos))
            fail("element lies outside its leaf box");
        }
        if (params_.use_bbox && c->box.distSq(e.pos) != 0)
          fail("element lies outside its leaf bbox");
      }
      if (c->count != c->items.size())
        fail("leaf count mismatch");
      if (static_cast<int>(c->items.blocks.size()) !=
          (c->items.size() + kLanes - 1) / kLanes)
        fail("block count mismatch");
      return c->items.size();
    }

    if (!c->items.empty())
      fail("internal cell holds entries");
    const int a = c->axis;
    if (a < 0 || a >= Dim)
      fail("bad axis");
    else {
      if (!(c->lo[a] < c->threshold && c->threshold < c->hi[a]))
        fail("threshold outside box");
      for (int s = 0; s < 2; ++s) {
        const Cell *k = c->child[s];
        for (int d = 0; d < Dim; ++d) {
          Scalar want_lo = (d == a && s == 1) ? c->threshold : c->lo[d];
          Scalar want_hi = (d == a && s == 0) ? c->threshold : c->hi[d];
          if (k->lo[d] != want_lo || k->hi[d] != want_hi)
            fail("child box is not parent box split at threshold");
        }
      }
    }
    if (params_.use_bbox) {
      for (const Cell *k : c->child) {
        if (k->count == 0)
          continue;
        for (int d = 0; d < Dim; ++d)
          if (k->box.mn[d] < c->box.mn[d] || k->box.mx[d] > c->box.mx[d])
            fail("child bbox is not inside parent bbox");
      }
    }
    int n0 = checkCell(c->child[0], c, err);
    int n1 = checkCell(c->child[1], c, err);
    if (c->count != n0 + n1)
      fail("internal count mismatch");
    return n0 + n1;
  }

  MovingBSPParams<Scalar> params_;
  std::deque<Cell> pool_;
  std::vector<Cell *> free_cells_;
  Cell *root_ = nullptr;
  int total_elements_ = 0;
  std::vector<Scalar> scratch_;
  MovingBSPStats stats_;
};

} // namespace SpatialTree

#endif // SPATIAL_TREE_MOVING_BSP_TREE_HPP
