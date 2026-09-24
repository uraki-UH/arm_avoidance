// bsp3d: 動く点群向け空間索引 + Growing Neural Gas（単一ヘッダ版）
//
// 旧SpatialTree実装を起源とする単一ヘッダ。
// 生成元: SpatialTree/include/SpatialTree/{Config.hpp, Point.hpp, Policy.hpp, Traits.hpp, SpatialTree.hpp, MovingBSPTree.hpp, GNG/Node.hpp, GNG/GNGPolicy.hpp, GNG/GNG.hpp}
// 生成日: 2026-09-23
//
// 2026-09-24以降の保守正本は本ヘッダ。外部SpatialTreeへの生成依存なし。
// 既存のSpatialTree名前空間と互換型の保持。
#ifndef BSP3D_SINGLE_HEADER_HPP
#define BSP3D_SINGLE_HEADER_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <stdexcept>
#include <type_traits>
#include <vector>

// ===== Config.hpp ==================================================
namespace SpatialTree {
/**
 * @brief 座標や計算に使用するデフォルトのスカラ型。
 */
using DefaultScalar = float;

} // namespace SpatialTree

// ===== Point.hpp ===================================================
namespace SpatialTree {

/**
 * @brief Eigen 非依存の N次元ポイント型（std::array ベース）。
 * GNG で必要な基本的なベクトル演算を記述
 */
template <typename Scalar, int Dim>
struct Point {
    std::array<Scalar, Dim> data;

    Point() { data.fill(0); }
    
    // 値指定コンストラクタ（可変長引数）
    template<typename... Args>
    Point(Args... args) : data{{static_cast<Scalar>(args)...}} {
        static_assert(sizeof...(Args) == Dim, "Incorrect number of arguments for Point constructor");
    }

    static Point Zero() { return Point(); }

    Scalar& operator[](int i) { return data[i]; }
    const Scalar& operator[](int i) const { return data[i]; }

    // 軸別アクセス (x, y, z) - テンプレートメタプログラミングで Dim に応じて提供
    Scalar& x() { return data[0]; }
    Scalar x() const { return data[0]; }
    
    Scalar& y() { 
        static_assert(Dim >= 2, "Y axis only available for Dim >= 2");
        return data[1]; 
    }
    Scalar y() const { 
        static_assert(Dim >= 2, "Y axis only available for Dim >= 2");
        return data[1]; 
    }

    Scalar& z() { 
        static_assert(Dim >= 3, "Z axis only available for Dim >= 3");
        return data[2]; 
    }
    Scalar z() const { 
        static_assert(Dim >= 3, "Z axis only available for Dim >= 3");
        return data[2]; 
    }

    // 演算子オーバーロード
    Point& operator+=(const Point& other) {
        for (int i = 0; i < Dim; ++i) data[i] += other.data[i];
        return *this;
    }
    Point operator+(const Point& other) const {
        Point res;
        for (int i = 0; i < Dim; ++i) res.data[i] = data[i] + other.data[i];
        return res;
    }

    Point& operator-=(const Point& other) {
        for (int i = 0; i < Dim; ++i) data[i] -= other.data[i];
        return *this;
    }
    Point operator-(const Point& other) const {
        Point res;
        for (int i = 0; i < Dim; ++i) res.data[i] = data[i] - other.data[i];
        return res;
    }

    Point operator*(Scalar s) const {
        Point res;
        for (int i = 0; i < Dim; ++i) res.data[i] = data[i] * s;
        return res;
    }
    
    Point operator/(Scalar s) const {
        Point res;
        for (int i = 0; i < Dim; ++i) res.data[i] = data[i] / s;
        return res;
    }

    // ベクトル計算
    Scalar squaredNorm() const {
        Scalar res = 0;
        for (int i = 0; i < Dim; ++i) res += data[i] * data[i];
        return res;
    }
    
    Scalar norm() const {
        return std::sqrt(squaredNorm());
    }

    Scalar dot(const Point& other) const {
        Scalar res = 0;
        for (int i = 0; i < Dim; ++i) res += data[i] * other.data[i];
        return res;
    }

    bool isZero(Scalar epsilon = static_cast<Scalar>(1e-9)) const {
        for (int i = 0; i < Dim; ++i) {
            if (std::abs(data[i]) > epsilon) return false;
        }
        return true;
    }
};

// Scalar * Point
template <typename Scalar, int Dim>
Point<Scalar, Dim> operator*(Scalar s, const Point<Scalar, Dim>& p) {
    return p * s;
}

} // namespace SpatialTree

// ===== Policy.hpp ==================================================
namespace SpatialTree {

/**
 * @brief セルマージンなしポリシー(trueの場合)
 * 静的なデータセットの場合に最適
 */
struct NoHysteresis {
  static constexpr bool enabled = false;
  static constexpr bool use_upward_traversal = false;
};

/**
 * @brief 適応的マージンポリシー（マージンベースの更新抑制）。
 * GNG の学習など、ノードが頻繁に微小移動する場合など、セル境界で振動的な更新が発生しそうな場合に有効
 */
struct AdaptiveHysteresis {
  static constexpr bool enabled = true;
  static constexpr bool use_upward_traversal = true; 
};

} // namespace SpatialTree

// ===== Traits.hpp ==================================================
namespace SpatialTree {

template <typename T, typename Scalar, int Dim> struct SpatialTraits {
  static const Point<Scalar, Dim> &getPosition(const T *obj) {
    return obj->position;
  }

  static void setPosition(T *obj, const Point<Scalar, Dim> &pos) {
    obj->position = pos;
  }

 
  //自分自身が所属しているツリー内のセルのアドレスを取得
  // これにより、削除や更新が O(1) 程度で実行
  static const void *getHandle(const T *obj) { return obj->spatial_handle; }

  static void setHandle(T *obj, const void *handle) {
    obj->spatial_handle = const_cast<void *>(handle);
  }

  static int getIndex(const T *obj) { return obj->index_in_cell; }
  static void setIndex(T *obj, int index) { obj->index_in_cell = index; }
};

} // namespace SpatialTree

// ===== SpatialTree.hpp =============================================
namespace SpatialTree {
template <typename T, typename Scalar = DefaultScalar, int Dim = 2>
struct SearchResult {
  T *element;
  const void *cell_handle;
  Scalar distance_sq;
};

/**
 * @brief N次元バウンディングボックス。
 */
template <typename Scalar, int Dim> struct BoundingBox {
  Point<Scalar, Dim> center;       // 中心座標
  Point<Scalar, Dim> half_extents; // 各軸方向の長さの半分

  bool contains(const Point<Scalar, Dim> &p) const {
    for (int i = 0; i < Dim; ++i) {
      if (p[i] < center[i] - half_extents[i] ||
          p[i] > center[i] + half_extents[i])
        return false;
    }
    return true;
  }

  bool contains_with_margin(const Point<Scalar, Dim> &p, Scalar margin) const {
    for (int i = 0; i < Dim; ++i) {
      if (p[i] < center[i] - half_extents[i] - margin ||
          p[i] > center[i] + half_extents[i] + margin)
        return false;
    }
    return true;
  }

  Scalar squared_distance_to(const Point<Scalar, Dim> &p) const {
    Scalar dist_sq = 0;
    for (int i = 0; i < Dim; ++i) {
      Scalar d =
          std::max<Scalar>(0, std::abs(p[i] - center[i]) - half_extents[i]);
      dist_sq += d * d;
    }
    return dist_sq;
  }
};

/**
 * @brief 空間計算の動作パラメータ。
 */
template <typename Scalar> struct SpatialTreeParams {
  int max_nodes_per_cell = 20;
  int min_nodes_for_merge = 4;
  Scalar min_cell_size = 0.01;
  int max_depth = 32; // 安全上の上限（通常は min_cell_size が優先される）
};


} // namespace SpatialTree

// ===== MovingBSPTree.hpp ===========================================
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

  // 閉区間AABB内の要素列挙。葉の座標キャッシュと空部分木の省略。
  // ヒステリシス併用時のsearch_marginは、位置更新時の許容幅。
  template <typename visitor_type>
  void query_aabb(const PointT &min_point, const PointT &max_point,
                  visitor_type visitor, Scalar search_margin = 0) const {
    if (!std::isfinite(search_margin) || search_margin < 0)
      throw std::invalid_argument("範囲検索には有限で非負の余白が必要です");
    for (int axis = 0; axis < Dim; ++axis)
      if (!std::isfinite(min_point[axis]) || !std::isfinite(max_point[axis]) ||
          min_point[axis] > max_point[axis])
        throw std::invalid_argument("範囲検索には有限で整列済みの上下限が必要です");
    const auto visit = [&](const auto &self, const Cell *cell) -> void {
      if (cell->count == 0) return;
      if (params_.use_bbox) {
        for (int axis = 0; axis < Dim; ++axis)
          if (cell->box.mx[axis] < min_point[axis] ||
              cell->box.mn[axis] > max_point[axis]) return;
      }
      if (!cell->isLeaf()) {
        // 分割軸だけの枝刈り。探索不要な子のデータ参照なし。
        if (min_point[cell->axis] <= cell->threshold + search_margin)
          self(self, cell->child[0]);
        if (max_point[cell->axis] >= cell->threshold - search_margin)
          self(self, cell->child[1]);
        return;
      }
      for (int idx = 0; idx < cell->items.size(); ++idx) {
        bool is_inside = true;
        for (int axis = 0; axis < Dim; ++axis) {
          const Scalar value = cell->items.coord(idx, axis);
          is_inside = is_inside && value >= min_point[axis] && value <= max_point[axis];
        }
        if (is_inside) visitor(cell->items.elem(idx));
      }
    };
    visit(visit, root_);
  }

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

// ===== GNG/Node.hpp ================================================
namespace SpatialTree {

/**
 * @brief 隣接ノード -> エッジの年齢 の表。
 *
 * GNG の次数は高々数十なので、ハッシュ表ではなく連続配列に持つ。
 * 辺ごとのメモリ確保がなくなり、隣接をたどるときのキャッシュミスが減る。
 * std::unordered_map のうち GNG が使う操作（operator[], find, erase,
 * 範囲 for の ->first / ->second）と同じ書き方で使える。
 *
 * 注意: erase(iterator) は末尾の要素をその位置へ移して詰める。返り値の
 * イテレータは「まだ見ていない要素」を指すので、
 *   for (it = begin(); it != end();) { if (...) it = erase(it); else ++it; }
 * の形の走査はそのまま正しく動く。走査順は挿入順ではない。
 */
template <typename Key> class NeighborList {
public:
  struct value_type {
    Key first;
    int second;
  };
  using iterator = value_type *;
  using const_iterator = const value_type *;

  iterator begin() { return data_.data(); }
  iterator end() { return data_.data() + data_.size(); }
  const_iterator begin() const { return data_.data(); }
  const_iterator end() const { return data_.data() + data_.size(); }

  std::size_t size() const { return data_.size(); }
  bool empty() const { return data_.empty(); }
  void clear() { data_.clear(); }

  iterator find(Key k) {
    for (auto &e : data_)
      if (e.first == k)
        return &e;
    return end();
  }
  const_iterator find(Key k) const {
    for (const auto &e : data_)
      if (e.first == k)
        return &e;
    return end();
  }
  std::size_t count(Key k) const { return find(k) != end() ? 1 : 0; }

  // 無ければ年齢 0 で追加する
  int &operator[](Key k) {
    iterator it = find(k);
    if (it != end())
      return it->second;
    data_.push_back({k, 0});
    return data_.back().second;
  }

  std::size_t erase(Key k) {
    iterator it = find(k);
    if (it == end())
      return 0;
    erase(it);
    return 1;
  }

  iterator erase(iterator it) {
    std::size_t i = static_cast<std::size_t>(it - begin());
    data_[i] = data_.back();
    data_.pop_back();
    return begin() + i;
  }

private:
  std::vector<value_type> data_;
};

/**
 * @brief GNG のノード構造体（N次元対応）。
 */
template <typename Scalar, int Dim = 2> struct Node {
  int id = -1;
  Point<Scalar, Dim> position;

  // GNG 固有のパラメータ
  Scalar error = 0;    // 累積誤差
  bool active = false; // ノードが活性化（使用中）かどうか

  NeighborList<Node *> neighbors; // 隣接ノード -> エッジの年齢

  // 空間インデックス用ハンドル
  void *spatial_handle = nullptr;
  int index_in_cell = -1;
  int index_in_active_list = -1;
  int error_heap_index = -1;
  int utility_heap_index = -1;

  Node() = default;
  Node(int id, const Point<Scalar, Dim> &pos)
      : id(id), position(pos), active(true) {}
};

} // namespace SpatialTree

// ===== GNG/GNGPolicy.hpp ===========================================
namespace SpatialTree {

/**
 * @brief GNGの更新量に応じた適応的マージン +
 * 近傍ノード更新の際のセル更新判定のスキップ (trueの場合)
 */
struct LazyNeighborUpdate : public AdaptiveHysteresis {
  static constexpr bool lazy_neighbors = true;
};

/**
 * @brief マージンなし（第一勝者ノードは常に厳密にセル更新） +
 * 近傍ノード更新の際のセル更新判定のスキップ(trueの場合)
 */
struct NoHysteresisLazyNeighbor : public NoHysteresis {
  static constexpr bool lazy_neighbors = true;
};

} // namespace SpatialTree

// ===== GNG/GNG.hpp =================================================
namespace SpatialTree {

// Treeの適用方法のポリシーを定義
template <typename T> struct has_lazy_neighbors_trait {
private:
  template <typename U>
  static auto test(int) -> std::integral_constant<bool, U::lazy_neighbors>;
  template <typename U> static auto test(...) -> std::false_type;

public:
  static constexpr bool value = decltype(test<T>(0))::value;
};

template <typename T> constexpr bool has_lazy_neighbors() {
  return has_lazy_neighbors_trait<T>::value;
}

/**
 * @brief GNG の学習パラメータ。
 */
template <typename Scalar> struct GNGParams {
  int lambda = 100;
  Scalar alpha = 0.5;
  Scalar beta = 0.995;
  Scalar eps_w = 0.05;
  Scalar eps_n = 0.0006;
  int max_age = 50;
  int max_nodes = 1000;
  Scalar n_best_candidates = 2; // 実装の都合上64以下にすること
  // いまのところ4が早そう

  // マージン関連
  Scalar lpf_alpha = 0.1;
  Scalar hysteresis_margin_factor = 0.15;
};

/**
 * @brief N次元対応 Growing Neural Gas クラス。
 */
template <typename Scalar = DefaultScalar, int Dim = 2,
          typename HysteresisPolicy = AdaptiveHysteresis,
          bool UseMinCellSize = true,
          template <typename, typename, int, typename, typename>
          class TreeTemplate = MovingBSPTree>
class GrowingNeuralGas {
public:
  using PointT = Point<Scalar, Dim>;
  using NodeT = Node<Scalar, Dim>;
  using TraitsT = SpatialTraits<NodeT, Scalar, Dim>;
  using TreeT = TreeTemplate<NodeT, Scalar, Dim, TraitsT, HysteresisPolicy>;

  // 木の型に対応するパラメータ型
  template <typename TreeParams>
  GrowingNeuralGas(const PointT &full_extents,
                   const GNGParams<Scalar> &g_params,
                   const TreeParams &s_params)
      : g_params_(g_params), tree_(full_extents, s_params) {

    node_pool_.reserve(g_params.max_nodes);
    for (int i = 0; i < g_params.max_nodes; ++i) {
      node_pool_.push_back(std::make_unique<NodeT>());
      node_pool_.back()->id = i;
      free_node_indices_.push_back(i);
    }
  }

  void train_step(const PointT &sample) {
    std::array<SearchResult<NodeT, Scalar, Dim>, 64>
        nearest_buffer; // Stack buffer for N-best search
    Scalar margin = 0;
    if constexpr (HysteresisPolicy::enabled) {
      margin = g_params_.hysteresis_margin_factor * global_filtered_error_;
    }

    int n_to_find = static_cast<int>(g_params_.n_best_candidates);
    int found_count = tree_.findNBest(sample, n_to_find, nearest_buffer, margin);

    if (found_count < 2) {
      if (active_nodes_ < g_params_.max_nodes)
        addNode(sample);
      return;
    }

    NodeT *s1 = nearest_buffer[0].element;
    NodeT *s2 = nearest_buffer[1].element;

    Scalar d1_sq = nearest_buffer[0].distance_sq;
    Scalar d1 = std::sqrt(d1_sq);
    s1->error += d1 / global_error_multiplier_;
    upHeapError(s1->error_heap_index);

    // セルマージン有効時のみグローバル累積誤差を更新
    if constexpr (HysteresisPolicy::enabled) {
      global_filtered_error_ =
          (1.0 - g_params_.lpf_alpha) * global_filtered_error_ +
          g_params_.lpf_alpha * d1;
    }

    const void *s1_cell = TraitsT::getHandle(s1);

    PointT next_s1 = s1->position + g_params_.eps_w * (sample - s1->position);
    margin = g_params_.hysteresis_margin_factor * global_filtered_error_;
    tree_.updatePosition(s1, next_s1, margin);

    s1->neighbors[s2] = 0;
    s2->neighbors[s1] = 0;

    std::vector<NodeT *> to_remove;
    for (auto it = s1->neighbors.begin(); it != s1->neighbors.end();) {
      NodeT *nbr = it->first;
      it->second++;

      if (it->second > g_params_.max_age) {
        nbr->neighbors.erase(s1);
        it = s1->neighbors.erase(it);
        if (nbr->neighbors.empty())
          to_remove.push_back(nbr);
      } else {
        PointT next_nbr =
            nbr->position + g_params_.eps_n * (sample - nbr->position);

        if constexpr (has_lazy_neighbors<HysteresisPolicy>()) {
          TraitsT::setPosition(nbr, next_nbr);
        } else {
          tree_.updatePosition(nbr, next_nbr, margin);
        }
        ++it;
      }
    }

    for (auto *n : to_remove)
      removeNode(n);
    if (s1->neighbors.empty())
      removeNode(s1);

    if (++step_count_ % g_params_.lambda == 0) {
      add_distributed_node();
    }
    decay_errors();
  }

  const std::vector<NodeT *> &getActiveNodes() const {
    return active_node_ptrs_;
  }

  // 学習の途中でパラメータを差し替える（収束後に学習率を下げる等）
  const GNGParams<Scalar> &getParams() const { return g_params_; }
  void setParams(const GNGParams<Scalar> &p) { g_params_ = p; }
  int getNodesCount() const { return active_nodes_; }
  const TreeT &getTree() const { return tree_; }

  NodeT *addNode(const PointT &pos) {
    if (!free_node_indices_.empty()) {
      int idx = free_node_indices_.back();
      free_node_indices_.pop_back();
      auto &n = node_pool_[idx];
      n->active = true;
      n->position = pos;
      n->error = 0;
      n->neighbors.clear();
      tree_.add(n.get());
      n->index_in_active_list = static_cast<int>(active_node_ptrs_.size());
      active_node_ptrs_.push_back(n.get());
      active_nodes_++;

      n->error_heap_index = static_cast<int>(error_heap_.size());
      error_heap_.push_back(n.get());
      upHeapError(n->error_heap_index);
      return n.get();
    }
    return nullptr;
  }

  void removeNode(NodeT *node) {
    if (!node || !node->active)
      return;
    tree_.remove(node);
    node->active = false;

    int idx = node->index_in_active_list;
    if (idx >= 0 && idx < static_cast<int>(active_node_ptrs_.size())) {
      if (idx != static_cast<int>(active_node_ptrs_.size()) - 1) {
        active_node_ptrs_[idx] = active_node_ptrs_.back();
        active_node_ptrs_[idx]->index_in_active_list = idx;
      }
      active_node_ptrs_.pop_back();
    }
    node->index_in_active_list = -1;

    int h_idx = node->error_heap_index;
    if (h_idx >= 0 && h_idx < static_cast<int>(error_heap_.size())) {
      if (h_idx != static_cast<int>(error_heap_.size()) - 1) {
        error_heap_[h_idx] = error_heap_.back();
        error_heap_[h_idx]->error_heap_index = h_idx;
        upHeapError(h_idx);
        downHeapError(h_idx);
      }
      error_heap_.pop_back();
    }
    node->error_heap_index = -1;

    free_node_indices_.push_back(node->id);
    active_nodes_--;
  }

  void add_distributed_node() {
    if (error_heap_.empty())
      return;
    NodeT *q = error_heap_.front();
    if (!q)
      return;

    NodeT *f = nullptr;
    Scalar max_f_err = -1;
    for (auto &pair : q->neighbors) {
      if (pair.first->error > max_f_err) {
        max_f_err = pair.first->error;
        f = pair.first;
      }
    }
    if (!f)
      return;

    PointT mid = static_cast<Scalar>(0.5) * (q->position + f->position);
    NodeT *r = addNode(mid);
    if (r) {
      q->neighbors.erase(f);
      f->neighbors.erase(q);
      q->neighbors[r] = 0;
      r->neighbors[q] = 0;
      f->neighbors[r] = 0;
      r->neighbors[f] = 0;

      q->error *= g_params_.alpha;
      f->error *= g_params_.alpha;
      r->error = q->error;

      downHeapError(q->error_heap_index);
      downHeapError(f->error_heap_index);
      upHeapError(r->error_heap_index);
    }
  }

  void decay_errors() {
    global_error_multiplier_ *= g_params_.beta;

    if (global_error_multiplier_ < 1e-15) {
      for (auto *n : active_node_ptrs_) {
        n->error *= global_error_multiplier_;
      }
      global_error_multiplier_ = 1.0;
    }
  }

  void upHeapError(int idx) {
    while (idx > 0) {
      int p = (idx - 1) / 2;
      if (error_heap_[idx]->error <= error_heap_[p]->error)
        break;
      std::swap(error_heap_[idx], error_heap_[p]);
      error_heap_[idx]->error_heap_index = idx;
      error_heap_[p]->error_heap_index = p;
      idx = p;
    }
  }

  void downHeapError(int idx) {
    while (true) {
      int l = idx * 2 + 1;
      int r = idx * 2 + 2;
      int largest = idx;
      if (l < static_cast<int>(error_heap_.size()) &&
          error_heap_[l]->error > error_heap_[largest]->error)
        largest = l;
      if (r < static_cast<int>(error_heap_.size()) &&
          error_heap_[r]->error > error_heap_[largest]->error)
        largest = r;
      if (largest == idx)
        break;
      std::swap(error_heap_[idx], error_heap_[largest]);
      error_heap_[idx]->error_heap_index = idx;
      error_heap_[largest]->error_heap_index = largest;
      idx = largest;
    }
  }

private:
  GNGParams<Scalar> g_params_;
  TreeT tree_;
  std::vector<std::unique_ptr<NodeT>> node_pool_;
  std::vector<NodeT *> active_node_ptrs_;
  int active_nodes_ = 0;
  int step_count_ = 0;
  std::vector<int> free_node_indices_;
  Scalar global_error_multiplier_ = 1.0; // 誤差の減衰を効率的に行うための乗数。
  Scalar global_filtered_error_ =
      0; // セルマージン用の第一勝者に対するグローバル累積誤差(使わないなら処理を無効化した方がよい)
  std::vector<NodeT *> error_heap_;
};

} // namespace SpatialTree


// ===== 3次元向けの型エイリアスと既定値 =======================
namespace bsp3d {

using Scalar = float;
inline constexpr int kDim = 3;
using Point3 = SpatialTree::Point<Scalar, kDim>;

/// 索引に入れる要素が満たすべき形（position / spatial_handle / index_in_cell）
/// 例:
///   struct MyNode { bsp3d::Point3 position; void* spatial_handle = nullptr; int index_in_cell = -1; };
// 既存float既定値と、静的索引用double指定の両立。
template <typename scalar>
using point3 = SpatialTree::Point<scalar, kDim>;

template <typename scalar>
using index_params = SpatialTree::MovingBSPParams<scalar>;

template <class T, typename scalar = Scalar>
using Index = SpatialTree::MovingBSPTree<T, scalar, kDim>;

template <class T>
using SearchResult = SpatialTree::SearchResult<T, Scalar, kDim>;

using Params = SpatialTree::MovingBSPParams<Scalar>;

/// GNG（索引に Index を使う）。Node は GNGNode を使う
using GNGNode = SpatialTree::Node<Scalar, kDim>;
using GNG = SpatialTree::GrowingNeuralGas<Scalar, kDim, SpatialTree::NoHysteresis, true,
                                          SpatialTree::MovingBSPTree>;
using GNGParams = SpatialTree::GNGParams<Scalar>;

} // namespace bsp3d

#endif // BSP3D_SINGLE_HEADER_HPP
