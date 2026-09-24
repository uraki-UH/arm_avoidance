# bsp3d

動く点群（GNG のノードなど）向けの空間索引と Growing Neural Gas の、**単一ヘッダ・外部依存なし**のパッケージです。3次元向けに既定値を調整してあります。

旧SpatialTreeを起源とする二分木の実装です。現在の正本は `include/bsp3d/bsp3d.hpp`。外部のSpatialTreeディレクトリや生成スクリプトへの依存はありません。内部の `SpatialTree::` 名前空間は既存APIとの互換用です。

## 使い方

```cpp
#include <bsp3d/bsp3d.hpp>

// 索引に入れる要素は次の3つのメンバを持つこと
struct MyNode {
  bsp3d::Point3 position;          // 座標
  void *spatial_handle = nullptr;  // 索引が使う
  int index_in_cell = -1;          // 索引が使う
};

bsp3d::Index<MyNode> index;        // 既定値は3次元向けに自動調整
index.add(&node);                  // 追加
index.updatePosition(&node, p);    // 移動（座標は必ずこれ経由で変える）
index.remove(&node);               // 削除

std::array<bsp3d::SearchResult<MyNode>, 64> res;
int k = index.findNBest(q, 2, res);   // 最近傍 2 点（距離の昇順、distance_sq は二乗距離）
```

GNG は次のように使います。

```cpp
bsp3d::GNGParams gp; gp.lambda = 100; gp.max_nodes = 20000;
bsp3d::GNG gng(bsp3d::Point3(200, 200, 200), gp, bsp3d::Params{});
gng.train_step(sample);            // 1 サンプル学習
gng.getActiveNodes();              // 学習後のノード
```

ビルドは CMake で行います。ヘッダのみなので、`include/` をインクルードパスに足すだけでも使えます。

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
./build/bsp3d_test     # 総当たりとの突き合わせテスト
./build/bsp3d_basic    # 索引の基本操作
./build/bsp3d_gng      # GNG の学習
```

## 過去の性能記録（Apple M3、float、GNG の学習ループ 1反復あたり）

定常状態（ノード数の20倍の反復で暖機した後）で測定した値です。

| 条件 | 最適化前 | **本パッケージ** |
|---|---|---|
| 3次元・一様分布・1万ノード | 0.473 µs | **0.375 µs** |
| 3次元・一様分布・10万ノード | 1.133 µs | **0.618 µs** |
| 3次元・多様体分布・10万ノード | 0.794 µs | **0.439 µs** |

SIMD 総当たりとの比較（同一実行内で測ったもの。実行ごとに絶対値は ±20% ほどぶれます）:

| 条件 | SIMD 総当たり | 本パッケージ | 倍率 |
|---|---|---|---|
| 3次元・一様分布・1万ノード | 3.97 µs | 0.50 µs | 8 倍 |
| 3次元・一様分布・10万ノード | 33.4 µs | 0.81 µs | 41 倍 |
| 3次元・多様体分布・10万ノード | 34.4 µs | 0.66 µs | 52 倍 |

他方式との比較（3次元、同一条件）:

| 方式 | 一様分布・10万 | 多様体分布・1万 |
|---|---|---|
| **本パッケージ（木）** | **0.575 µs** | **0.284 µs** |
| 密配列の一様グリッド | 0.581 µs | 0.956 µs |
| ハッシュの一様グリッド | 0.761 µs | 0.974 µs |

一様分布ではグリッドと互角、分布が偏ると木が 3.4 倍速くなります。グリッドはセル幅の調整が必要ですが、本実装は次元から自動で決めます。

## 主な最適化

- 葉は座標を 8 点ずつ軸ごとに並べて持ち（SoA）、要素ポインタも同じブロックに入れる。8 点の距離は SIMD でまとめて計算
- 上位 2 件までの探索では、葉のブロックから上位 2 点だけを候補に入れる（4 次元以下で有効）
- 点が葉の領域に留まる限り、移動は O(次元) の比較だけで終わる。領域を出たときは、その境界を作った祖先へ直接跳ぶ
- GNG の隣接は連続配列（辺ごとのメモリ確保をしない）
- 葉の容量・枝刈り方式は次元から自動決定（3 次元では葉 32、bbox なし）

## パラメータ

既定のままで動きます。変えるときは `bsp3d::Params` を渡します。

| 項目 | 既定（3次元） | 意味 |
|---|---|---|
| `max_leaf_size` | 32（自動） | 葉の容量。超えたら分割する |
| `use_bbox` | 0（自動） | 点の実際の範囲でも枝刈りするか。10 次元以下では維持コストの方が大きい |
| `approx_eps` | 0 | 0 なら厳密。> 0 で (1+eps) 近似探索 |
| `split_rule` | 0 | 0: 中央値付近の隙間、1: 広がりの中点 |

## 注意

- **座標は必ず `updatePosition` 経由で変えてください。** 葉が座標の複製を持っているため、直接書き換えると探索結果がずれます
- 次元を変えたい場合は `SpatialTree::MovingBSPTree<T, float, N>` を直接使えます（本ヘッダに含まれています）

## 閉区間の範囲検索とdouble座標

`query_aabb(min_point, max_point, visitor)` は各軸の上下限を含む要素を列挙します。空の部分木を省略し、分割軸で枝刈りした後に葉の座標キャッシュを照合。呼出し側で不要な候補ソートや検索用配列確保は不要です。

```cpp
struct entry {
  bsp3d::point3<double> position;
  void *spatial_handle = nullptr;
  int index_in_cell = -1;
};
bsp3d::Index<entry, double> tree;
// tree.add(&value)による要素の追加後の問い合わせ
const bsp3d::point3<double> min_point{0, 0, 0}, max_point{1, 1, 1};
tree.query_aabb(min_point, max_point, [](const entry *value) {
  // 該当要素の利用。問い合わせ中の索引変更は禁止。
});
```

上下限が非有限または逆転した場合と、不正な検索余白は `std::invalid_argument`。入力は有限座標を前提とし、非有限値検査を保持するため `-ffast-math` の使用は避けてください。独自のヒステリシスポリシーを使う場合、第4引数 `search_margin` に位置更新時の許容幅を包含する値が必要です。既定の `Index` はヒステリシスなし、余白0。

`bsp3d::Index<T>`、`Point3`、`SearchResult<T>` の既定は従来どおりfloat。doubleの検索結果バッファには `SpatialTree::SearchResult<T, double, 3>` を使用できます。旧 `AdaptiveTree` は削除済み。

目標選択への移行結果は[2026-09-24の比較](../benchmarks/bsp3d_migration_20260924/README.md)。範囲検索は短縮しましたが、初期構築は遅く、全用途での高速化を示す結果ではありません。

## 保守

単一ヘッダを直接編集し、`cmake --build build` と `ctest --test-dir build --output-on-failure` で検証します。旧生成スクリプトは削除済み。
