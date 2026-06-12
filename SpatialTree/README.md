# GenericSpatialTree: High-Performance N-Dimensional Spatial Index

`GenericSpatialTree` は、高性能な N次元空間分割ツリー（Quadtree/Octree）を提供する、**ゼロ依存・ヘッダーオンリー**の C++17 ライブラリです。
GNG (Growing Neural Gas) などの動的なトポロジー学習や、リアルタイム性が要求される点群データの近傍探索などに利用可能

## 説明

- 近傍探索の結果を `SearchResult`（要素と**二乗距離**のペア）で返す
- ノードがセル境界で頻繁に入れ替わるのを防ぐマージン設定可能。探索時にも動的マージンも可能
- **ヘッダーオンリー & N次元対応**:
  - 外部依存なし。テンプレート引数で任意の次元に対応。(ただし2,3次元がメインで最大でも6次元程度を推奨)

## ディレクトリ構造

- **`include/SpatialTree/` (Core)**:
  - `SpatialTree.hpp`: 最適化されたアダプティブ空間分割木。
  - `Point.hpp`, `Traits.hpp`, `Policy.hpp`: 木構造を支える基盤。
- **`include/SpatialTree/GNG/` (Application)**:
  - `GNG.hpp`, `GNGUtility.hpp`: 空間分割木を応用したトポロジー学習。

## 基本操作

```cpp
#include <SpatialTree/SpatialTree.hpp>

struct MyNode {
    SpatialTree::Point<float, 2> position;
    void* spatial_handle = nullptr; // 高速更新用に必須
};

// 1. ツリーの初期化 (2D, float, 領域 1000x1000の場合)
SpatialTree::AdaptiveTree<MyNode, float, 2> tree({1000.0, 1000.0}); 

MyNode node;
node.position = {500.0, 500.0};

// 2. ノードの追加
tree.add(&node);

// 3. N-best 探索 (SearchResult を取得)
auto results = tree.findNBest(target_pos,n); 
//(target_posに近いノードを最大n個 返す)

// 4. ノード位置更新 
SpatialTree::Point<float, 2> next_pos = {505.0, 502.0};
tree.updatePosition(&node, next_pos);

高速化したい場合,
隣接ノード更新の際に,微小更新のためセル更新はしないとみなし,
setPosition(今の座標,更新後の座標)
で座標更新だけ実行してもよい
-   **注意点**: 座標とツリー内の位置が一時的にわずかに食い違う可能性がありますが、次にそのノードが勝者（s1）として選ばれた瞬間に正しい位置へ再インデックスされるため、アルゴリズムの収束性への影響は軽微です。

(やや厳密性は損なわれるが計算時間は概ね2倍以上は速くなる)
ベンチマークでもその仕組みを適用

// 5. ノードの削除
tree.remove(&node);
```

## パフォーマンス (比較)

1,000,000 回の学習（100,000 ノード）における、1 回あたりの平均実行時間（µs/iter）
※ Apple M3環境での実測値。

| 次元 | 総当たり (Brute Force) | **SpatialTree (Optimized)** | 性能向上 |
| :--- | :--- | :--- | :--- |
| **2D** | 66.2 µs | **0.67 µs** | **~100 倍** |
| **3D** | 67.8 µs | **1.20 µs** | **~56 倍** |
| **5D** | 113.2 µs | **4.7 µs** | **~24 倍** |
| **7D** | 122.7 µs | **33.1 µs** | **~4 倍** |

## ビルド方法

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
./2d_benchmark  # 2D パフォーマンス
./3d_benchmark  # 3D パフォーマンス
./update_mechanism_benchmark # 更新アルゴリズムの比較
```
