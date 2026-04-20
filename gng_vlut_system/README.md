# Multi-Layer Geometric GNG Project

このプロジェクトは、Growing Neural Gas (GNG) アルゴリズムを用いたロボットアームの軌道計画および自己干渉回避システムです。
角度空間と座標空間のデュアルスペース学習、および幾何学的な自己干渉判定を特徴としています。

## 依存ライブラリ (Dependencies)

このプロジェクトのビルドには以下の外部ライブラリが必要です。

### 必須 / Core Libraries

- **Eigen3**: 解析計算、ベクトル・行列演算に使用。
- **urdfdom** / **urdfdom_headers**: URDFロボットモデルのパースに使用。
- **tinyxml2**: XMLのパースに使用（urdfdomで使用）。
- **console_bridge**: ロギングユーティリティ。


> ROS 2 では `gng_safety` パッケージの launch と RViz2 を使って可視化します。

## セットアップ (Build Instructions)

### legacy CMake / 非 ROS2 環境での依存導入例

```bash
brew install eigen opencv tinyxml2 console_bridge urdfdom ode freeglut
```

### ビルド

```bash
mkdir build
cd build
cmake ..
make -j4
```

## ガイド

- [ROS2 Guide](ROS2_GUIDE.md)
- [Offline Guide](OFFLINE_GUIDE.md)

## GNGグラフの生成パイプライン (Pipeline Workflow)

ステータス付きの高精度なGNGグラフを生成するには、以下の順序で各オフラインプログラムを実行します。各プログラムは `config.txt` の設定に従って自動的に入出力を接続します。

1. **自己干渉を考慮した学習 (Phase 2)**

   ```bash
   ./build/offline_gng_main3
   ```

   幾何学的な自己干渉判定を用いて、ロボットが自分自身にぶつからない関節角空間を学習します。

2. **環境障害物との干渉チェック (Phase 3)**

   ```bash
   ./build/offline_gng_main4
   ```

   生成されたグラフを環境点群と照合し、障害物と干渉するノード/エッジに無効フラグを立てます。

3. **最大連結成分の抽出 (Phase 4 - Island Pruning)**

   ```bash
   ./build/offline_gng_main6
   ```

   到達不可能な孤立した島（アイランド）を除去し、最大のネットワークのみを残します。

4. **クリーンアップと特徴量計算 (Final Phase)**

   ```bash
   ./build/offline_gng_main5
   ```

   無効な要素を物理的に削除し、全ノードに対して「可操作度（Manipulability）」や「手先方向」などの高度なステータスを計算・付与します。

## 実行：オンライン統合 (Simulation)

生成された最終マップ（`..._final.bin`）を使用して、シミュレーションを実行します。

```bash
./build/online_integration
```

## 設定

- `config.txt`: 全体の実験IDやファイル入出力サフィックスの設定。
- `gng_offline.cfg`: オフライン学習時の学習率や最大ノード数の設定。
