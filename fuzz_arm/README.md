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

### シミュレーション・可視化 / Simulation & Visualization
- **ODE (Open Dynamics Engine)**: 物理シミュレーションおよび基本的な干渉検出。
- **drawstuff**: ODEに付属する簡易レンダリングライブラリ。
- **OpenGL**: 3Dグラフィックス描画。
- **GLUT**: ウィンドウ管理および入力処理。
- **OpenCV**: 画像処理、一部の可視化機能（オプション）。

## セットアップ (Build Instructions)

### macOS (Homebrewを使用する場合)
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
- **'g' キー**: ランダムな目標地点を生成。
- **'f' キー**: GNGグラフに基づいた経路計画と移動を開始。
- **'1'〜'4' キー**: 経路計画のコスト指標を動的に切り替えます（'f'コマンド実行時に使用されます）。
    - **'1'**: 関節空間ユークリッド距離 (Joint Distance) - 最短経路を優先。
    - **'2'**: 運動学的可操作度 (Kinematic Manipulability) - 特異点を回避し、動きやすい姿勢を維持。
    - **'3'**: 動的可操作度 (Dynamic Manipulability) - 動的な動作のしやすさを考慮。
    - **'4'**: 方向的可操作度 (Directional Manipulability) - 手先方向への動きやすさを最大化。
    - **'5'**: ワークスペース距離 (Workspace Distance) - 手先位置の移動量を最小化。

## 設定
- `config.txt`: 全体の実験IDやファイル入出力サフィックスの設定。
- `gng_offline.cfg`: オフライン学習時の学習率や最大ノード数の設定。
- `gng_online.cfg`: オンライン制御時のパラメータ。
