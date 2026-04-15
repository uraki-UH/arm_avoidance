# VLUT Reconstruction & ROS 2 Humble Guide

本ドキュメントは、Ubuntu (ROS 2 Humble) 環境において、既存の GNG 実験データから最新の「自己構成対応形式（Auto-Configuration）」の VLUT バイナリを再構築するための手順をまとめたものです。

## 1. ビルド手順 (Ubuntu / ROS 2 Humble)

まず、ワークスペース内でパッケージをビルドします。物理シミュレーション（ODE）や描画機能（Drawstuff）を使用しないオフライン構築用のビルド設定です。

```bash
# 依存関係のインストール
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# ビルド (GNG/VLUT構築用: Releaseモード推奨)
colcon build --packages-select gng_planner --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_ODE=OFF -DUSE_DRAWSTUFF=OFF

# セットアップ
source install/setup.bash
```

## 2. 既存データの VLUT 再構築 (Reconstruction)

既存の `gng_results/` フォルダにある各実験データの VLUT バイナリを、最新のヘッダ付き形式（解像度・範囲情報を埋め込み済み）に更新します。`--vlut-only` フラグにより、GNG の再学習はスキップされます。

### 実験 v1 (Resolution: 0.02m)
```bash
ros2 run gng_planner offline_urdf_trainer --id topoarm_real_v1 --vlut-only --res 0.02
```

### 実験 v2 (Resolution: 0.02m)
```bash
ros2 run gng_planner offline_urdf_trainer --id topoarm_real_v2 --vlut-only --res 0.02
```

### 実験 v3 (Resolution: 0.05m など、解像度を変更する場合)
```bash
ros2 run gng_planner offline_urdf_trainer --id topoarm_6d_otf_v3 --vlut-only --res 0.05
```

## 3. 最新形式（Version 2）のメリット

再構築された `gng_spatial_correlation.bin` は、以下の「自己構成機能」を備えています：

*   **自動解像度認識**: `config.txt` の設定値に関わらず、バイナリ自身が持っている解像度（0.02m 等）をロード時に自動適用します。
*   **自動範囲認識**: バイナリ構築時の空間範囲（Bounds）も自動で読み取られ、グリッドが再構成されます。
*   **不整合エラーの防止**: 実行時の設定ミスによる座標計算の狂いを原理的に防ぎます。

## 4. 非 ROS 環境 (macOS 等) での実行

ROS 2 がない環境でも、これまでの CMake 手順で実行可能です。

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_ODE=OFF -DUSE_DRAWSTUFF=OFF
make offline_urdf_trainer -j4

# 実行例
./offline_urdf_trainer --id topoarm_real_v1 --vlut-only --res 0.02
```
