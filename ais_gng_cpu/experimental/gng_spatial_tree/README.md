# bsp3d・grid版GNG（独立比較版）

2026-09-23時点の`ais_gng_cpu/src/gng_cpu`をコピーした実験版。元のcugng.cpp・cugng.hpp・CPUライブラリ・YAMLへの変更なし。API互換の共有ライブラリとして単独実行。ROS launchへの組込み・インストール先の置換は未実施。

2026-09-24に旧SpatialTree方式を削除。過去の計測スクリプトとのパス互換のため、ディレクトリ名 `gng_spatial_tree` は維持。旧版の再現にはGit履歴または保存ライブラリが必要。

## 実装

- `src/cpu/cugng.cpp`: bsp3d版。元CPU版から独立したコピー。
- `src/cpu/cugng_grid_reference.cpp`: 同時点のグリッド版比較用コピー。
- ノード追加・移動・削除に合わせた索引更新。移動時の索引更新省略なし。
- bsp3d版は`bsp3d/include/bsp3d/bsp3d.hpp`の`bsp3d::Index`（`MovingBSPTree`）を索引として利用。既存GNGの学習規則を維持し、`findNBest`で木全体の2近傍を取得。範囲検索・探索時の子セルソートなし。
- bsp3dの3次元既定値を使用。葉32点、実点群境界箱の更新なし、近似誤差0。追加・移動・削除をすべて索引API経由で反映し、葉内の座標キャッシュと同期。
- 警戒領域・寿命リセット・重点学習ラベルの判定対象も最近傍2ノード。距離判定には既存の`node.interval`・`node.s1_reset_range`・`ds.range_max`を継続使用。3番目以降のノードによる判定なし。
- 同距離時は木の探索順に依存し、グリッド版とのノードID選択順・グラフの完全一致保証なし。
- YAMLの入力範囲・ボクセル処理・ノード上限・セル当たり10ノードの管理は従来どおり。入力点の追加除外なし。固定グリッドの管理メモリは残存。

## ビルドとテスト

コンテナ内:

```bash
cmake -S /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree \
  -B /tmp/gng_spatial_copy_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/gng_spatial_copy_build -j2
ctest --test-dir /tmp/gng_spatial_copy_build --output-on-failure --timeout 30
```

APIテスト16件（2方式×8件）、最近傍検索の全走査照合1件、bsp3d同梱の索引テスト1件、計18件。空・1ノード・遠方・境界・同距離・追加・移動・削除・再初期化、寿命とラベル判定を検証。

`libgng_grid.so`・`libgng_bsp3d.so`を生成。通常のROSパッケージとは別ビルド。
既定の`GNG_DETERMINISTIC_BENCHMARK=ON`は2方式の学習乱数・LPF時間刻み（0.1秒）を固定。これは比較専用の条件。実時間・非固定乱数の動作には同オプションをOFFで再ビルド。

## 実行時間の再現

```bash
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 90 python3 \
  /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree/benchmark.py \
  --library /tmp/gng_spatial_copy_build/libgng_bsp3d.so \
  --config /ros2_ws/src/benchmarks/gng_spatial_tree_20260923/at128_snapshot.yaml \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --output /tmp/gng_spatial_result.json
```

比較元は`--library`を`libgng_grid.so`へ変更。同じ30フレームを50回入力し、先頭10回を除いた40回を集計。JSONに全フレームの時間・ノード数・エッジ数・グラフハッシュを保存。ROS変換・TF・配信・viewer・外部分類器の実行時間は含まない。入力はbagのセンサー座標をそのまま使用。

bsp3d版の比較条件・実測結果は[検証記録](../../../gng_vlut_system/docs/releases/2026-09-23_gng_bsp3d.md)。設定・起動スクリプト・集計結果は`benchmarks/gng_bsp3d_20260923/`、比較ライブラリ・生ログはGit管理外の`artifacts/gng_bsp3d_20260923/`へ保存。

SpatialTreeの最近傍2ノード版の実測結果・制約は[検証記録](../../../gng_vlut_system/docs/releases/2026-09-23_gng_spatial_nearest.md)。設定・比較起動スクリプト・集計結果は`benchmarks/gng_spatial_nearest_20260923/`、共有ライブラリ・生ログはGit管理外の`artifacts/gng_spatial_nearest_20260923/`へ保存。

旧AABB版の実測は[旧版の比較記録](../../../gng_vlut_system/docs/designs/gng_spatial_tree_20260923.md)。旧版ライブラリはGit管理外の`artifacts/gng_spatial_tree_20260923/`に保存済み。再現に必要なローカル保存物と各保存先は[計測資料README](../../../benchmarks/README.md)を参照。
