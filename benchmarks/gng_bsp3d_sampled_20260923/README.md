# 固定ノードグリッド撤去・照合回数制限の比較

独立版の仕様は[実装README](../../ais_gng_cpu/experimental/gng_bsp3d_sampled/README.md)、計測条件と結果は[検証記録](../../gng_vlut_system/docs/releases/2026-09-23_gng_bsp3d_sampled.md)。通常ROSへの組込みなし。

## 保存資料

- `at128_voxel_0_1.yaml`、`at128_voxel_0_5.yaml`: 最終比較設定。全域探索上限16000回、学習4000回。
- `at128_fast_*.yaml`: 探索上限4000回の低負荷設定。0.1 m条件で細部の被覆率低下あり。
- `at128_probe_*.yaml`: 探索上限8000・16000回の比較設定。
- `at128_node_grid_*.yaml`: `node.grid`非依存の比較設定。
- `summary.json`: 最終3試行・連続300フレーム・グリッド非依存検証の保存済み集計。
- `sha256.json`: 最終ソース、設定、計測ライブラリの照合資料。

入力YAMLは以前の比較設定のスナップショット。通常launchの現在値を取得した設定ではない。ROS用項目のうちコアAPIが受理しない項目は生JSONの`ignored`へ記録。

## 再現

既存コンテナ`gng_cpu_container`内のROS Humble、Pythonのnumpy・scipy・PyYAML・rosbag2_py、Boost・OpenSSL・ykpivを使用。追加インストールなし。グラフ描画にはmatplotlibも必要。

1. 実装READMEのコマンドでReleaseビルドとCTestを実行。
2. `artifacts/gng_bsp3d_sampled_20260923/`を作成し、ビルド先から`libgng_bsp3d_pure.so`と`libgng_bsp3d_sampled.so`を配置。
3. 比較元の`artifacts/gng_bsp3d_20260923/libgng_grid.so`と`libgng_bsp3d.so`を確認。生成方法は[旧独立版README](../../ais_gng_cpu/experimental/gng_spatial_tree/README.md)。保存済みバイナリのSHA-256は`sha256.json`を参照。
4. コンテナ内で以下を実行。

```bash
bash /ros2_ws/src/benchmarks/gng_bsp3d_sampled_20260923/run_benchmarks.sh 3
bash /ros2_ws/src/benchmarks/gng_bsp3d_sampled_20260923/run_extended.sh
bash /ros2_ws/src/benchmarks/gng_bsp3d_sampled_20260923/run_grid_invariance.sh
python3 /ros2_ws/src/benchmarks/gng_bsp3d_sampled_20260923/summarize.py
OPENBLAS_NUM_THREADS=1 python3 /ros2_ws/src/benchmarks/gng_bsp3d_sampled_20260923/plot_comparison.py
```

使用bagは`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`、トピックは`/lidar_points`。bagファイルからの直接読込で、ROSノードやbag再生の新規起動なし。

生JSON・ログ・出力グラフ・共有ライブラリ・再集計結果はGit管理外の`artifacts/gng_bsp3d_sampled_20260923/`へ保存。`summarize.py`による、このディレクトリの保存済み集計の上書きなし。生データのないcheckoutでは再計測が必要。

同一版の試行間グラフ照合と、同一版の`node.grid`違いの照合を実施。異なる方式同士のグラフ完全一致を示すものではない。
