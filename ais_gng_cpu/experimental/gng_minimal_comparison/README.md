# 最小GNGの入力ボクセル・ノード探索比較

`gng_bsp3d_minimal`の独立コピー。`COLCON_IGNORE`付きで通常のROSビルド・launch・本番ライブラリへの組込みなし。

| ライブラリ | 学習候補 | ノード探索 |
| --- | --- | --- |
| `libgng_minimal_tree_raw.so` | YAML範囲内の元点 | bsp3dの厳密2近傍 |
| `libgng_minimal_tree_voxel.so` | 入力ボクセルの重心 | bsp3dの厳密2近傍 |
| `libgng_minimal_grid_raw.so` | YAML範囲内の元点 | 従来の周囲27セル |
| `libgng_minimal_grid_voxel.so` | 入力ボクセルの重心 | 従来の周囲27セル |

## 共通処理

- YAMLの点数上限・座標範囲、有限座標の確認。原点・重複点への独自フィルタなし。
- `node.learning_num`回の復元抽出による一様学習。非空入力では1学習あたり1回の最近傍検索。
- 未被覆位置へのノード追加と直後の接続、勝者・隣接ノード移動、エッジ寿命更新、勝者の寿命リセット。
- 既存の法線・ラベル計算、ノード寿命・孤立ノードの処理。
- 全入力とノードの事前照合、観測セル索引、重点サンプリング、平面・曲面クラスタリングなし。

入力ボクセル版はセル番号の整数ソート・連続区間・重心計算を使用。空間全体の入力ボクセル配列は確保せず、入力点数に対応する配列だけを保持。元CPU版の処理手順を基に有限座標とYAML上端の境界点を保持し、重心の丸め誤差を境界内へ制限。重心は元画素番号を持たないため、観測支持API使用時は角度表を使わず方向ベクトルから算出。

`node.grid`版は元CPU版の27セル探索、セルごとの最大10ノード、256セル単位の遅延ページ確保を移植。セル別ノード数・ページ番号の配列は設定空間全体に対応。満杯セルへの追加・移動を拒否。ツリーの警戒領域判定は最近傍2ノード、グリッドは探索セル内の全候補。局所探索範囲・収容数・警戒領域判定が異なり、グラフが同一となる方式間比較ではない。

## 設定と計測API

- `input.voxel_grid_unit`: voxel版だけで受理。有限な正値。raw版では拒否。
- `node.grid`: grid版だけで受理。有限な正値。tree版では拒否。
- `sampling.*`・`ds.*`・`cluster.*`・`classify.*`・`node.unknown_learning_rate`・`node.s1_reset_range`: 全版で拒否。
- 既存の学習・寿命・ラベル・入力範囲の設定は共通。設定の受理・拒否を計測JSONへ保存。
- `gng_get_comparison_statistics()`: 比較版の統計形式を識別するAPI。
- `input_prepare_ms`: 範囲確認または入力ボクセル化と候補番号整理。voxel版の`voxel_ms`は同じ時間の別名であり、合算不可。
- `num_input_points`: 範囲内・有限な元点数。`num_training_points`: 一様抽出対象の元点数または占有ボクセル数。
- `num_tree_moves`: 既存統計との互換用名称。grid版では受理したノード移動呼出回数。セル移動を伴わない座標更新も含む。

## Releaseビルドと検証

コンテナ内:

```bash
cmake -S /ros2_ws/src/ais_gng_cpu/experimental/gng_minimal_comparison \
  -B /tmp/gng_minimal_comparison_build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DGNG_DETERMINISTIC_BENCHMARK=ON
cmake --build /tmp/gng_minimal_comparison_build -j 4
ctest --test-dir /tmp/gng_minimal_comparison_build --output-on-failure
cp /tmp/gng_minimal_comparison_build/libgng_minimal_*.so \
  /ros2_ws/src/artifacts/gng_minimal_comparison_20260924/
```

比較時は乱数をフレーム番号、ラベルLPFの時間刻みを0.1秒に固定。本番利用を想定する場合は`GNG_DETERMINISTIC_BENCHMARK=OFF`が必要だが、今回の計測・ROS組込みの対象外。

API16件、厳密2近傍・局所近傍・入力重心の3件、計19件。局所近傍テストは移動・削除・ID再利用、セル満杯時の拒否を検証。

[計測条件と保存済み結果](../../../benchmarks/gng_minimal_comparison_20260924/README.md)。

## 制約

最小化によって全点からの観測寿命維持と重点学習を除去。元の本番版と同じ機能・品質ではない。生点群では点密度、ボクセルでは占有セル数に比例した学習配分。固定4,000回の学習では、入力候補を減らしても検索回数自体は減らない。

エッジ表は従来のノード上限の二乗に比例する構造を維持。今回の20,000ノード上限では、この共通構造のメモリ消費が大きい。計測JSONの最大RSSにはbagを先読みしたPythonプロセスも含む。
