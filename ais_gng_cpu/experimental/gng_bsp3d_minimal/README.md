# 入力ボクセル化なしの最小bsp3d版GNG

前回の`gng_bsp3d_sampled`をコピーした独立実験版。元CPU版・前回実験版・本番インストール先の変更なし。`COLCON_IGNORE`により通常colconビルドから除外。

## 最小処理

1. 入力点をYAMLのxyz範囲と有限座標で確認し、範囲内の元点番号だけを入力順に保持。
2. 元点番号から一様乱数で`node.learning_num`個を復元抽出。点の平均化・ソート・重複除去なし。
3. 選択した元点についてbsp3dの厳密2近傍探索、警戒領域に基づくノード追加、ノード移動、接続・エッジ寿命更新。
4. 法線・ラベル計算、ノード寿命・孤立ノードの整理。

入力ボクセル・観測用セル索引・全点のノード照合・全域探索枠・未知物体や人の重点候補生成・平面／曲面クラスタリングは未使用。最近傍探索は、範囲内入力がある場合に学習回数と同数。

## 設定とAPI

`node.learning_num`、`node.num_max`、学習係数、ノード間隔、寿命、エッジ寿命、入力範囲・上限などは使用。

`node.grid`、`input.voxel_grid_unit`、`sampling.*`、`node.unknown_learning_rate`、`node.s1_reset_range`、`ds.*`、`cluster.*`、`classify.*`は最小版で未使用。`gng_setParameter`はこれらに失敗を返却。比較用YAMLに含まれる未使用項目は計測JSONの`ignored`へ記録。

`gng_set_priority_input`・`gng_set_weighted_priority_input`は空指定のみ受理。`gng_getDownSampling`は元点ごとの範囲内フラグだけを返却。クラスタ出力は0件。法線と地形ラベルは既存処理を維持。観測支持・学習イベント・差分・近傍数の基本APIは保持。

`gng_get_minimal_statistics()`で元点準備時間と元点数を含む計測値を取得。`voxel_ms`・`attention_ms`・`cluster_ms`は0。元点準備の全点走査は`input_prepare_ms`へ計上。

## 挙動の差

- 元点の密度と重複数が、そのまま学習確率に反映。原点の重複も除外なし。
- ノード寿命のリセットは学習で選ばれた勝者だけ。近傍に点が観測された全ノードの寿命維持は未実施。
- 事前の全点照合を使わないため、新規ノード追加時に取得済み近傍へ接続。
- 学習・グラフの範囲被覆・ノード数は従来版と非同値。処理を省いた結果の時間であり、同じ品質・機能での高速化ではない。
- 密なエッジ配列は従来どおり。最大20000ノードで約2 GBの配列確保。
- 比較ビルドの乱数とLPF刻みは固定。実時間利用には`GNG_DETERMINISTIC_BENCHMARK=OFF`での再ビルドが必要。

## ビルド・検証

コンテナ内:

```bash
cmake -S /ros2_ws/src/ais_gng_cpu/experimental/gng_bsp3d_minimal \
  -B /tmp/gng_bsp3d_minimal_build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build /tmp/gng_bsp3d_minimal_build -j2
ctest --test-dir /tmp/gng_bsp3d_minimal_build --output-on-failure --timeout 30
```

[再現スクリプトと設定](../../../benchmarks/gng_bsp3d_minimal_20260923/)。

計測結果は[検証記録](../../../gng_vlut_system/docs/releases/2026-09-24_gng_bsp3d_minimal.md)を参照。
