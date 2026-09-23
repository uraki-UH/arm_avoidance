# bsp3d版GNGのボトルネック計測（2026-09-23）

## 結論

現在の独立bsp3d版では、全ボクセル代表点のノード照合を含む`GNG::attention()`が61.01 ms、GNG本体95.78 msの約64%を占める。学習4,000回の前に約79,577点すべてで最近傍2ノードを検索し、接続を更新する構造が主要因。

次にボクセル処理10.32 ms、法線・ラベル計算9.86 ms。学習ループは5.10 msであり、学習回数だけの削減による改善余地は限られる。

## 対象と計測方法

- 対象は`ais_gng_cpu/experimental/gng_spatial_tree`の`libgng_bsp3d.so`。通常のROS launch全体は対象外。
- 前回計測時のcugngソース・bsp3dヘッダーとのSHA-256一致を確認。
- 実装を一時コピーし、Release・`-O3 -DNDEBUG`・近似なし・学習乱数／時間刻み固定で計測。元実装・YAMLへの変更なし。
- 同一bagの30フレームを50回入力し、最初の10回を除く40回を集計。入力16万点、ボクセル代表点平均79,576.85点。
- 段階計測、全呼び出し詳細計測、64回に1回の標本計測、タイマーなしの保存ライブラリ対照を実施。

## 段階別の実測

高頻度タイマーを無効にし、段階境界だけを計時した結果。

| 処理 | 平均 ms | 内容 |
| --- | ---: | --- |
| 全ボクセル点の照合・重点学習入力生成 | 61.01 | 2近傍探索、追加判定、接続更新、ラベル反映、対応表の整列 |
| ボクセル処理 | 10.32 | YAML入力範囲、ボクセル番号の整列、代表点の平均化 |
| 法線・ラベル計算 | 9.86 | 各ノードの法線、rho、fuzzy判定、LPF |
| GNG内部クラスタリング | 6.46 | コアライブラリ内の分類処理 |
| 学習 | 5.10 | 4,000回の最近傍取得、勝者と隣接ノードの移動など |
| グラフの維持 | 2.92 | エッジ距離、寿命、孤立ノードなどの確認 |

API本体95.78 ms、入力変換1.56 ms、結果生成1.35 ms、合計98.69 ms。タイマーなし対照は本体95.37 ms、合計98.27 msで、段階計測による本体差は約0.41 ms。

GNG内部クラスタリングは、ROS側の平面クラスタリングやCurve検出とは別。後者の実行時間はこの直接API計測に含まれない。

## 呼び出し回数と詳しい箇所

`gng_exec`内の1フレーム平均。入力変換時のノード移動はこの集計外。

| 処理 | 回数 |
| --- | ---: |
| 全ボクセル点の最近傍照合 | 79,576.85 |
| 学習時の最近傍照合 | 4,000 |
| BSP最近傍探索の合計 | 83,576.85 |
| `connect`呼び出し | 83,576.85 |
| 木の位置更新 | 26,443.25 |
| ノード追加関数の呼び出し | 18,231.70 |
| 実際のノード追加 | 112.48 |
| 容量上限による追加拒否 | 18,119.03 |
| 空きノード探索の確認数 | 1,116,125.73 |

探索の約95.2%は学習前の全点照合。`getDownSampling()`は全代表点を一度ずつ処理し、`getDownSamplingGrid()`→`query_spatial()`→`findNBest(..., 2)`、必要に応じた追加、`connect()`を実行。ここは`node.learning_num`の回数制限を受けない。

詳細計測ではBSP最近傍探索が最大。標本推定ではBSP検索約51.4 ms、接続更新約14.0 ms、木の位置更新約1.5 ms。ボクセルの整列は約8.2 ms、対応表の整列は約3.5 ms。後二者は子セルソートとは別の処理。

ただし高頻度関数の推定時間には計時自体の費用と標本の偏りを含む。標本推定の部分時間が実際の包含区間を超える場合もあり、合算や厳密な配分には不適切。処理全体の配分には上の段階別計測を採用。ノード追加関数は詳細／標本とも約1.7〜1.8 msで、空きID走査は改善候補だが最大要因ではない。

## 計測負荷と結果の一致

| 計測方法 | 本体平均 ms | 前回保存結果と一致したフレーム |
| --- | ---: | ---: |
| タイマーなし対照 | 95.37 | 50 / 50 |
| 段階計測 | 95.78 | 50 / 50 |
| 全呼び出し詳細計測 | 131.83 | 50 / 50 |
| 64回に1回の標本計測 | 101.18 | 50 / 50 |

全呼び出し詳細計測は約36 msの負荷増加。標本計測は最近傍探索約1,306回／フレームを実測し、全呼び出し回数へ外挿。入力点やGNG処理を省いたものではない。

## 次に検討する箇所

1. 全代表点の2近傍探索と接続更新の実装。比較時は入力点数・近傍条件・グラフ出力を維持して効果を確認。
2. ボクセル番号・ノード対応表の整列と法線・ラベル計算。
3. 空きノードIDの毎回の走査。

これらは調査結果からの改善候補。今回の実装変更や効果検証は未実施。木の追加・移動・削除だけを重点的に最適化しても、現在の主要な全点照合の費用は残る。

## ソース参照

- [全代表点の照合](../../../ais_gng_cpu/experimental/gng_spatial_tree/src/cpu/cugng.cpp): `getDownSampling`、`query_spatial`、`connect`。
- [処理段階と対応表の整列](../../../ais_gng_cpu/experimental/gng_spatial_tree/src/cpu/gng.cpp): `exec`、`attention`。
- [ボクセル処理](../../../ais_gng_cpu/experimental/gng_spatial_tree/src/cpu/voxel_grid.cpp): `applyFilter`。
- [法線・ラベル](../../../ais_gng_cpu/experimental/gng_spatial_tree/src/cpu/labelling.cpp): `labelling_fuzzy`。

## 保存物と起動コマンド

計測器・実行スクリプト・保存済み集計`summary.json`・タイマーヘッダーは[benchmarks/gng_bsp3d_profile_20260923](../../../benchmarks/gng_bsp3d_profile_20260923/)へ移管。全フレームJSON・詳細ログ・ビルド条件の生成ファイルはGit管理外の`artifacts/gng_bsp3d_profile_20260923/`にローカル保管。再集計の出力先も`artifacts/`であり、Git管理中の集計値は上書きなし。

以下はコンテナ内での起動コマンド。各Python計測は実行スクリプト内で`timeout -s INT -k 5 120`つき。

```bash
python3 /ros2_ws/src/benchmarks/gng_bsp3d_profile_20260923/instrument.py \
  /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree /tmp/gng_bsp3d_profile_source
# 初回の全呼び出し計測器の再現用ヘッダー
cp /ros2_ws/src/benchmarks/gng_bsp3d_profile_20260923/full_timer.hpp \
  /tmp/gng_bsp3d_profile_source/src/cpu/bsp_profile.hpp
cmake -S /tmp/gng_bsp3d_profile_source -B /tmp/gng_bsp3d_profile_build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DSPATIAL_TREE_INCLUDE_DIR=/ros2_ws/src/SpatialTree/include \
  -Dbsp3d_include_dir=/ros2_ws/src/bsp3d/include
cmake --build /tmp/gng_bsp3d_profile_build --target gng_bsp3d -j2
bash /ros2_ws/src/benchmarks/gng_bsp3d_profile_20260923/run_profile.sh

python3 /ros2_ws/src/benchmarks/gng_bsp3d_profile_20260923/instrument.py \
  /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree /tmp/gng_bsp3d_sample_profile_source
cmake -S /tmp/gng_bsp3d_sample_profile_source -B /tmp/gng_bsp3d_sample_profile_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DSPATIAL_TREE_INCLUDE_DIR=/ros2_ws/src/SpatialTree/include \
  -Dbsp3d_include_dir=/ros2_ws/src/bsp3d/include
cmake --build /tmp/gng_bsp3d_sample_profile_build --target gng_bsp3d -j2
bash /ros2_ws/src/benchmarks/gng_bsp3d_profile_20260923/run_sample_profile.sh
python3 /ros2_ws/src/benchmarks/gng_bsp3d_profile_20260923/summarize.py
```

全計測セッションは終了。新規ROSノードの起動なし、既存ROS・再生への停止操作なし。CPU固定なし、1条件での測定であり、別の点群分布やノード数では配分が変わる。
