# 2026-09-29 - 平面形状判定の共通化

## 要約

面内縦横比判定を生成確定時・統合時へ集約。独立設定3個を1個へ削減。
生成・統合の正規化RMS計算も共通関数へ集約。続報で接続長の共通化と統合残差悪化判定を撤去し、追加3設定を削減。

- 共通設定：`min_plane_aspect_ratio: 0.45`。
- 旧`min_cluster_planarity`、`min_growth_planarity`、`merge_min_planarity`の宣言・内部フィールドを撤去。
- 旧設定の自動変換なし。独自YAMLでは旧3項目を削除して共通設定へ移行が必要。
- 縦横比は面内短軸／長軸の標準偏差比であり、厚みを示す量とは別物。
- 面幅条件`min_plane_width_ratio`とのOR判定を維持。長い路面の過剰分割防止。
- 未完成の種・成長途中への形状判定と、その整合調整コードを撤去。
- 成長途中の各入力点の距離・法線判定と倍化時の再フィットは維持。
- 生成・統合の厚み上限は既存`max_normalized_cluster_residual`を継続使用。
- 単一点の取り込み、所属保持、統合各側の残差は評価対象が異なるため独立性を維持。
- 取り込み・断片統合の接続長を`max_connection_edge_ratio_th: 2.5`へ統一。
- 旧`max_absorption_edge_ratio_th`と`max_fragment_edge_ratio_th`は上記へ置換。異なる値を使っていた独自設定では共通値の選択が必要。
- `merge_residual_growth_ratio`と`merge_residual_growth_min_th`は削除。独自YAMLからも削除が必要、自動移行なし。
- 「元より残差が増えたか」の判定・統計・単独ノードのgrowthログを撤去。統合後の厚み、各側・接触部・近接候補の残差判定は維持。
- 時間安定化、法線、接続証拠、短い1本接続の時間確認は維持。

全パラメータを4系統へ再設計した完成版ではなく、形状判定の共通化段階。
既存の`planarity`出力フィールドは互換維持。面内縦横比の意味も維持。
小さい種まで面幅だけで制限する方式は今回未実装。

## 条件・検証

- 最終版の平面回帰74件成功。CPU/GPUコンポーネント・単独平面ノード・試験のビルドとinstall成功。
- 起動したビルド・試験は全て終了。既存ROS・コンテナの停止・再起動なし。
- 共通値0.25の試行では細帯除外の回帰試験が失敗。従来の確定値0.45へ変更。
- 0.45を種にも適用する試行では不均一接続の回帰が失敗。未完成の種への形状制限を撤去。
- 追加試験：縦横比の共通設定による長方形の生成、倍率0.1・1・10の一様拡大縮小。
- 追加試験：共通接続長の両経路反映、400点対25点の同一面・微小ノイズ統合と段差・傾斜の分離。3スケール・両入力順で確認。
- 続報の比較は形状共通化済みの変更前（7abe0c5相当）と接続長共通化・悪化判定撤去後。形状共通化そのものの効果は対象外。
- 保存済み交差点由来60グラフ、先頭20入力除外、CPU 4固定、同じ-O3ビルド、前後各5回。ROS配信・GNG学習なし。
- 平面CPU時間の試行平均6.732→6.848 ms（+1.7%）、中央値6.705→6.847 ms。高速化の主張なし。
- 計測40入力の平均クラスタ数68.525→65.250、平均所属ノード数10757.975→10702.000、有効ノード所属率55.495→55.207%。
- 計測40入力で旧悪化判定の棄却259件。各方式5回の出力指紋は一致、方式間では全60入力中57入力で変化。
- 10試行成功、予測10秒・実測4.68秒、全試行cleanup_ok確認。クラスタ数の減少は品質改善の証拠ではなく、正解ラベル・実画面による誤統合評価は未実施。
- 候補探索の漸近計算量は変更なし。細帯の早期棄却をなくしたため、棄却までの探索量が増える可能性あり。

既存コンテナ内でROS環境を読込み、以下を実行。

```bash
timeout 240 cmake --build /ros2_ws/build/ais_gng --target ais_gng_component_cpu ais_gng_component_gpu plane_cluster_incremental_node -j2
timeout 180 cmake --build /ros2_ws/build/ais_gng --target test_plane_cluster_incremental -j2
timeout 60 /ros2_ws/build/ais_gng/test_plane_cluster_incremental
timeout 60 cmake --install /ros2_ws/build/ais_gng
cd /ros2_ws/src
timeout 180 bash benchmarks/plane_merge_simplification_20260929/build.sh
python3 /tmp/plane_merge_simplification_run_batch.py benchmarks/plane_merge_simplification_20260929/cases.json --output artifacts/plane_merge_simplification_20260929/batch --repeats 5 --timeout-sec 20 --max-total-sec 180 --estimate-sec 1
```

比較条件：[cases.json](../../../benchmarks/plane_merge_simplification_20260929/cases.json)、[正規化済み設定](../../../benchmarks/plane_merge_simplification_20260929/plane_parameters.txt)。
結果正本：`artifacts/plane_merge_simplification_20260929/batch/report.json`と各試行の`statistics.csv`・`fingerprints.csv`。
ビルドの前提：同ディレクトリの保存済み`before.cpp`・`before_include`と、`artifacts/voxel_plane_lookup_20260928/inheritance_perf/capture/graphs.bin`。
runnerは`run-benchmark-batch/scripts/run_batch.py`をコンテナの上記`/tmp`へコピー。再試行では未使用の出力先を指定。
