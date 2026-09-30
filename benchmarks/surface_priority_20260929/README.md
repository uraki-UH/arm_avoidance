# 曲面パッチ履歴・優先順の固定入力比較

履歴追加前のsnapshotと追加後のON/OFFを、同じGNGノード・平面所属の入力列で比較。
生成物と数値の正本は `artifacts/surface_priority_20260929/`。

- `before/` は作業開始時の未改変ソース。`after` はビルド時点のworkspaceソース。
  正式比較時の新実装ソースは `after_snapshot/` にも保存。
  最終ガード追加後の比較は `after_final_snapshot/` と `batch_final/`、先行結果は上書きなし。
- `small`：保存済み1,545ノード・10平面・21入力。
- `large_curve`：半径0.1 mの円柱288点・8平面と、遠方の単一平面19,000点、20入力。
- `input`：交差点bag由来、平均19,143.9ノード、100入力。
- `micro_curve`：`large_curve` の円柱のみy回転振幅0.003 rad、x並進振幅0.001 m、40入力。
- 元入力の出典は[既存ベンチ](../surface_local_20260929/README.md)、ハッシュは `input_provenance.json`。
- 保存形式に未収録の平面重心・位置共分散は、所属点から読込時に補完。全比較条件で同じ補完、時間計測外。
- 保存入力のノード生成世代は未収録のため0。世代再利用の安全性は本ベンチの検証対象外（別途単体試験）。
- `tracker.update()` のCPU時間・経過時間のみ計測。入力デコード・結果保存・品質検査・ROS描画は計測外。
- 曲面所属、モデル型・半径[m]・点モデル残差[m]・再利用数・保留数をフレーム別保存。元平面の所属変更なし。
- 曲面出力なしの残差はフレーム記録でnull、`num_residuals=0`。有効入力のない残差集計キーは省略。
  先行 `batch/` は未評価残差を0と記録しており、最終評価は `batch_final/` を参照。
- 合成円柱は先頭5入力を除き曲面支持率95%、全入力で遠方平面混入0の品質検査。
  残差0だけでは出力消失と区別できないため、支持率と曲面ノード数を併記。
- OFFは変更前との曲面出力完全一致を検査。ONは曲面所属集合IoUと元曲面ごとのbest IoUを集計。
  人手正解のない交差点入力のIoUは互換性指標であり、正解精度ではない。
- 通常運用の既定予算を利用。直接 `extract()` ではなく `tracker.update()` でON/OFFを切替。
- 履歴・優先順・予算管理は `retention_options.enable_retention=true`（既定値）が前提。

下記standalone手順は既存 `gng_cpu_container` のみ使用。既存ROSノードの停止・本番overlayのビルドなし。
各コマンドはホストで実行、出力先 `batch_final` が存在する場合は新しい名前への変更。

```bash
docker exec gng_cpu_container bash -c '
set -eo pipefail
cd /ros2_ws/src
timeout 120 python3 -B benchmarks/surface_priority_20260929/prepare.py
timeout 600 bash benchmarks/surface_priority_20260929/build.sh before
timeout 600 bash benchmarks/surface_priority_20260929/build.sh after
timeout 600 python3 -B skills/run-benchmark-batch/scripts/run_batch.py \
  artifacts/surface_priority_20260929/cases.json \
  --output artifacts/surface_priority_20260929/batch_final --repeats 3 \
  --timeout-sec 90 --max-total-sec 580 --estimate-sec 2
python3 -B benchmarks/surface_priority_20260929/compare.py \
  artifacts/surface_priority_20260929/batch_final
python3 -B benchmarks/surface_priority_20260929/summarize.py \
  artifacts/surface_priority_20260929/batch_final
'
```

`report.json` は試行数・実行失敗・後片付け、`quality_comparison.json` は所属比較。
`summary.json` / `summary.csv` は試行ごとの平均・p95等の中央値。フレームの一括平均ではない。
各条件の `command.json` が実際の起動コマンド、`frames.jsonl` と `quality.jsonl` が生結果。
既存bag/TF/Viewerの負荷を含む環境のため、少数反復の微小な時間差は改善の根拠にしない。

## 通常ビルドと回帰試験の実行記録

standalone比較とは別に、通常のais_gngビルドと曲面97件・平面74件の試験を実行し成功。
実行中ROSの停止・再起動なし。ビルド・単体試験のプロセスは終了済み。

```bash
docker exec gng_cpu_container bash -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
timeout 600 cmake --build /ros2_ws/build/ais_gng --parallel 2
'
docker exec gng_cpu_container bash -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
timeout 180 /ros2_ws/build/ais_gng/test_surface_model --gtest_color=no \
  --gtest_output=xml:/ros2_ws/src/artifacts/surface_priority_20260929/unit_97.xml
timeout 180 /ros2_ws/build/ais_gng/test_plane_cluster_incremental --gtest_color=no \
  --gtest_output=xml:/ros2_ws/src/artifacts/surface_priority_20260929/plane_verified.xml
'
```

ROS setupは未定義環境変数を参照するため、上記sourceの実行時は `set -u` を使用しない。
