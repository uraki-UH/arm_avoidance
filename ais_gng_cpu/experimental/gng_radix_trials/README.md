# 本番CPU相当の基数ソート比較版

基数ソート採用前の最適化済み本番 `ais_gng_cpu/src/gng_cpu` からの独立コピー。全入力の近傍照合、重点学習、観測・統計、ノード／エッジ更新、クラスタ処理を維持。`COLCON_IGNORE`付きで通常のcolcon探索から除外。比較用コピー自体の本番への差替えなし。

## 比較対象

- `GNG_RADIX_VOXELS=OFF`: 採用前の本番と同じBoostの整数ソート。
- `GNG_RADIX_VOXELS=ON`（このコピーの既定）: 32bitセル番号を8bitずつ4回処理する安定基数ソート。点群上限と同じ長さの作業バッファを再利用。
- `GNG_DETERMINISTIC_BENCHMARK=ON`（既定）: 乱数シードを `trial_seed + frame_number`（既定シード20260924）、LPFの時間刻みを0.1秒へ固定。

セル番号の量子化方法、元点の採否、重心の計算式、学習回数の変更なし。ソート順の変更により、重心の浮動小数点加算順に加え、重点候補配列・観測代表点の元番号も変化し得る。固定シードの出力完全一致は保証しない。

## 検証

```bash
bash /ros2_ws/src/benchmarks/gng_radix_production_20260924/build.sh
bash /ros2_ws/src/benchmarks/gng_radix_production_20260924/run.sh
python3 /ros2_ws/src/benchmarks/gng_radix_production_20260924/summarize.py
```

Release・LTO、同じ全入力・同じ学習4,000回で比較。ソート時間・GNG段階時間、全点のセル対応、重心、実際の学習点、グラフ／クラスタ、原点を除いた入力点群の被覆を記録。原点の除外は品質評価だけで、学習入力からの除外なし。

[計測条件と結果](../../../benchmarks/gng_radix_production_20260924/README.md)を参照。計測用APIと学習点記録機能はこのコピー内だけに存在。

## 複数シードの追加検証

比較専用API `gng_set_trial_seed` と `gng_set_trial_last_representative` を追加。既定値は従来のシードとセル先頭代表点。代表点切替はグラフへの影響を切り分ける対照条件で、本番機能ではない。

[6シードの比較と採用判断](../../../benchmarks/gng_radix_multiseed_20260924/README.md)に基づき、本番CPUには入力ソート部分だけを反映。計測・固定シード・固定時刻・代表点切替は引き続きこのコピー内に限定。
