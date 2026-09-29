# 曲面の局所候補探索・固定入力比較（2026-09-29）

## 要約

最終候補は `local_flat`。比較旧版・候補版のソース、固定入力、48試行のログ、品質照合結果を
[artifacts/surface_local_20260929/perf](../../artifacts/surface_local_20260929/perf/) に保存。
性能値の正本は同ディレクトリの `summary.csv` / `summary.json`、条件の詳細は `input_provenance.json`。
`commands.json` は実行済みコマンド、各 `batch_*/*/command.json` は個々の計測起動記録。
本手順は当時のsnapshotを再構築。現在のインストール済み曲面ライブラリへのリンクなし。
採用結果・互換性は[リリース記録](../../gng_vlut_system/docs/releases/2026-09-29_surface_local_search.md)、
本番ビルド・ROS試験を含む起動記録は[commands.md](../../artifacts/surface_local_20260929/commands.md)。

## 条件・検証

- 対象は `tracker.update()` のCPU時間と経過時間。入力読込・CBOR/JSON処理・平面生成・ROS通信・Marker生成・描画は計測外。
- `input.cbor.gz`：指定bag `/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2` の `/lidar_points` 由来。
  保存済み `artifacts/plane_consistency_20260924/frames.bin` の150入力へsnapshot平面処理を適用し、先頭50入力を除いた100入力のMap+Planeを固定。
  平均19,143.9ノード。旧保存形式に `boundary_evidence` がなく0。報告された実運転60 msの直接再現ではない。
- `small.cbor.gz`：`tmp/surface_incremental_20260915/observed.json` 由来、1,545ノード・10平面・21入力。
- `large_curve.cbor.gz`：円柱288点・8平面と、離れた単一平面19,000点の合成、20入力。
- 入力変換は `prepare.cpp` / `convert_small.py` / `make_large_curve.py`。再生成には上記元データとPython `cbor2` が必要。通常の再計測は保管済みCBORを復元するだけでよい。
- GCC/C++17/`-O3 -DNDEBUG`、CPU固定なし、既存bag・TF・rosbridgeが稼働。各条件3試行の平均・p95の中央値、最大は全試行最大。p95は昇順添字 `floor((N-1)*0.95)`。
- `local` は複数平面または既存追跡を含む成分の候補限定。対象外の診断用パッチ・unknown領域は省略。
  `local_flat` は同条件に16bit IDの直接参照を追加。いずれも計測側で `enable_plane_local_search=true`、必要元平面数2。
- baseline→localは141入力の表示対象曲面、local→local_flatは141入力の全出力が一致。
  候補が大半を占める小規模入力は約7.6 msであり、任意入力で3 ms以内を保証する方式ではない。
- 保管省略：巨大な生の品質JSON（微修正4条件だけで約1.8 GB）、実行バイナリ、object、計測外の品質生成時タイミング。
  照合結果JSON・ソース・固定入力・正式試行の全タイミングは保管。曲率V1の全品質照合は中断・未完了で非採用。
- 全計測プロセス終了を `cleanup.json` に記録。作業中に外部で起動されたfrontendは停止せず維持。

### 固定入力の復元とビルド

既存の `gng_cpu_container` と `/ros2_ws/install/ais_gng_msgs`、ROS Humbleのヘッダ、Eigen3、nlohmann-json、g++が必要。
スクリプト・manifestは **`/tmp/surface_perf_20260929` の絶対パスに依存**。別パスへ移す場合は各 `.sh` / `.py` / `cases*.json` の対応箇所を変更。
以下はホストから実行。コンテナの起動・停止や既存ROSノードの変更なし。

```bash
docker exec gng_cpu_container bash -c '
set -euo pipefail
trial_root=/tmp/surface_perf_20260929
trial_saved=/ros2_ws/src/artifacts/surface_local_20260929/perf
mkdir -p "$trial_root"
cp -a "$trial_saved"/. "$trial_root"/
for trial_input in input small large_curve; do
  gzip -dc "$trial_saved/inputs/$trial_input.cbor.gz" > "$trial_root/$trial_input.cbor"
done
timeout 240 bash "$trial_root/build.sh" baseline
timeout 210 bash "$trial_root/build_local.sh"
timeout 150 bash "$trial_root/build_flat.sh"
'
```

### 交互3反復の再計測

入力は事前にメモリへ読み込み、JSON保存は時間計測後。下記の出力先は新規必須、再実行時は別名へ変更。
各runnerの `report.json` に失敗・終了状態、条件別 `metrics.json` に集計、`frames.jsonl` に各入力の内訳。

```bash
docker exec gng_cpu_container bash -c '
set -euo pipefail
trial_root=/tmp/surface_perf_20260929
trial_runner=/ros2_ws/src/skills/run-benchmark-batch/scripts/run_batch.py
timeout 240 python3 "$trial_runner" "$trial_root/cases_local.json" --output "$trial_root/recheck_local" --repeats 3 --timeout-sec 40 --max-total-sec 220 --estimate-sec 4
timeout 200 python3 "$trial_runner" "$trial_root/cases_flat.json" --output "$trial_root/recheck_flat" --repeats 3 --timeout-sec 35 --max-total-sec 180 --estimate-sec 3
'
```

`summarize.py` は保存済み `batch_v1` / `batch_local` / `batch_flat` を集計するスクリプト。
再計測の出力を集計する場合は同スクリプト先頭の対象ディレクトリ名を変更。既存試行への上書きは不要。

### 品質の再照合

以下は全141入力でbaseline→localの表示曲面と、local→local_flatの全出力を再照合。
生の品質JSONを生成するため追加ディスク容量が必要。品質生成時の `*_quality_timing.jsonl` は性能比較に使用しない。
`compare*.py` の終了コードだけでなく結果JSONの **`is_equal: true`** を確認。

```bash
docker exec gng_cpu_container bash -c '
set -euo pipefail
trial_root=/tmp/surface_perf_20260929
for trial_input in input small large_curve; do
  for trial_variant in baseline local local_flat; do
    timeout 60 "$trial_root/$trial_variant/measure" "$trial_root/$trial_input.cbor" "$trial_root/${trial_variant}_${trial_input}_quality_timing.jsonl" tracker 100 "$trial_root/${trial_variant}_${trial_input}_quality.jsonl"
  done
  timeout 120 python3 "$trial_root/compare_display.py" "$trial_root/baseline_${trial_input}_quality.jsonl" "$trial_root/local_${trial_input}_quality.jsonl" "$trial_root/recheck_${trial_input}_display.json"
  timeout 120 python3 "$trial_root/compare.py" "$trial_root/local_${trial_input}_quality.jsonl" "$trial_root/local_flat_${trial_input}_quality.jsonl" "$trial_root/recheck_${trial_input}_full.json"
done
python3 "$trial_root/cleanup.py"
'
```

`measure` の100は上限であり、smallは21入力、large_curveは20入力を使用。
圧縮入力の容量・SHA256は `inputs/manifest.json`、ソースのSHA256は `source_sha256.json`。
