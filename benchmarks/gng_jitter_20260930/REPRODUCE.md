# GNG時間変動の再現手順（2026-09-30）

成果は `artifacts/gng_jitter_20260930/live/` と `profile/` に保管。実行済み各試験の完全な起動引数は `live/batch_*/001_*/command.json`、外側のバッチ起動コマンドは `live/run_*_command.txt`。本番ソース・build・installへの変更なし。計測版は `/tmp` のみの共有ライブラリ。

## 前提と保存範囲

- 実workspace: `/home/uraki/uraki_ws`。Docker: `gng_cpu_container`、ROS 2 Humble。ホストworkspaceのコンテナ内位置は `/ros2_ws/src`。
- 入力は既存の `ros2 bag play /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2 --loop --read-ahead-queue-size 20`、topic `/lidar_points`、ROS_DOMAIN_ID=0。既存 `intersection_tf.launch.py`・Viewerも稼働した状態。今回の試験担当によるbag・TF・Viewerの起動／停止なし。
- 同じGNG launchを別途稼働させていない時間帯での計測。既存プロセスを自動停止する処理なし。
- CPUはIntel Core i7-14650HX。論理CPU 0–15がPコア、16–23がEコア。PコアのSMT兄弟は0–1、2–3、…、14–15。計測observerはlaunch生成後にCPU 23へ固定。
- `powersave`、`intel_pstate`、`no_turbo=1`。設定上限はP 2.2 GHz／E 1.6 GHz。DockerのCPU quotaなし、cpuset 0–23。機種・番号が違う場合は `probe.py` のobserver番号、`cases_cores.json` の4／18、解析のP/E分類の変更が必要。
- OMP/GOMP系の既存環境変数がない状態。PASSIVEケースでは `OMP_WAIT_POLICY=PASSIVE KMP_BLOCKTIME=0` のみ追加。`GOMP_SPINCOUNT`の指定なし。
- 各試験は既存bagのその時点から受信。開始位置・フレーム内容を揃えた決定論的再生ではなく、同じbagのライブ入力による比較。
- 保管対象は原launchログ、段階別CSV、OS採取原データ、解析結果、ケース・計測・解析スクリプト、ソースsnapshot、ビルド記録。`system_samples.jsonl`はgzip圧縮。`live/archive_manifest.json`に元／保存ファイルのSHA-256一覧。
- バイナリ、object、CMake buildディレクトリ、Python cache、外部rosbag本体、Dockerイメージ、既存ROS build/install全体は省略。計測ライブラリのsource snapshotは `profile/profile_source.tar.gz`。既存ROS構成は別途必要。

## `/tmp`への復元とビルド

以下はホストで実行。元の診断用 `/tmp` ディレクトリが存在しない環境での復元例。

```bash
mkdir -p /tmp/gng_jitter_root_20260930 /tmp/gng_jitter_20260930
cp -a /home/uraki/uraki_ws/artifacts/gng_jitter_20260930/live/. /tmp/gng_jitter_root_20260930/
cp -a /home/uraki/uraki_ws/artifacts/gng_jitter_20260930/profile/. /tmp/gng_jitter_20260930/
tar -xzf /tmp/gng_jitter_20260930/profile_source.tar.gz -C /tmp/gng_jitter_20260930

docker exec gng_cpu_container mkdir -p /tmp/gng_jitter_root_20260930 /tmp/gng_jitter_20260930
docker cp /tmp/gng_jitter_root_20260930/. gng_cpu_container:/tmp/gng_jitter_root_20260930/
docker cp /tmp/gng_jitter_20260930/profile_source gng_cpu_container:/tmp/gng_jitter_20260930/profile_source
```

snapshotは計測挿入済み。`instrument.py`の再適用は不要。同スクリプトは未加工の同版gng_cpuソースへ計測を挿入するための記録。

ビルド時に実行したコマンド:

```bash
docker exec gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash; cmake -S /tmp/gng_jitter_20260930/profile_source -B /tmp/gng_jitter_20260930/profile_build -DCMAKE_BUILD_TYPE=Release -DGNG_VERSION=0 -DGNG_ENABLE_FRAME_LOG=ON -DGNG_ENABLE_AUTHENTICATION=OFF -DGNG_BUILD_BENCHMARKS=OFF -Dallow_external_sampler=ON -Denable_voxel_framework=OFF -Denable_voxel_fuzzy=OFF -Denable_voxel_history=OFF > /tmp/gng_jitter_20260930/configure.log 2>&1 && cmake --build /tmp/gng_jitter_20260930/profile_build --target gng_cpu -j2 > /tmp/gng_jitter_20260930/build.log 2>&1'
```

Release・LTO・FRAME_LOG等は本番と同じ構成。公開シンボル29件の一致を確認済み（`profile/profile_validation.json`）。本番へinstallする工程なし。`LD_LIBRARY_PATH`先頭への追加により `libgng_cpu.so` のみ切替。実際のロード先は各試験の `gng_maps.txt` に記録。

## 実行済みの全試験起動コマンド

Docker内で、以下の環境読込後に順番に実行したもの。バッチrunnerはworkspaceの `skills/run-benchmark-batch/scripts/run_batch.py`。同内容を `live/run_batch.py` にも保管。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ROS_DOMAIN_ID=0 timeout -s INT -k 25 240 python3 /ros2_ws/src/skills/run-benchmark-batch/scripts/run_batch.py /tmp/gng_jitter_root_20260930/cases_unbound.json --output /tmp/gng_jitter_root_20260930/batch_unbound --repeats 1 --timeout-sec 215 --max-total-sec 220 --estimate-sec 185

ROS_DOMAIN_ID=0 timeout -s INT -k 25 310 python3 /ros2_ws/src/skills/run-benchmark-batch/scripts/run_batch.py /tmp/gng_jitter_root_20260930/cases_wait_policy.json --output /tmp/gng_jitter_root_20260930/batch_wait_policy --repeats 1 --timeout-sec 145 --max-total-sec 290 --estimate-sec 123

ROS_DOMAIN_ID=0 timeout -s INT -k 25 185 python3 /ros2_ws/src/skills/run-benchmark-batch/scripts/run_batch.py /tmp/gng_jitter_root_20260930/cases_cores.json --output /tmp/gng_jitter_root_20260930/batch_cores --repeats 1 --timeout-sec 85 --max-total-sec 170 --estimate-sec 63
```

原結果を復元した環境で再実行する場合は、`--output`のみ `batch_unbound_rerun`、`batch_wait_policy_rerun`、`batch_cores_rerun`など未使用ディレクトリへ変更。runnerの同版再利用には `/tmp/gng_jitter_root_20260930/run_batch.py` も使用可能。

各ケースでrunnerから起動したコマンド（`CASE_DIR`はrunnerが作成した `001_<ケース名>` の絶対パス）:

```bash
python3 /tmp/gng_jitter_root_20260930/probe.py --seconds 180 --lib-dir /tmp/gng_jitter_20260930/profile_build --output CASE_DIR
python3 /tmp/gng_jitter_root_20260930/probe.py --seconds 120 --lib-dir /tmp/gng_jitter_20260930/profile_build --sample-sec .05 --output CASE_DIR
python3 /tmp/gng_jitter_root_20260930/probe.py --seconds 120 --lib-dir /tmp/gng_jitter_20260930/profile_build --sample-sec .05 --output CASE_DIR --wait-policy PASSIVE
python3 /tmp/gng_jitter_root_20260930/probe.py --seconds 60 --lib-dir /tmp/gng_jitter_20260930/profile_build --sample-sec .1 --wait-policy PASSIVE --affinity 4 --enable-main-affinity --output CASE_DIR
python3 /tmp/gng_jitter_root_20260930/probe.py --seconds 60 --lib-dir /tmp/gng_jitter_20260930/profile_build --sample-sec .1 --wait-policy PASSIVE --affinity 18 --enable-main-affinity --output CASE_DIR
```

`probe.py`が起動したROSコマンドは全5ケース共通:

```bash
GNG_LIB_PROFILE_CSV=CASE_DIR/stages.csv \
LD_LIBRARY_PATH=/tmp/gng_jitter_20260930/profile_build:$LD_LIBRARY_PATH \
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml input_topic:=/lidar_points
```

core比較の固定対象はGNGのmainスレッドのみ。launch後10秒経過時に `os.sched_setaffinity(main_tid, {4})` または `{18}` を適用。他のTorch／DDSスレッドへの固定なし。全プロセスを `taskset` で起動する比較とは条件が異なる。frame 150以降を集計し、core比較では全計測点の配置一致を確認。

試験時間終了後、probeが自分のlaunchプロセスグループへSIGINT、必要時のみSIGTERM／SIGKILL。CSVのstdio buffer確定には正常終了が必要。各試験の `cleanup.json`、バッチの `report.json`／`events.jsonl` に終了結果を保存。今回の5試験とコンパイルは終了済み。

## 保存データの展開と解析

ホストに復元した原結果のOSデータを展開:

```bash
python3 - <<'PY'
from pathlib import Path
import gzip
import shutil
for path in Path('/tmp/gng_jitter_root_20260930').glob('batch_*/001_*/system_samples.jsonl.gz'):
    target = path.with_suffix('')
    if not target.exists():
        with gzip.open(path, 'rb') as source, target.open('wb') as output:
            shutil.copyfileobj(source, output)
PY
```

解析コマンド:

```bash
python3 /tmp/gng_jitter_root_20260930/analyze.py /tmp/gng_jitter_root_20260930/batch_unbound/001_unbound --min-frame 150
python3 /tmp/gng_jitter_root_20260930/analyze.py /tmp/gng_jitter_root_20260930/batch_wait_policy/001_fine_default --min-frame 150
python3 /tmp/gng_jitter_root_20260930/analyze.py /tmp/gng_jitter_root_20260930/batch_wait_policy/001_fine_passive --min-frame 150
python3 /tmp/gng_jitter_root_20260930/analyze.py /tmp/gng_jitter_root_20260930/batch_cores/001_p_core --min-frame 150
python3 /tmp/gng_jitter_root_20260930/analyze.py /tmp/gng_jitter_root_20260930/batch_cores/001_e_core --min-frame 150
python3 /tmp/gng_jitter_root_20260930/analyze_threads.py /tmp/gng_jitter_root_20260930/batch_wait_policy/001_fine_default --min-frame 150
python3 /tmp/gng_jitter_root_20260930/analyze_threads.py /tmp/gng_jitter_root_20260930/batch_wait_policy/001_fine_passive --min-frame 150
```

`summary.json`／`report.txt`に段階別分布、周期候補、ログ対応、slow eventsを出力。`thread_analysis.json`にthread消費とslow frameの前後観測を出力。解析スクリプトは解析先の結果ファイルを更新するため、原保管先ではなく復元先での実行。

## 列と計測範囲

`stages.csv`は各frameにentry／voxel／registration／learn／label／check／cluster／finish／totalの9行。mainスレッドのみの段階計測。

| 列・記録 | 意味 |
| --- | --- |
| `frame`, `thread_id`, `stage` | GNG内部frame、実行thread、段階名 |
| `start_wall_ns`, `end_wall_ns`, `wall_ms` | CLOCK_MONOTONICの開始・終了・差分 |
| `cpu_ms` | CLOCK_THREAD_CPUTIME_ID差分。並列worker CPUの合算なし |
| `voluntary_switches`, `involuntary_switches` | RUSAGE_THREADの任意／強制context switch差分 |
| `minor_faults`, `major_faults` | RUSAGE_THREADのpage fault差分 |
| `start_cpu`, `end_cpu` | 各計測点のsched_getcpu値。区間内の往復移動の完全記録なし |
| `num_input_points`, `num_voxels`, `num_attention_points` | 点数・voxel数・attention対象数 |
| `num_nodes`, `num_clusters` | 処理後のノード数・クラスタ数 |
| `num_checked_cells`, `num_added_nodes`, `num_aged_nodes`, `num_removed_nodes` | 既存挿入統計の値。全種類のノード削除の合算とは別 |
| `system_samples.jsonl` | monotonic時刻、main／各threadのproc stat、全CPUのproc stat、scaling_cur_freq |

`total`は初期化済み確認後・beginMapDeltaFrame前から、finishMapDeltaFrameおよびlog.screen後まで。登録段階はbegin_update_frame、未観測寿命処理、attention／近傍登録を含む。API呼出元のstdout capture準備、取得結果展開、GNG後のTorch分類・平面・曲面・描画は区間外。初回ファイルopenとCSV書込も区間外。1 MiB buffer、毎frameのflushなし。段階の採取自体にはclock/getrusage等の小さな追加負荷あり。

proc statのCPU時間は100 Hz（10 ms）単位。thread解析は観測区間の消費をframe重複長で按分する概算。processorは最終CPU番号であり、SMT兄弟での厳密な同時実行・因果を保証しない。`sample_coverage_ms=0` は未観測でありworker不在ではない。非mainスレッドにはDDS等も含む。段階start/end CPU一致だけで途中の移動を排除できない。周期の相関・位相平均差だけによる固定周期の断定も不可。
