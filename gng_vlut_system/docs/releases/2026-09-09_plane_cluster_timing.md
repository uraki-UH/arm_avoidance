# 2026-09-09 - 平面クラスタ処理時間の修正前後比較

## Summary

合成グラフを用いたDocker内の比較。修正後の平均は約3千ノードで0.218〜0.485ms、約1万ノードで0.955〜1.416ms。
一律の高速化ではなく、定常状態はほぼ同程度、併合条件・残すクラスタ数によって増減。

## Changed

`benchmark_plane_cluster_incremental`に入力規模・処理局面・試行回数の指定を追加。
既定パラメータは`declareClusterOptions`のCPU直結経路から取得。GNG学習本体・平面クラスタ実装への追加変更なし。

## Added

- `steady <copies> <samples>`: 床・壁・物体面の既存2760ノード入力を複製した定常状態。
- `birth <copies> <samples>`: 空状態からの初回検出。確認待ち3フレームのため当該フレームの出力クラスタ数は0。
- `merge <width> <samples>`: 6x6ノードの平面パッチをwidth x width配置し、生成後に境界を接続。
- `reject <width> <samples>`: 隣接パッチ間に10cmの段差。併合拒否が継続する定常状態。
- `chain <patches> <samples>`: 6x6ノードパッチを一直線に接続。線状化の抑制を伴う一斉併合。
- ラウンド別の[測定結果CSV](2026-09-09_plane_cluster_timing.csv)。全サンプルの生時系列ではなく各500回の集計値。

## Fixed

定常状態だけでは併合処理の修正を評価できなかったベンチマークの計測範囲。

## Removed

なし。

## Behavior Impact

変更は計測プログラムのみ。稼働中ROSノードへの反映・再起動なし。

## Topics / Params / Messages

ROSインターフェースの変更なし。ベンチマークのCLI引数のみ追加。

## Verification

### 測定条件

- CPU: AMD Ryzen 7 260 w/ Radeon 780M Graphics、8コア16スレッド。
- Docker: `gng_cpu_container`、Ubuntu 22.04、g++ 11.4.0。
- ビルド: 両実装とも`-std=c++17 -O3 -DNDEBUG -Wall -Wextra`、LTOなし。
- 実行: `taskset -c 4`で同じ論理CPUへ固定、単一プロセス。
- 各条件500回 x 3ラウンド、各ラウンドの先頭20回は除外。2ラウンド目のみ修正前後の順序を反転。
- 測定対象: `Clusterizer::update()`全体。隣接構築・法線処理・所属保守・生成/分割/併合・出力メッセージ構築を含む。
- 測定対象外: 入力生成、初期化、併合試験用の事前クラスタ生成、戻り値の破棄、GNG学習、ROS通信、viewer描画。
- 他プロセスは停止せず。準備中の`vmstat 1 3`ではCPUアイドル84〜85%。周波数変動・スケジューリング等の影響が残る非専用環境。
- 確認時の`/topological_map` publisher数は0。実環境グラフの連続入力は未測定。

### バージョン

- 修正前: `0226744d840de33552ea68a6c9e6cfe145f0c97e`の`plane_cluster_incremental.cpp`。
- 修正後ソースSHA256: `ad5b842be59b3e971e86f9547f8c0748a0d3d5dae83808edfe8639f90f375042`。
- ベンチマークソースSHA256: `abe5847a3b67c19e168b12b4b17171760789bcd0bd168b248cb4238fe21ee3ba`。
- 旧版は比較専用の一時ファイルからビルド。作業ツリーのロールバックなし。

### 結果

平均は1500回分の平均。P95欄は各500回のP95のうち最大値であり、1500回をまとめたP95ではない。

| ケース | ノード数 | 修正前平均 ms | 修正後平均 ms | 修正後P95の最大 ms | 修正後単発最大 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| 定常状態 | 2760 | 0.266 | 0.267 | 0.352 | 0.573 |
| 定常状態 | 11040 | 1.075 | 1.093 | 1.601 | 2.261 |
| 初回検出 | 2760 | 0.209 | 0.218 | 0.336 | 0.431 |
| 初回検出 | 11040 | 0.919 | 0.955 | 1.713 | 2.623 |
| 面状パッチの一斉併合 | 2916 | 0.444 | 0.485 | 0.873 | 3.525 |
| 面状パッチの一斉併合 | 10404 | 1.468 | 1.389 | 1.935 | 3.075 |
| 段差による併合拒否 | 2916 | 0.356 | 0.351 | 0.478 | 0.567 |
| 段差による併合拒否 | 10404 | 1.349 | 1.366 | 1.937 | 2.678 |
| 鎖状パッチの一斉併合 | 2916 | 0.436 | 0.376 | 0.537 | 0.751 |
| 鎖状パッチの一斉併合 | 10404 | 1.386 | 1.416 | 1.967 | 4.478 |

鎖状2916ノードでは出力クラスタ数が旧1から新27へ、10404ノードでは旧1から新97へ変化。
後者の面内広がり比は旧0.003412から新0.329178となり、下限0.25を満たす分割を保持。
面状パッチの一斉併合は両実装とも1クラスタ。段差ケースは両実装とも81/289クラスタを保持。

### 実行コマンド

比較用ファイル配置後のビルドコマンド。一時ディレクトリは検証終了後に削除済み。

```bash
docker exec gng_cpu_container bash -lc '
set -e
bench_dir=/tmp/plane-merge-timing.D6RcHw
includes=(-I/usr/include/eigen3
  -I/ros2_ws/src/ais_gng_cpu/src/ais_gng/include
  -I/ros2_ws/install/ais_gng_msgs/include/ais_gng_msgs)
for dir in /opt/ros/humble/include/*; do
  if [ -d "$dir" ]; then includes+=("-I$dir"); fi
done
for version in baseline current; do
  source="$bench_dir/baseline.cpp"
  if [ "$version" = current ]; then
    source=/ros2_ws/src/ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp
  fi
  timeout --kill-after=5s 180s g++ -std=c++17 -O3 -DNDEBUG -Wall -Wextra "${includes[@]}" \
    "$source" /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/benchmark_plane_cluster_incremental.cpp \
    -o "$bench_dir/$version"
done
'

docker exec gng_cpu_container timeout --kill-after=5s 240s bash -lc '
set -euo pipefail
bench_dir=/tmp/plane-merge-timing.D6RcHw
for round in 1 2 3; do
  for spec in "steady 1" "steady 4" "birth 1" "birth 4" \
    "merge 9" "merge 17" "reject 9" "reject 17" "chain 81" "chain 289"; do
    versions="baseline current"
    if [ "$round" = 2 ]; then versions="current baseline"; fi
    for version in $versions; do
      taskset -c 4 "$bench_dir/$version" $spec 500 | sed "s/^/round=$round version=$version /"
    done
  done
done | tee "$bench_dir/results.txt"
'
```

確認用ROSノード`plane_merge_bench_discovery`は3秒のグラフ参照後にdestroy/shutdown済み。
ベンチマーク・ビルドの全プロセスは終了済み。一時ソース・バイナリは削除済み。

## Risk / Notes

- 合成グラフの結果であり、実際の点群の欠損・ノイズ・不安定な法線を含む性能保証ではない。
- 同じ入力でも鎖状ケースの出力は修正前後で異なるため、純粋に同じ仕事の高速化比較ではない。
- 数%の差を有意な速度差とは断定しない。定常約3千ノードは平均0.001ms未満の差。
- 50Hzの1周期20msと比較すると、この試験の処理時間は小さい。ただしROS全体の50Hz動作を保証する測定ではない。
- 計測した単発最大は修正後4.478ms。ハードリアルタイムの上限ではない。
