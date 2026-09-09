# 2026-09-09 - 平面クラスタの分割・削除不要時の処理省略

## Summary

判定条件や更新頻度を変えず、分割・削除のないフレームの不要処理を省略。
合成グラフの定常・併合ケースで平均処理時間を約7〜12%短縮。

## Changed

- `splitClusters`: 接続切断の確認カウンタ更新後、確定分割がなければ早期終了。
- `cullClusters`: 生存・確認カウンタ更新後、削除対象がなければ早期終了。
- 分割・削除が必要なフレームは既存処理を継続。

## Added

- 切断確認待ち、再接続時の待ち状態解除、再切断後の分割・再併合を確認するテスト。
- 削除不要状態の後に先頭クラスタを削除し、残存クラスタの所属添字とID維持を確認するテスト。
- [ラウンド別計測結果](2026-09-09_plane_cluster_fastpath.csv)。

## Fixed

分割のないフレームにおける全ノードの旧ID投票、削除のないフレームにおけるクラスタコピー・全ノード所属再採番の重複処理。

## Removed

検出機能・判定項目の削除なし。

## Behavior Impact

- 前回の連鎖併合修正、ノードの誤差共分散を必須にしない方式を維持。
- ノードの間引き・処理周期の間引き・判定閾値の緩和なし。
- GNG学習本体、ROSメッセージ、viewerへの変更なし。

## Topics / Params / Messages

変更なし。

## Verification

### 回帰テスト

最適化直前・最適化後の双方で`test_plane_cluster_incremental`の15件すべて合格。
追加2件を含み、前回の鎖状併合・厚み判定・誤吸収防止の試験も維持。

### 性能比較

- 比較元は前回の連鎖併合修正済みソース。SHA256: `ad5b842be59b3e971e86f9547f8c0748a0d3d5dae83808edfe8639f90f375042`。
- 最適化後SHA256: `6854b741564507998e2fb74b761cc4f38597ca1641a27c361f981017a06859d8`。
- AMD Ryzen 7 260、Docker `gng_cpu_container`、g++ 11.4.0、`-O3 -DNDEBUG`、CPU4固定。
- 同じベンチマークを500回 x 3ラウンド。最適化前後を交互に実行し、2ラウンド目のみ順序反転。
- 入力生成・初期化を除いた`Clusterizer::update()`全体の壁時計時間。GNG学習・ROS通信・描画は対象外。
- ベンチマーク・共通設定の詳細は[前回の検証記録](2026-09-09_plane_cluster_timing.md)と同じ。
- 30組の比較で、出力したノード数・エッジ数・クラスタ数・所属ノード数・併合数・候補対数・面内広がり比・チェックサムが一致。
- 出力統計の一致は浮動小数点表示6桁までの確認であり、全メッセージのビット単位比較ではない。

平均は各条件1500回分。P95欄は各ラウンドのP95の最大値であり、全1500回のP95ではない。

| ケース | ノード数 | 最適化前平均 ms | 最適化後平均 ms | 短縮率 | 最適化後P95の最大 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| 定常状態 | 2760 | 0.255 | 0.235 | 7.8% | 0.318 |
| 定常状態 | 11040 | 1.058 | 0.968 | 8.5% | 1.365 |
| 初回検出 | 2760 | 0.205 | 0.204 | 0.5% | 0.313 |
| 初回検出 | 11040 | 0.971 | 0.910 | 6.2% | 1.568 |
| 面状パッチの併合 | 2916 | 0.431 | 0.400 | 7.1% | 0.587 |
| 面状パッチの併合 | 10404 | 1.328 | 1.217 | 8.3% | 1.800 |
| 段差による併合拒否 | 2916 | 0.334 | 0.304 | 9.1% | 0.394 |
| 段差による併合拒否 | 10404 | 1.275 | 1.127 | 11.7% | 1.462 |
| 鎖状パッチの併合 | 2916 | 0.344 | 0.310 | 10.0% | 0.410 |
| 鎖状パッチの併合 | 10404 | 1.310 | 1.174 | 10.3% | 1.590 |

初回検出の小規模ケースはほぼ同じ。今回の省略対象である分割ID投票は、初回には旧所属がないため元から軽い処理。
単発最大は最適化後4.586ms。非専用CPU環境のため、最大時間が改善する保証はない。

### 実行コマンド

一時ディレクトリに最適化直前ソースを配置後、両版のオブジェクト・テスト・ベンチマークを順次ビルド。
既存インストール・共有ビルド先への書き込みなし。

```bash
docker exec gng_cpu_container bash -lc '
set -e
bench_dir=/tmp/plane-fastpath-bench.CXvNDm
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
    -c "$source" -o "$bench_dir/$version.o"
  timeout --kill-after=5s 120s g++ -std=c++17 -O3 -DNDEBUG "${includes[@]}" \
    "$bench_dir/$version.o" /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/test_plane_cluster_incremental.cpp \
    -lgtest_main -lgtest -pthread -o "$bench_dir/test_$version"
  timeout 30s "$bench_dir/test_$version"
  timeout --kill-after=5s 120s g++ -std=c++17 -O3 -DNDEBUG "${includes[@]}" \
    "$bench_dir/$version.o" /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/benchmark_plane_cluster_incremental.cpp \
    -o "$bench_dir/$version"
done
'

docker exec gng_cpu_container timeout --kill-after=5s 240s bash -lc '
set -euo pipefail
bench_dir=/tmp/plane-fastpath-bench.CXvNDm
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

ビルド・テスト・ベンチマークのプロセスは終了済み。一時ディレクトリと比較用ソースは削除済み。ROSノードの新規起動なし。

## Risk / Notes

- 合成グラフでの測定。実環境グラフの連続入力・ROS全体の処理時間は未検証。
- 既存の他プロセスは停止せず。過去の別測定との絶対値比較ではなく、この測定の対になった実行を比較。
- 少数%の差を厳密な統計的有意差とは断定しない。
- 稼働中ノードへのインストール・再起動は未実施。反映にはais_gngの再ビルドと対象ノードの再起動が必要。
