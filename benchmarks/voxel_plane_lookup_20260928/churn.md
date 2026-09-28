# 入力支持とノード追加削除の比較（2026-09-28）

## 要約

占有更新が有効なとき、入力代表点を既存警戒領域内で受け持つ最近傍ノードの通常寿命を更新。
既存の最近傍ID・距離を再利用。新たな距離・点数条件、追加上限、空間探索なし。
同じ占有セル内の全ノードを保護する処理ではない。早期削除の3入力条件は維持。
GNGの通常エイジングとの競合による追加削除を抑制。実物体の追従精度改善は未検証。

診断は専用ビルドのみ。通常のcugngオブジェクトにgng_churnシンボルがないことをnmで確認。
通常gng_cpuをビルド・install反映。既存ROSの停止・再起動なし。

## 条件・検証

Macnica保存済み先頭60入力、20入力をウォームアップとして除外、条件ごと3試行。
ノード上限20,000、入力上限200,000点、voxel 0.5 m、学習4,000回、CPU 4固定。
実GNG・平面クラスタリングを直接呼出し。現行の平面ID再利用修正を含む両条件の比較。
ROS通信・人車分類・本番の重点サンプラー設定を含まない。既存ROSとのCPU競合あり。
点群代表は両条件とも実測点。旧重心方式との比較ではない。

| 指標（1入力あたり平均） | 修正前 | 支持寿命更新 |
| --- | ---: | ---: |
| 全追加 [ノード] | 1036.008 | 832.775 |
| 全削除 [ノード] | 1038.708 | 833.467 |
| 削除セルへの再追加 [ノード] | 285.200 | 203.692 |
| 通常寿命削除 [ノード] | 779.075 | 612.483 |
| 通常寿命削除のうち警戒領域内の最近傍支持あり [ノード] | 46.008 | 0 |
| 平均保持ノード数 | 19220.925 | 19387.517 |
| GNG [ms] | 49.920 | 49.745 |
| 平面処理 [ms] | 6.714 | 6.623 |

追加19.6%減、再追加28.6%減。速度はほぼ横ばいで高速化の根拠には不十分。
時間は診断集計を含み、本番処理時間ではない。形状被覆・移動体の追従品質は別途評価が必要。
再追加は当該入力と直前3入力の削除セルへの追加。セル一致であり、同一物体の誤削除の断定ではない。
全追加は入力処理・学習の両方。削除記録は実際の削除位置、同じセルへの複数追加も各1回。
初回診断では通常方式1052.742、占有方式1040.692ノード/入力の追加。
したがって、占有方式だけが大量追加を発生させたという仮説は支持されない。

最終比較6/6成功、予測24秒・実測21.91秒、全試行cleanup成功。
製品CTest 23/23、通常26/26成功。OFF時、同セルの非最近傍、警戒領域外の非保護を回帰確認。
診断初回ビルドはVec3fのconst呼出しで失敗、試験用コピーへ訂正後に成功。
全ビルド・runner・試験終了。通常installに診断機能は非搭載。

再現コマンドはdocker exec gng_cpu_container bash -lc内、ROS環境読込み後の/ros2_ws/srcで実行：

```bash
timeout 40 cmake -S benchmarks/voxel_plane_lookup_20260928 -B artifacts/voxel_plane_lookup_20260928/churn_build -DCMAKE_BUILD_TYPE=Release -Denable_churn_diagnostics=ON
timeout 240 cmake --build artifacts/voxel_plane_lookup_20260928/churn_build -j2
PYTHONDONTWRITEBYTECODE=1 python3 skills/run-benchmark-batch/scripts/run_batch.py benchmarks/voxel_plane_lookup_20260928/support_cases.json --output artifacts/voxel_plane_lookup_20260928/support_final_batch --repeats 3 --timeout-sec 90 --max-total-sec 540 --estimate-sec 4
timeout 240 cmake --build /ros2_ws/build/gng_cpu -j2
timeout 60 ctest --test-dir /ros2_ws/build/gng_cpu --output-on-failure
timeout 60 cmake --install /ros2_ws/build/gng_cpu
```

保存済み入力・reference.hppは既存prepare.pyによる準備が前提。再試行時は新しい保存先を指定。
初回診断はchurn_cases.jsonとchurn_batch、試作比較はsupport_batch、最終結果はsupport_final_batch。
各report.jsonにargv・種・時間・終了状態、各churn_frames.csvに入力ごとの計数を保存。
製品試験はartifact内productでビルド後、LD_LIBRARY_PATH先頭をproductとしてctestを実行。
support_product_final_test.log、support_normal_test.logに最終テスト結果を保存。
