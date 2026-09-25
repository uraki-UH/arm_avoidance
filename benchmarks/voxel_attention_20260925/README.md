# ボクセル重点学習の検証（2026-09-25）

本機能は同日のユーザー指示で撤去済み。以下は削除前の評価記録で、現行機能の説明ではない。
旧専用ベンチ・スモーク・比較スクリプトはソースとともに`artifacts/nonplane_attention_removal_20260925/source_before.tar.gz`へ退避。
以下の旧コマンドは削除前の環境用。現在の確認手順は[削除後検証](../nonplane_attention_20260925/REMOVAL.md)を参照。

## 要約

非平面・新規占有・表現不足のセルスコア抽選を検証。追加したオクルージョン判定はユーザー指示で撤去。
同日、整列済みセルでの履歴照合と、学習・出力共通の部分和へ変更。全候補点への乱数・対数・順位付けを撤去。
抽選コストは減少したが、重点化OFFより速くなる結果や、移動物体の再現性改善は未確認。
評価当時の仕様・設定・未対応範囲：[ボクセル重点学習](../../ais_gng_cpu/docs/voxel_attention.md)。
CPUの単体・API回帰22対象に成功。点密度独立性・履歴失効・混合比率・元添字、背景999セルでも非平面群80%を確認。
ROSは実bag12フレーム×6条件に成功。通常、成分サイズ不足、成分出力OFF、デバッグOFF、機能OFF、平面OFFを比較。
最新の出力は20,000点。元点・支持セル集合・重複元点の水増しなしを照合し、内部重点学習は2,000回の維持を確認。
群別正規化で非平面への学習配分を約80%へ修正。出力は候補十分時16,000/20,000点、不足分は探索群へ返却。
これは重点枠内の割合であり全4,000回中の80%ではない。高速化後も確率分布・点数・元番号の非重複を維持、同じ乱数と点列の保証なし。
時刻逆行・時間切れ・座標名変更でも現在入力の重点点を出力。PublisherのOFF条件・空でないTmapを確認。

## 条件・検証

- 実入力：`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の先頭12件。
- 入力上限100,000点、voxel 0.5 m、学習4,000回、ノード上限20,000、at128＋共通平面設定。
- 再生試験はlocal座標、有効期間5秒。人車分類OFF、重点学習専用の原点設定なし。
- Viewer目視、長時間の残像減少率、人車の検出率は未評価。動的TFの正確性・スキャン内歪みは保証外。
- 出力元点・重複数・支持セル・非平面群への点数配分を照合。出力用重点点群は実学習点列ではない。
- 試験専用ROSドメインを使用。Pythonは子プロセスグループを記録し、finallyでSIGINT→TERM→KILLの上限付き停止。

全体比較は`compare.py`を使用。実bagは60フレーム、初回20件除外後の平均を、全条件完了の2試行で平均。CPU 0固定。
合成は床80,000点＋移動楕円体20,000点、0.08 m/フレーム移動、40フレーム中初回10件除外、各平均の3試行中央値。CPU 2固定。
全体時間はノード内processing（入力・GNG・変換・平面・出力処理）。DDS到着待ち・評価用Python・Viewer描画は含まない。

| 条件 | 実bag全体[ms] | 合成全体[ms] | 合成物体の距離p90[m] | 合成物体の0.2 m被覆率[%] |
| --- | ---: | ---: | ---: | ---: |
| 変更前OFF | 58.130 | 11.700 | 0.154 | 96.89 |
| 変更前ON＋20,000点出力 | 65.569 | 18.365 | 0.226 | 85.58 |
| 変更後ON＋20,000点出力 | 63.803 | 16.783 | 0.218 | 87.53 |
| 変更後ON・出力なし | 59.822 | 13.854 | 0.222 | 86.66 |
| 変更後OFF | 56.914 | 12.867 | 0.146 | 98.06 |

被覆率は既知の移動表面から5,000点を取り、最近傍GNGノードまでの距離で評価。対象識別や追跡器の正解率ではない。
実bagの参考領域は初回OFFの非平面セル由来でOFF側の選択バイアスあり。全点距離は範囲外入力も含むため、品質改善の結論には不使用。
OFFは旧unknown重点学習。`node.unknown_learning_rate: 0.3`は受付範囲外で警告、既定5（候補あり時80%）を使用。今回この既存設定仕様は変更なし。
本番乱数は非固定。背景負荷・グラフ差による変動あり。高速化自体の効果と、重点化の採用価値を区別。

コンテナ内`/ros2_ws`での通常ビルド：

```bash
source /ros2_ws/install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 timeout -s INT -k 20 480 colcon build \
  --packages-select gng_cpu ais_gng --executor sequential --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

コンテナ内`/ros2_ws/src`での起動・再検証：

```bash
ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 180 \
  python3 benchmarks/voxel_attention_20260925/smoke.py \
  --executable /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3
```

全体比較の起動コマンド（変更前の2ライブラリを`optimization_before/`へ保存済み）：

```bash
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 600 \
  python3 benchmarks/voxel_attention_20260925/compare.py \
  --before /ros2_ws/src/artifacts/voxel_attention_20260925/optimization_before \
  --after /ros2_ws/src/artifacts/nonplane_attention_20260925/install/lib \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --output artifacts/voxel_attention_20260925/compare_real --trials 2
```

合成試験はドメイン177、出力先`compare_synthetic`へ変更し、`--synthetic --cpu 2 --frames 40 --warmup 10 --trials 3`を指定。
初回実bag比較は3試行指定で600秒に到達、最後の条件が未完了。表は最初の2試行だけを使用。中断時の二重shutdownを修正。

隔離検証では実行ファイルを`artifacts/nonplane_attention_20260925/ros_build/ais_gng_cpu`へ変更し、
同ディレクトリと`artifacts/nonplane_attention_20260925/install/lib`を絶対パスで`LD_LIBRARY_PATH`の先頭に設定。
CPU試験は同じ隔離`gng_build`をRelease/LTO・`GNG_BUILD_BENCHMARKS=ON`でビルド後、`ctest --output-on-failure`。
`ais_gng_cpu`は動的読込ラッパーのため、`ais_gng_component_cpu`も明示的なビルド対象。

追加処理比較は同じ実入力100,000点・対象座標6,654点・入力セル13,654個を使用。
既存voxel生成・最近傍照合・GNG学習・ROS配信は測定外。距離残差は事前計算。mode 0は半径、1はセル選択、2は1＋出力用抽選。
CPU 4固定の変更前後交互3試行で、選択のみ3.106→2.254 ms、出力込み7.617→4.049 ms。各9回中初回2回除外の中央値、さらに3試行中央値。
以前のCPU 0測定5.633 msとは負荷条件が異なる。全体速度改善率への換算なし。ログは`performance_optimized_paired.log`。

```bash
timeout 120 g++ -std=c++20 -O3 -DGNG_VERSION=0 \
  -Iais_gng_cpu/src/gng_cpu/include -Iais_gng_cpu/src/gng_cpu/src -Iais_gng_cpu/src/ais_gng/include \
  benchmarks/voxel_attention_20260925/benchmark.cpp \
  ais_gng_cpu/src/gng_cpu/src/cpu/{cugng,voxel_grid}.cpp \
  ais_gng_cpu/src/gng_cpu/src/utils/{node,param,vec3f,utils}.cpp \
  -o artifacts/voxel_attention_20260925/benchmark_optimized
for trial in 1 2 3; do
  timeout 60 taskset -c 4 artifacts/voxel_attention_20260925/benchmark_no_visibility \
    artifacts/nonplane_attention_20260925/selection.bin
  timeout 60 taskset -c 4 artifacts/voxel_attention_20260925/benchmark_optimized \
    artifacts/nonplane_attention_20260925/selection.bin
done
```

ログはGit対象外の`artifacts/voxel_attention_20260925/`。最適化後CPU22対象は`test_optimized_core.log`、隔離ROS6条件は`smoke_optimized.log`で成功。
通常ビルドは`build_optimized_installed.log`、配布先6条件は`smoke_optimized_installed.log`で成功。試験前後のライブラリSHA-256一致。
中断確認は比較コマンドを5秒上限・`--trials 1`・出力先`compare_shutdown`へ変更。子ノード終了と二重shutdown解消を確認。
全試験・ベンチマーク・実行セッションは終了。既存3コンテナ、bag 293745・GNG 305235・Viewer launch 305269を維持。追加デーモン残留なし。
初期のターゲット未生成・const演算・試験ログ参照・所有権・並行ビルドのリンク失敗は修正後に解消。SciPy/NumPy版互換と既存C++警告は継続。
過去の半径方式・判定撤去時の結果は同ディレクトリの旧ログへ保持。撤去判断は[記録](../../gng_vlut_system/docs/reject.md)を参照。
