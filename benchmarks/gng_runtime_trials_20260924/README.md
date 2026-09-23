# 学習量を維持したGNG高速化の比較

[実装・ビルド設定](../../ais_gng_cpu/experimental/gng_runtime_trials/README.md)、[検証記録](../../gng_vlut_system/docs/releases/2026-09-24_gng_runtime_trials.md)。

## 結果

ノード追加時の先頭走査を省く空き番号管理、ファイルをまたぐ最適化（LTO）、ボクセル区間検出と重心計算の統合を組み合わせた標準版で、元のグラフを維持して高速化。

同じ300個の連続入力、先頭50回を除いた250回のGNG本体平均:

| 条件 | 基準版 ms | 完全一致版 ms | 短縮率 | 基数ソート版 ms |
| --- | ---: | ---: | ---: | ---: |
| 生点群＋tree | 11.20 | 6.43 | 42.6% | 対象外 |
| 生点群＋node.grid | 9.79 | 5.45 | 44.3% | 対象外 |
| voxel 0.1 m＋tree | 25.61 | 17.53 | 31.6% | 11.25 |
| voxel 0.1 m＋node.grid | 24.31 | 16.10 | 33.8% | 10.22 |
| voxel 0.5 m＋tree | 22.08 | 14.96 | 32.2% | 11.12 |
| voxel 0.5 m＋node.grid | 21.00 | 13.48 | 35.8% | 10.08 |

全条件で`node.learning_num=4000`、`node.num_max=20000`、入力上限200,000、`node.grid=0.5 m`。入力範囲・学習係数・寿命・ノード／エッジ追加削除・探索精度は同じ。raw版は入力voxel設定を拒否、tree版はnode.grid設定を拒否する前回仕様も維持。

完全一致版は6条件×300フレームすべてで座標・法線・ラベル・ノードID・生成フレーム・エッジを含むグラフハッシュが基準版と一致。各フレームの入力数・候補数・原点選択数・ノード追加削除移動回数も一致。

## 試した実装

初期の各3試行（先頭30個の入力を繰り返した50回更新、先頭10回を除外）:

| 変更 | 生点群＋treeの平均 | 判断 |
| --- | ---: | --- |
| 基準版 | 12.29 ms | 比較用 |
| 空き番号の探索開始位置を保持 | 8.56 ms | 標準構成へ採用 |
| 最小ヒープで空き番号を管理 | 8.92 ms | 比較用に保持 |
| LTOだけ | 10.05 ms | 標準構成へ採用 |
| 上記の開始位置管理＋LTO＋重心走査統合 | 6.83 ms | 標準構成 |

入力voxel 0.1 mの重心走査統合単独は27.36→25.89 ms、0.5 mは23.52→23.19 ms。入力点の加算順序は維持。各方式の詳細内訳は`summary.json`。

追加の3試行では、実行CPU向け命令選択（`-march=native -ffp-contract=off`）の差は、同時期の完全一致版に対して約−2.4〜＋0.8%の短縮率。明確な改善がなく標準設定はOFF。

GNG入力の同じフレームを使った別計測では、ボクセル0.1 mのソート8.11 ms、0.5 mのソート6.30 msが入力整理の大半。`voxel_profile.cpp`は計測対象の重心と通常実装の一致を確認した上で段階計時。

別の測定期同士の絶対時間は混ぜず、同じ比較群内で評価。CPUや既存プロセスの負荷による変動を含むため、代表条件を単一コア固定した追加確認は`pinned_summary.json`に保存。CPU 0で各3試行した生点群＋treeは11.94→6.86 ms（42.6%短縮）、入力voxel 0.1 m＋treeは27.14→18.60 ms（31.5%短縮）、基数ソートは12.09 ms（55.4%短縮）。

## 基数ソート版

セル番号を8bitずつ4回に分けて並べ替える方式。全入力点と占有セル数、4,000回の学習枠を維持し、ボクセル版をさらに高速化。ただし、同じセル内の点の加算順序が変わるため、完全一致版とは分離した任意オプション。

実bag先頭フレーム153,512有効点の照合で、点の欠落・重複・セル番号の不一致なし。基準版との重心の座標成分差は0.1 mで最大0.0000038147 m、0.5 mで最大0.0000152588 m。

300フレームにおけるノードID・生成フレーム・ラベル・エッジの一致件数:

| 条件 | 構造・ラベル一致 | 全座標・法線を含む一致 |
| --- | ---: | ---: |
| tree、voxel 0.1 m | 300/300 | 0/300 |
| node.grid、voxel 0.1 m | 211/300 | 0/300 |
| tree、voxel 0.5 m | 35/300 | 0/300 |
| node.grid、voxel 0.5 m | 35/300 | 0/300 |

連続入力の原点以外の0.2 m被覆率は、基準版に対してtree 0.1 mで変化なし、grid 0.1 mで＋0.0016ポイント、tree 0.5 mで＋0.0042ポイント、grid 0.5 mで−0.0495ポイント。座標の丸め差が長期の接続判断へ波及するため、グラフの完全一致が必要な標準構成ではOFF。詳細は`extra_summary.json`。

## 条件と制約

- Intel Core i7-14650HX、既存`gng_cpu_container`、GCC 11.4、C++20、Release `-O3 -DNDEBUG`。
- `/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`、`/lidar_points`。
- 乱数はフレーム番号、ラベルLPFの時間刻みは0.1秒。比較群で固定。
- GNG本体の経過時間を計測。bag読込・逆シリアル化・出力品質評価は計測外。ROS通信・Viewer描画を含まない。
- 全点観測による寿命維持・重点サンプリング・クラスタリングは、比較元の最小構成と同じく停止。今回追加で省略した処理ではない。
- `Vec3f`のゼロ初期化不具合を全構成に共通修正。旧コードでテスト失敗を再現。今回の基準版は前回保存済みグラフと6条件×50フレーム一致。
- 最大RSSはbag先読みに用いるPythonも含み、ライブラリ単独のメモリ量ではない。
- 別のbag、実時間LPF、ROS統合での速度・品質は今回の測定対象外。

## 再現

既存コンテナ内で実行。ソースのビルドと測定にはそれぞれ有限タイムアウトを設定。

```bash
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/build_trials.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/run_trials.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/build_extra.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/run_extra.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/run_extended.sh
bash /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/run_pinned.sh
python3 /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/summarize.py
python3 /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/summarize_extra.py
python3 /ros2_ws/src/benchmarks/gng_runtime_trials_20260924/summarize_pinned.py
```

`build_trials.sh`は前回ソースのゼロ初期化不具合の再現も含む。各実装のCMake設定・CTest・ビルドログを方式別ディレクトリへ保存。

生データ・共有ライブラリ・ログはGit管理外の`artifacts/gng_runtime_trials_20260924/`。このディレクトリの`summary.json`等は保存済み集計であり、再集計の出力による自動上書きなし。

[verification.json](verification.json)に学習量・ソース不変性・ビルド・後片付けの照合結果、`sha256.json`に最終ソース・設定・保存ライブラリのハッシュを保存。

計測・ビルド・テストは全終了、一時ビルド領域は削除済み。本番差替え・既存プロセスへの停止操作なし。作業中に既存GNG・デーモンの停止とbag切替が外部で発生し、開始前13／終了後9プロセス。今回の測定は独立ライブラリへの直接入力であり、既存bag再生を入力源に使用していない。前後差分は生資料に記録し、外部で変更された構成は復元操作の対象外。
