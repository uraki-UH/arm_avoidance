# 最小GNGのボクセル・ツリー比較

[実装仕様](../../ais_gng_cpu/experimental/gng_minimal_comparison/README.md)、[検証記録](../../gng_vlut_system/docs/releases/2026-09-24_gng_minimal_comparison.md)。

## 条件

- 既存`gng_cpu_container`、ROS Humble、C++20、Release、`-O3 -DNDEBUG`、乱数・時間刻み固定。
- `/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の`/lidar_points`。
- 先頭30フレームを繰り返した50回更新、先頭10回を除外。6条件×3試行、順番を変更して順次実行。
- 入力は約153,310点／フレーム。`node.learning_num=4000`、`node.num_max=20000`、`node.grid=0.5 m`。
- 入力voxelは0.1 mと0.5 m。学習候補数は約79,577セルと15,668セル。
- 全点の事前ノード照合・重点候補・クラスタリングを全条件で停止。
- `gng_exec()`の経過時間を測定。bag読込・逆シリアル化・距離評価は時間計測外。
- 入力転送・出力変換は別計測。ROSの配送・Viewer描画は含まない。
- 既存ROS・bag・Viewerを稼働したまま測定。別日時の測定結果との絶対時間の混合集計なし。

## 結果

| 条件 | 本体 ms | 入力整理 ms | 学習 ms | 法線・ラベル ms | 寿命整理 ms | 平均ノード数 | 0.2 m被覆率 | 0.4 m被覆率 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 生点群＋tree | 11.30 | 0.98 | 6.95 | 2.89 | 0.47 | 8441 | 52.34% | 85.95% |
| 生点群＋node.grid | 11.15 | 0.99 | 7.29 | 2.50 | 0.35 | 7550 | 51.26% | 84.40% |
| 入力voxel 0.1 m＋tree | 26.37 | 12.10 | 9.89 | 3.75 | 0.62 | 10779 | 53.33% | 91.54% |
| 入力voxel 0.1 m＋node.grid | 24.98 | 11.81 | 9.36 | 3.36 | 0.45 | 10154 | 51.78% | 90.82% |
| 入力voxel 0.5 m＋tree | 22.47 | 8.92 | 9.38 | 3.57 | 0.60 | 11314 | 41.11% | 89.38% |
| 入力voxel 0.5 m＋node.grid | 21.37 | 8.64 | 8.93 | 3.33 | 0.46 | 11191 | 41.58% | 89.55% |

被覆率はYAML範囲内の原点以外の各元点について、最近傍ノードまでの距離が各値未満の比率。5時点で評価し平均。原点を含む全点の評価も`summary.json`と生JSONに保持。入力から原点を削除したわけではない。

生点群で探索方式の差は約0.16 ms。voxel版でnode.gridはtreeより約1.1〜1.4 ms短いが、局所探索・収容上限の違いでノード数・接続・被覆率も異なる。

入力ボクセル化は約8.6〜12.1 ms。学習はどの条件も4,000回のため、候補数の削減による検索回数減少なし。占有セル単位の一様抽出で点密度による偏りが変わり、ノード数が約7,550〜8,441から約10,154〜11,314へ増加。学習・ラベル計算も増加。

生点群の原点学習は平均642回／4,000回、voxel 0.1 mは0.05回、0.5 mは0.325回。密な原点群の学習比率も大幅に変化。原点を含む被覆率では、原点付近のノードが保持されない影響が大きい。

これらは最小構成の短時間比較。全点観測による寿命維持・重点サンプリングを含む本番版への性能・品質の保証ではない。

## 再現

上記実装READMEの手順で4ライブラリを`artifacts/gng_minimal_comparison_20260924/`へ配置。既存コンテナ・依存関係・bagが必要。

```bash
docker exec gng_cpu_container bash -lc \
  'timeout -s INT -k 10 900 bash /ros2_ws/src/benchmarks/gng_minimal_comparison_20260924/run_benchmarks.sh'
docker exec gng_cpu_container bash -lc \
  'python3 /ros2_ws/src/benchmarks/gng_minimal_comparison_20260924/summarize.py'
```

- `run_benchmarks.sh`: 18試行。個別計測は120秒の上限付き。
- `summarize.py`: 900フレームの試行間グラフ一致・検索回数・設定受理の確認。保存済みコピー元JSONがある場合、tree_rawの50フレーム一致も確認。
- `summary.json`: 保存済み集計。再集計の出力は`artifacts/`であり自動上書きなし。
- `verification.json`: ソース不変性・Release・テスト・後片付けの確認結果。
- `sha256.json`: ソース・設定・スクリプト・ライブラリのハッシュ。
- 生JSON・グラフ・ログ・共有ライブラリ・実行前後の状態はGit管理外の`artifacts/gng_minimal_comparison_20260924/`。

計測・ビルド・CTestは全終了。既存プロセスへの停止・再起動操作なし。一時ビルド`/tmp/gng_minimal_comparison_build`は削除済み。
