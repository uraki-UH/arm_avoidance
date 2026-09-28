# 2026-09-28 - 追従候補の1段隣接拡張の試作・撤去

## 要約

同日のユーザー指示「26近傍方式はやらなくていい」により、試作実装・API・設定受付・専用テストを撤去。
以下は撤去前の測定記録。現行ソースでは`adjacent_nonplane`を利用不可、通常設定の変更なし。
撤去後も通常Release2パッケージを再ビルド・反映し、CPU25件・ROS関連5件成功。既存プロセス維持。
撤去後の起動は下記の通常colconコマンドと、各CTestへ`timeout 60`を付加したもの。すべて終了済み。

試作の`adjacent_nonplane`は最近傍非平面に一致した入力セルから、26近傍の現在占有セルへ1段だけ拡張。
元点の再集計・別グリッド・半セルシフト・再帰拡張なし。既存候補トピックと固定学習枠を共用。
2倍セル・半セルシフトの試作は撤去。ユーザーの単純方式への変更指示を反映。
ただし追加コストに見合う追従品質の改善は未確認。ROS既定・at128は`nearest_nonplane`を維持。
比較用modeとbenchmark引数拡張・専用manifestは撤去済み。生ログ・計測結果だけ保持。

合成円筒の80入力×3条件×3種、停止20入力後の移動60入力平均を3試行平均。時間はms/入力：

| 入力セル幅 | 指標 | nearest | adjacent |
| --- | --- | ---: | ---: |
| 0.5 m | API全体時間 | 7.889 | 8.192 |
| 0.5 m | 物体候補被覆率 | 99.758% | 100% |
| 0.5 m | 候補中の物体割合 | 96.106% | 84.351% |
| 0.5 m | 物体→最近傍ノード平均距離 | 0.10368 m | 0.10515 m |
| 0.1 m | API全体時間 | 41.935 | 42.433 |
| 0.1 m | 物体候補被覆率 | 98.816% | 100% |
| 0.1 m | 物体→最近傍ノード平均距離 | 0.10362 m | 0.10358 m |

0.5 mでは背景の取り込みが増え、誤差は約1.4%悪化。0.1 mの誤差変化は小さく改善主張なし。
実交差点bag・60入力・2条件×2試行ではGNG欄40.298→45.100 ms、全体processing61.181→66.807 ms。
配分50%と候補点群の購読を双方共通に設定。追加索引照合だけでなく、学習結果・後段処理の変化も含む。
全試行成功。合成各9試行の所要時間12.31／40.11秒、初期予測72／18秒。実bag4試行143.90秒、初期予測140秒。

## 条件・検証

- 合成：10万点（地面95,000、円筒5,000）、上限4,000ノード・学習4,000回、CPU 4固定。
  非平面所属は既知形状の高さによる代用。実車・平面クラスタリングの品質評価ではない。
  再現用ライブラリのコピーだけ乱数・経過時間を固定。本番の乱数・時刻更新は変更なし。
- API時間は入力設定からグラフ取得まで。候補添字取得・正解照合は別計時、ROS配信費用を含まない。
  OFFのAPI時間はセル0.5 mで7.465 ms、0.1 mで41.409 ms。セル幅間は同一処理負荷ではない。
- 実bag：Macnica交差点先頭60入力、上限20万点・2万ノード、voxel 0.5 m、CPU 4固定。
  DOMAIN_ID=183、local座標、平面／非平面抽出ON、人車分類OFF。通常点群表示OFF、候補配信ON。
  各試行の入力20〜54の平均を試行平均。正解ラベル・車体全体の被覆・Viewer描画は未検証。
- 通常Releaseのgng_cpu・ais_gngをビルド。CPU25件・ROS関連5件成功。
  対角・負座標・範囲端・複数起点・重複・非再帰・候補なし・voxel OFF・古い世代・固定配分を検証。
- 実bagの別試行で、初回・frame変更・巻戻し・時刻ギャップの候補失効を検証。
- 製品構成のCTest22件成功。候補失効試行の最大135,270点、初回・末尾4入力は0点。
- 互換性：組込みenumの値を追加。外部規則構造体も拡張のため利用側を再ビルド。
  製品版の任意外部サンプラー禁止は維持。設定・詳しい制約は[仕様](../../ais_gng_cpu/docs/sampling.md)。

撤去前の起動コマンド。`docker exec gng_cpu_container bash -lc '…'`内で実行、現行コードへの再実行不可：

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src
PYTHONDONTWRITEBYTECODE=1 timeout 240 python3 benchmarks/tracking_attention_20260926/verify.py prepare --tag adjacent_20260928_
PYTHONDONTWRITEBYTECODE=1 python3 benchmarks/tracking_attention_20260926/verify.py suite --tag adjacent_20260928_ --cases off builtin_nearest builtin_adjacent --ratio .5 --input-voxel-size .5 --frames 80 --warmup-frames 20 --output artifacts/tracking_attention_20260926/adjacent_cases_05.json
PYTHONDONTWRITEBYTECODE=1 python3 skills/run-benchmark-batch/scripts/run_batch.py artifacts/tracking_attention_20260926/adjacent_cases_05.json --output artifacts/tracking_attention_20260926/adjacent_batch_05 --repeats 3 --timeout-sec 90 --max-total-sec 500 --estimate-sec 8
PYTHONDONTWRITEBYTECODE=1 python3 skills/run-benchmark-batch/scripts/run_batch.py benchmarks/tracking_attention_20260926/adjacent_cases.json --output artifacts/tracking_attention_20260926/adjacent_ros --repeats 2 --timeout-sec 120 --max-total-sec 480 --estimate-sec 35
ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 15 120 taskset -c 4 python3 benchmarks/tracking_attention_20260926/verify.py ros --tracking-mode adjacent_nonplane --cell-size .5 --ratio .5 --max-points 200000 --seed 28 --validation --output artifacts/tracking_attention_20260926/adjacent_validation.json
```

0.1 m比較はsuiteの入力幅を`.1`、出力接尾辞を`01`、runner予測を2秒へ変更。他の条件は同じ。
manifest・runner出力は新しいパス限定。生ログ・結果・子ノードのPIDと終了コードは各出力先に保持。
通常ビルド：`cd /ros2_ws && timeout 360 colcon build --packages-select gng_cpu ais_gng --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+`。
回帰：`ctest --test-dir /ros2_ws/build/gng_cpu --output-on-failure`。
ROS回帰：`ctest --test-dir /ros2_ws/build/ais_gng -R 'test_grasp_attention|test_boundary_attention|test_spatial_sampling|test_nonplane_component_extractor|test_plane_cluster_incremental' --output-on-failure`。
製品検証は`artifacts/tracking_attention_20260926/adjacent_product`へ分離し、通常installを上書きしない。
CMakeはRelease・allow_external_sampler=OFF・GNG_BUILD_BENCHMARKS=ON・GNG_ENABLE_FRAME_LOG=OFF、ビルド`-j4`。
製品CTestは`LD_LIBRARY_PATH`先頭を同ディレクトリとし、`ctest --test-dir <product> --no-tests=error --output-on-failure`。

全ビルド・runner・試験ノードは終了。試験CPU535079／535104／535131／535158／535295は終了コード0。
既存GNG532504・平面532508・bag529072・Viewer333336と3コンテナを維持。停止・再起動操作なし。
ROSデーモンの新規起動なし、最終`git diff --check`成功。既存の符号比較・Torch等のビルド警告は未修正。
