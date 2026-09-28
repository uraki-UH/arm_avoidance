# 2026-09-28 - 入力セルと候補平面の照合コスト

## 要約

同じ交差点点群・同じグラフ状態で、現行のノード経由とセル対応索引を比較。
毎フレームの対応表再構築は128候補だけの現行判定より重く、判定範囲も同一ではない。
本番のノード挿入条件は変更せず、比較実装はこのディレクトリに限定。
接触セルの可視化追加は[リリースノート](../../gng_vlut_system/docs/releases/2026-09-28_plane_contact_voxels.md)を参照。

| セル幅・照合対象 | 現行照合 ms/frame | 対応表構築＋セル照合 ms/frame |
| --- | ---: | ---: |
| 0.5 m・128候補 | 0.00965 | 0.50518 |
| 1.0 m・128候補 | 0.00855 | 0.54243 |
| 0.5 m・全候補（平均1,104） | 0.06637 | 0.58182 |
| 1.0 m・全候補（平均900） | 0.04837 | 0.59098 |

0.5 m・128候補の内訳は構築0.49100 ms、照合0.01418 ms。
索引と作業配列は約0.685 MB。点群・入力voxel本体のコピーなし。
参照GNGは44.39 ms、平面計算7.13 ms、両方式共通の平面モデル・所属整理0.211 ms。
これらは別々の測定区間であり、ROS全体時間や実装切替後の実測時間ではない。
追加差分約0.496 msは参照GNG時間の約1.1%。対応表の差分更新・既存走査への統合は未測定。

## 条件・検証

- Macnica交差点bagの先頭60入力、最初の20入力を時間集計から除外、3試行の各平均を再平均。
- 入力上限200,000点、ノード上限20,000、実測平均19,107ノード、学習4,000回。
- Intel Core i7-14650HX、CPU 4固定、GCC 11.4 Release/O3、既存bag・GNG・Viewerの並行稼働。
- 同じフレーム内でノード方式・セル方式を各6回、実行順を交互に変更。指標は壁時計時間。
- GNGの乱数・時間依存は固定せず、試行間のグラフは異なる。各試行・各フレーム内は完全に共通。
- 現行判定は `cugng.cpp` から機械的に抽出。最近傍2ノード＋直接隣接・世代付き所属を使用。
- セル方式は平面所属ノードのセル番号→平面ID列。同一セル＋面共有6隣接だけの参照。
- 両方式の幾何条件は平面距離0.08 m、有限投影矩形の余白0.1 m。穴・凹形状の厳密判定なし。
- 点群の追加ボクセル化・最近傍検索・平面生成は共通前処理として照合時間から除外。
- 1 m条件でも学習用voxelは0.5 mのまま。照合用入力セル幅のみ変更した比較。
- 128は照合対象点数。実運用の32ノード挿入による早期打切りは再現せず、多めの照合量で評価。
- ノード追加を伴わない固定状態の判定比較。継続学習後の形状品質・追従改善は未検証。
- 0.5 m・128候補では現行のみ説明可能3.03点、セルのみ説明可能0.28点/frame。
- 1 mではそれぞれ0.38点、0.67点/frame。これは方式間差分であり、正解ラベルに対する精度ではない。
- 合成10条件と実bag各条件の先頭16候補について、索引と全所有ノード走査の6隣接判定が一致。
- 初回3試行は19.15秒、Header初期化警告の修正後3試行は19.90秒、失敗0。表は後者。
- 全試行 `cleanup_ok=true`、比較PID 574766/574772/574777および最終batchの子プロセスは終了。
- 原本：`artifacts/voxel_plane_lookup_20260928/final_batch/report.json`、各試行 `frames.csv`。
- 条件・入力SHA256：同ディレクトリ `conditions.json`。既存設定・installはこの比較では未変更。

コンテナ内 `/ros2_ws/src` での比較再現コマンド（出力先は未作成のパスが必要）：

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 timeout 90 python3 benchmarks/voxel_plane_lookup_20260928/prepare.py \
  --output /ros2_ws/src/artifacts/voxel_plane_lookup_retest
cmake -S benchmarks/voxel_plane_lookup_20260928 \
  -B artifacts/voxel_plane_lookup_retest/build -DCMAKE_BUILD_TYPE=Release
timeout 240 cmake --build artifacts/voxel_plane_lookup_retest/build -j4
PYTHONDONTWRITEBYTECODE=1 python3 skills/run-benchmark-batch/scripts/run_batch.py \
  artifacts/voxel_plane_lookup_retest/cases.json --output artifacts/voxel_plane_lookup_retest/batch \
  --repeats 3 --timeout-sec 120 --max-total-sec 360 --estimate-sec 6.4
```

実行時は `docker exec gng_cpu_container bash -lc '…'` 内で上記を使用。
現在の比較コードには可視化API抽出区間の計測も追加済み。上表はその追加前の照合比較。

2026-09-28追記：接触抽出の最適化を同じ入力・グラフ上で旧実装と比較。
最大8 MiBのビット表による直接参照と、前回使用した64セル単位の領域だけのクリアを採用。
点群の再ボクセル化なし。範囲の大きいグリッドは旧ハッシュ索引へ退避。
毎回の平面参照ノード走査は継続。移動・所属変更イベントによる完全な差分管理ではない。

- 最終3試行：旧API 1.225 → 新API 0.725 ms/出力（40.8%減）、平均12,929セル。
- 各入力で新旧を6回交互計測。参照列作成を除くAPI時間。ROS送信・描画時間は対象外。
- 参照列作成を含む最初の新API呼出しは1.101 ms。交互測定とはキャッシュ条件が異なる。
- 全60入力×3試行でセル数・順序・中心・点数・区分・幅が完全一致。
- 疎索引への退避、表縮小・復帰、古い世代・重複参照・復帰の6条件も各試行で一致。
- 初案の隣接キー事前展開は旧1.228→新1.463 msと悪化。順次照合案も悪化し、ともに製品不採用。
- 製品構成の再ビルドとCTest 23件成功。稼働installへの反映・ROS再起動は今回未実施。
- 最終比較は予測24秒→22.12秒、失敗0、全試行cleanup_ok=true。試験・ビルドは終了。
- 原本：同artifactのcontact_bits_final_batch/report.jsonとcontact_bits_product_test.log。
- 再現は上記コマンドの出力先を変更して実行。初案記録はcontact_opt_batch、順次案はcontact_merge_batch。
