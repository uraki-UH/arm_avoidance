# 平面クラスタの整合化・局所スケール追従と内部小面統合の比較

## 1本接続統合の点数制限撤去（2026-09-25）

`max_fragment_nodes`の30点上限と設定を削除。相対点数条件や新しい設定の追加なし。
短い1本接続・両側／接触部／統合後の残差・連続3フレーム確認を維持し、残差上限の選択を共通化。
通常の接続要求2本の場合だけ救済。要求3本以上・長い橋・段差・傾斜面の制約を維持。
追加探索・全点再走査・新しい処理段階なし。ただし従来は点数で除外した候補にも既存の平面評価が必要。

保存済み同一入力150フレーム、先頭50を除く平均の交互3試行中央値：[simple_summary.json](simple_summary.json)。

| 項目 | 点数制限あり | 点数制限なし |
| --- | ---: | ---: |
| 平面処理CPU時間 [ms] | 12.075 | 12.205 |
| 出力クラスタ数 | 64.99 | 63.99 |
| 平面所属点数 | 12,627.35 | 12,620.16 |
| 平面所属率 [%] | 65.960 | 65.923 |
| 未所属点数 | 6,516.55 | 6,523.74 |

CPU時間は約1.1%増で、試行分布には重なりあり。高速化・所属点増加の主張なし。
最終出力の位置とエッジの一致を確認したうえで、旧ID 49の93点中87点がID 11の1,170点へ合流、5点は未所属、1点はID 6へ移動。最終クラスタ数69→68。
実bagの誤統合率、全断片の解消、ライブ画像との対応は未検証。
`--disable-fragment-merge`時は全150フレームの非時間集計列が前後一致。ノード単位の完全一致検証とは別。

コンテナ内（`/ros2_ws/src`）での比較コマンド：

```bash
for method in simple_before simple_after; do
  timeout 100 bash benchmarks/plane_consistency_20260924/build.sh "$method"
done
for trial in 1 2 3; do
  for method in simple_before simple_after; do
    timeout 45 artifacts/plane_consistency_20260924/"$method"/replay \
      artifacts/plane_consistency_20260924/frames.bin \
      > artifacts/plane_consistency_20260924/"$method"/final_"$trial".csv
  done
done
python3 benchmarks/plane_consistency_20260924/summarize.py --prefix simple_
```

変更前ソースとヘッダは`artifacts/plane_consistency_20260924/simple_before/`へ保存。
変更後の平面53テスト成功、変更前は点数制限撤去の追加1件が失敗。1,170＋99点／1,170＋1,170点、スケール0.1/1/10倍、段差・傾斜面拒否を検証。
ROS用Releaseビルドと関連133テスト（平面53・非平面6・曲面74）が成功。
`install.sh simple_runtime_before`で3バイナリをinode単位で反映、変更前を保存。

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 timeout -s INT -k 15 90 \
  python3 benchmarks/plane_consistency_20260924/smoke.py
```

上記ROS試験で共有パラメータ・従来ケースに加え、1,170＋99点の3フレーム統合と段差0.04 mの拒否を確認。
ログは`simple_ros_build.log`・`simple_ros_ctest.log`・`simple_smoke.log`。
試験ノードPID 221849/221861、比較・ビルドプロセスは終了。既存GNG/平面212638/212642、bag 202445、gateway 202494と3コンテナの稼働を維持。
ROS試験後も検証用・配布先の3バイナリの一致を確認。反映前から稼働中のGNGは同じlaunchで再起動が必要。

## 未所属ノードの面内取り込み救済（先行測定、2026-09-25）

通常条件で拒否した未所属点だけを追加判定。確認済みの単一平面への短い接続と、全隣接エッジの面内方向を要求。
1本接続は通常要求2本のときだけ成長距離0.15まで、通常の接続要求を満たす複数接続は保持距離0.30まで。
`enable_coplanar_absorption: true`、`max_absorption_edge_angle_deg_th: 20.0`、`max_absorption_edge_ratio_th: 2.5`。
既所属間の移動・新規平面生成・通常の併合条件は変更なし。通常条件を通る点への追加制約ではなく、取り残しの救済専用。
候補点あたり最大1平面の隣接走査、二乗内積・二乗長で判定。追加近傍探索・PCA・エッジごとの角度関数なし。
追加走査は保守パスあたりO(N+E)の範囲。所属増加による既存の再フィット・分断確認の費用は別途存在。

同じ保存済み150フレーム、先頭50を除いた平均の交互3試行中央値：[absorption_summary.json](absorption_summary.json)。

| 項目 | 変更前 | 取り込み救済追加 |
| --- | ---: | ---: |
| 平面処理CPU時間 [ms] | 12.220 | 11.958 |
| 平面所属点数 | 11,614.82 | 12,627.35 |
| 平面所属率 [%] | 60.671 | 65.960 |
| 未所属点数 | 7,529.08 | 6,516.55 |
| 出力クラスタ数 | 69.30 | 64.99 |
| 解放点数 | 5.66 | 14.58 |
| 新規生成数 | 1.05 | 0.44 |
| 分割数 | 0.08 | 0.28 |

未所属点は約13.4%減少。救済数は平均69.85点/フレーム。所属点数の増加は実シーンの正解ラベルによる品質保証とは別。
解放・分割回数は増加。CPU時間は試行の分布に重なりがあり、厳密な高速化とは断定しない。
OFF時は全150フレームの非時間集計列が変更前と一致。ノード単位の完全一致検証とは別。
変更前保存ソースは`absorption_before/plane_cluster_incremental.cpp`、当時の共通ヘッダと組み合わせたABI統一比較。現行ビルドスクリプトでは点数上限廃止前の`simple_before/include`を使用。

規模別比較（各100測定フレーム、先頭20フレーム除外、交互3試行のCPU平均時間の中央値）：

| 入力 | ノード数 | 変更前 [ms] | 変更後 [ms] |
| --- | ---: | ---: | ---: |
| 定常 | 2,760 | 0.437 | 0.439 |
| 定常 | 22,080 | 3.747 | 3.822 |
| 定常 | 44,160 | 9.165 | 8.469 |
| 接続変化・並べ替え | 2,760 | 0.519 | 0.585 |
| 接続変化・並べ替え | 22,080 | 4.068 | 4.340 |
| 接続変化・並べ替え | 44,160 | 9.439 | 9.609 |

既存ROS稼働中、CPU固定なし。相対変化約−7.6〜+12.8%、全条件での高速化ではない。
各版の`scale_{steady|dynamic}_{1|8|16}_{1|2|3}.txt`へ保存。

コンテナ内の今回の比較コマンド（`/ros2_ws/src`、保存済み入力使用）：

```bash
for method in absorption_before absorption_after; do
  timeout 100 bash benchmarks/plane_consistency_20260924/build.sh "$method"
done
for trial in 1 2 3; do
  for method in absorption_before absorption_after; do
    timeout 45 artifacts/plane_consistency_20260924/"$method"/replay \
      artifacts/plane_consistency_20260924/frames.bin \
      > artifacts/plane_consistency_20260924/"$method"/final_"$trial".csv
  done
done
python3 benchmarks/plane_consistency_20260924/summarize.py --prefix absorption_
```

本番Releaseの関連131テスト（平面51・非平面6・曲面74）が成功。追加5テストのうち先行4件で変更前の失敗2件を確認。
`absorption_after/tests.log`は先行50件、最終51件は`ros_build/test_results/ais_gng/test_plane_cluster_incremental.gtest.xml`。
新規救済のスケール0.1/1/10倍・回転、距離・接続・面外形状・法線・長い橋・OFF・角度設定・絶対上限・確認待ち・既所属移動非適用を確認。
`install.sh absorption_runtime_before`で検証済みライブラリ2本と平面ノードをinode単位で反映、旧バイナリを保存。
分離ROSでCPU/平面ノードの3新規パラメータ宣言を照合。取り込み救済と面外形状の拒否を各6フレームで確認。

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 timeout -s INT -k 15 90 \
  python3 benchmarks/plane_consistency_20260924/smoke.py
```

ログは`absorption_ros_build.log`・`absorption_ros_ctest.log`・`absorption_smoke.log`。
試験ノードPID 204484/204497は正常終了。比較・ビルドプロセスも終了。既存ROS・bag・Viewerの停止なし。
最終照合時に通常ビルド先への別ビルド更新を検出。現行バイナリを上書きせず、`LD_LIBRARY_PATH=/ros2_ws/build/ais_gng:$LD_LIBRARY_PATH`で131テストと上記ROS試験を再実施し成功。
`absorption_current_runtime_ctest.log`・`absorption_current_runtime_smoke.log`・`absorption_current_runtime.sha256`へ保存。試験前後の3バイナリのハッシュ一致を確認。
再試験ノードPID 209816/209828も正常終了。bag 202445、gateway 202494と3コンテナの稼働を維持。
最終確認中に試験外でGNG/平面ノードが202402/202406から209853/209857へ更新。こちらからの停止操作なし。新GNGの起動設定で救済ON・角度20°・接続長比2.5を確認し、読み込み済み平面ライブラリのinodeが再検証済み現行版と一致。
反映前から起動中のインスタンスが別途残る場合は、同じlaunchで再起動が必要。

## 面外分断追加時の比較（先行測定）

2026-09-24に交差点bagから生成した同じGNG入力150フレームを、2026-09-25の面外分断修正前後へ入力。
先頭50フレームを除く100フレームの平均を、交互3試行の中央値で比較。
生データ・変更前ソース・ビルド・ログはGit対象外の`artifacts/plane_consistency_20260924/`。
当時の集計値と入力SHA-256は[direction_summary.json](direction_summary.json)。
先行の面幅・保持整合化は[summary.json](summary.json)、スケール追従は[scale_summary.json](scale_summary.json)、接触部照合は[contact_summary.json](contact_summary.json)、小断片救済は[fragment_summary.json](fragment_summary.json)に保存。

| 項目（1フレームあたり） | 小断片救済版 | 面外分断追加 |
| --- | ---: | ---: |
| 平面処理CPU時間 [ms] | 12.676 | 11.840 |
| 平面所属点数 | 11,521.26 | 11,614.82 |
| 平面所属率 [%] | 60.182 | 60.671 |
| 未所属点数 | 7,622.64 | 7,529.08 |
| 出力クラスタ数 | 77.29 | 69.30 |
| 解放点数 | 5.44 | 5.66 |
| 新規生成数 | 1.55 | 1.05 |
| 分割数（小成分の解放を含む） | 2.30 | 0.08 |

時間は`Clusterizer::update`全体。GNG本体・ROS変換・描画は対象外。
CPU時間は約6.6%減少。過去測定との時間差ではなく今回の交互試行間で比較。
後半100フレームの平均で、面外根拠不足の保持30.63成分、確認待ち0.85成分、全エッジ消失の猶予対象0点。
今回の実入力比較では孤立猶予の効果は未観測。別途、正解形状試験とROS試験で確認。
クラスタ総数の減少は、実シーンの全領域での分割正解を示す結果ではない。
長い同一平面の統合、段差・直交面・密度差のある平行面の分離は、正解形状既知の回帰テストで確認。

## 先行測定の条件・検証

- 入力：`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の`/lidar_points`先頭150件。
- 取得時の`at128.yaml`をコピーしてCPU GNGへ設定。入力voxel 0.5 m、learning 4000、ノード上限20,000。評価区間の平均ノード数19,143.90。
- `capture.py`はbagを読み取り専用で処理。ROSノード・publisher・TF変換なし。ライブViewerと同じ時刻の再現ではない。
- 平面設定：前後で同じ共有ROS既定値、rho種順序再利用OFF。通常接続要求2本・距離上限なし。
- 相互照合を直接接続エッジの端点へ限定。別途、各側全体の統合後平面へのRMS比を検査。各評価点群の局所間隔平均で正規化し、比0.15を適用。
- 接続端点の累積統計を既存エッジ走査で作成。対ごとの全点再走査・追加の固有値分解なし。連鎖統合時は更新された成分平面との照合。
- 救済は`enable_fragment_merge: true`、`max_fragment_nodes: 30`、`max_fragment_edge_ratio_th: 2.5`、`max_fragment_residual_ratio_th: 0.10`、`min_fragment_merge_frames: 3`。通常接続要求が2本の場合だけ適用。連続確認前の候補では通常なら省略する平面評価が必要。
- 同じ永続ID対の連続適合を記録。条件不適合・接続消失・reset・空入力で履歴を破棄。接続なしの成分・未所属点を挟む成分の橋渡しなし。
- 面外分断は`enable_directional_split: true`。最大成分以外の既存エッジだけを検査。平面との角度30°超の接続を持つノードが2点かつ成分の25%を満たす場合に確認を開始。既定3フレームの猶予後、4フレーム目に分割。
- 二乗内積比較と早期打切りによる判定。新たな全点探索・固有値分解なし。既存BFSの訪問列を再利用。非連結成分を保持するため、そのクラスタでは連結探索の再実行が必要。
- 全エッジ消失は前回の局所間隔・法線で5フレーム保持判定を継続。距離・法線逸脱の解除は維持。既存所属がない点、reset・空入力を挟んだ点は対象外。
- ビルド：コンテナ内g++、C++17、`-O3 -DNDEBUG`、Eigen3。CPU固定なし、既存ROS稼働中。ROS用ビルドは計測後。待機時間を除くスレッドCPU時間で比較。
- 変更前の小断片救済版ソースを`direction_before/`へ保存。追加設定・統計のABIを揃えるため現行ヘッダと旧実装を使用。変更後はワークスペース実装。先行比較のソース・ログも保持。
- 平面46回帰テスト成功。追加8件中6件は変更前に失敗、残り40件成功。面内接続切れの保持、未所属/別クラスタへの面外接続、小成分ごとの確認、根拠中断、添字並べ替え、孤立猶予を確認。0.1/1/10倍と座標回転も成功。
- 既存の幅20 m・内部小面幅2 m・法線差1°、0.1/1/10倍とクラスタ順反転、6フレームのID安定も成功。
- 同じ内部小面の高さ10 cm時と、接触部が一致する30°傾斜小面は分離。既存の段差・直交面・密度差・連鎖統合・スケール追従も成功。
- 各方式の3試行で、全150フレームの非時間出力列が一致。
- 面外分断OFF（`--disable-directional-split`）で、全150フレームの非時間集計列が変更前と一致。ノード単位の完全一致検証とは別。先行の小断片救済OFFの比較は当時の保存結果。
- GUI上の見た目、車・段差の正解ラベルを使った実bagの誤統合率は未検証。
- 救済追加前の診断：後半100フレームで接続不足18.47対、各側・接触部残差18.76対/フレームで拒否。診断自体の追加では非時間集計が一致。
- 同診断の最終フレームで、小片を含む隣接44対のうち幾何条件を通る4対が1本接続で残存。ID対66/399、21/152、285/399、139/448。接続長比約1.34〜2.15。小断片61個中19個は他平面への直接接続なし。
- 未所属点を1個挟む2対（176/223、285/439）の位置条件適合は救済追加前の診断結果。法線・時間安定性未評価、今回の統合対象外。画面との対応は未確認。
- 先行の診断資料は`artifacts/plane_consistency_20260924/fragment_diagnosis/`・`fragment_after/`に保持。現行版の最終点群・接続・所属と集計は`direction_after/last_frame.json`・`statistics.csv`。

規模別の合成入力（各100測定フレーム、先頭20フレーム除外、交互3試行のCPU平均時間の中央値）：

| 入力 | ノード数 | 変更前 [ms] | 変更後 [ms] |
| --- | ---: | ---: | ---: |
| 定常 | 2,760 | 0.406 | 0.428 |
| 定常 | 22,080 | 3.553 | 3.705 |
| 定常 | 44,160 | 7.573 | 7.483 |
| 接続変化・並べ替え | 2,760 | 0.473 | 0.468 |
| 接続変化・並べ替え | 22,080 | 3.849 | 3.799 |
| 接続変化・並べ替え | 44,160 | 7.889 | 8.040 |

各版の`scale_{steady|dynamic}_{1|8|16}_{1|2|3}.txt`へ保存。方向判定には追加費用があり、全条件での高速化ではない。
再現：各版の`synthetic steady 16 100`または`synthetic dynamic 16 100`（第2引数は入力コピー数）。

今回の診断（コンテナ内、取得済み`frames.bin`を使用）：

```bash
timeout 100 bash benchmarks/plane_consistency_20260924/build.sh absorption_after
timeout 45 artifacts/plane_consistency_20260924/absorption_after/replay \
  artifacts/plane_consistency_20260924/frames.bin --diagnose \
  > artifacts/plane_consistency_20260924/absorption_after/statistics.csv \
  2> artifacts/plane_consistency_20260924/absorption_after/last_frame.json
timeout 30 python3 benchmarks/plane_consistency_20260924/diagnose_fragments.py \
  artifacts/plane_consistency_20260924/absorption_after/last_frame.json
```

診断用ビルド・実行プロセスは全終了。診断によるROSノード・再生の起動停止なし。

面外分断の過去測定時の手順（現行実装の比較には上記`absorption_`を使用）：

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src
timeout -s INT -k 5 120 python3 benchmarks/plane_consistency_20260924/capture.py \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --config artifacts/plane_consistency_20260924/at128.yaml \
  --output artifacts/plane_consistency_20260924/frames.bin --frames 150
for method in direction_before direction_after; do
  timeout 90 bash benchmarks/plane_consistency_20260924/build.sh "$method"
  artifacts/plane_consistency_20260924/"$method"/tests
done
for trial in 1 2 3; do
  for method in direction_before direction_after; do
    timeout 45 artifacts/plane_consistency_20260924/"$method"/replay \
      artifacts/plane_consistency_20260924/frames.bin \
      > artifacts/plane_consistency_20260924/"$method"/final_"$trial".csv
  done
done
python3 benchmarks/plane_consistency_20260924/summarize.py --prefix direction_
```

`direction_before/tests`の追加6件の失敗は回帰検出の確認結果。保存済み入力の比較では取得コマンドを省略。
ROS用Releaseビルドは`artifacts/plane_consistency_20260924/ros_build`へ分離。
`ais_gng_component_cpu`・`plane_cluster_incremental_node`を再ビルドし、平面46・非平面6・曲面74テストが成功。
CTestでは`LD_LIBRARY_PATH=/ros2_ws/src/artifacts/plane_consistency_20260924/ros_build:$LD_LIBRARY_PATH`を指定し、ビルドしたライブラリを優先。
`bash benchmarks/plane_consistency_20260924/install.sh direction_runtime_before`でライブラリ2本・実行ファイル1本をinode単位で反映。
反映直前のバイナリは`direction_runtime_before/`へ保存済み。先行修正時のバックアップも保持。配布先と試験用ビルドの一致を`cmp`で確認。

配布版の分離ドメイン試験：設定宣言値と共有YAMLの照合、間隔5 mm・5 cm・2 mの720ノード長平面、1,706ノードの内部小面統合と高さ10 cm時の分離。内部小面は各6フレーム連続で確認。169ノードの1本接続対は最初の2フレームで分離、3フレーム目で統合、以降も安定。
追加で、同じ169ノードの切断後9フレームの所属維持、面外接続追加後4フレーム目の分割、孤立ノードの5フレーム猶予・6フレーム目の解除・再接続の復帰を確認。

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 timeout -s INT -k 15 90 \
  python3 /ros2_ws/src/benchmarks/plane_consistency_20260924/smoke.py
```

面外分断の先行ログは`direction_smoke.log`・`direction_ros_build.log`・`direction_ctest.log`。
先行の小断片救済版は`fragment_smoke.log`・`fragment_ctest.log`へ保存。並行ビルド更新後の再検証は`fragment_current_runtime_ctest.log`・`fragment_current_runtime_smoke.log`。同時点の整数型不一致は型指定後に解消（`fragment_ros_build_type_error.log`）。
先行修正のCTest旧ライブラリ参照、Python 3.10の配列初期化エラーは解消済み。失敗記録は`contact_ctest_old_library.log`・`contact_smoke_array_error.log`。
先行修正時のDDSドメイン上限・Durability不一致は対応済み。今回もドメイン173・Transient Localを使用。
取得・比較・単体試験・ROS試験プロセスはすべて終了済み。既存のbag再生・GNG・Viewerの停止なし。
今回の試験ノードPID 199828・199840は正常終了。開始前のbag 129194、gateway 159102、GNG 198732、exporter 198734、平面ノード198736と3コンテナの稼働を維持。ROSデーモンの新規起動なし。
再取得時のGNG乱数は固定していないため、保存済み`frames.bin`のSHA-256で同一入力を識別。
