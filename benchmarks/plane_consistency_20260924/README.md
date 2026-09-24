# 平面クラスタの整合化・局所スケール追従と内部小面統合の比較

## 要約

2026-09-24、交差点bagから生成した同じGNG入力150フレームを修正前後へ入力。
先頭50フレームを除く100フレームの平均を、交互3試行の中央値で比較。
生データ・変更前ソース・ビルド・ログはGit対象外の`artifacts/plane_consistency_20260924/`。
最新の集計値と入力SHA-256は[fragment_summary.json](fragment_summary.json)。
先行の面幅・保持整合化は[summary.json](summary.json)、スケール追従は[scale_summary.json](scale_summary.json)、接触部照合は[contact_summary.json](contact_summary.json)に保存。

| 項目（1フレームあたり） | 接触部照合版 | 小断片救済追加 |
| --- | ---: | ---: |
| 平面処理CPU時間 [ms] | 12.575 | 12.573 |
| 平面所属点数 | 11,522.08 | 11,521.26 |
| 平面所属率 [%] | 60.187 | 60.182 |
| 未所属点数 | 7,621.82 | 7,622.64 |
| 出力クラスタ数 | 80.68 | 77.29 |
| 解放点数 | 5.43 | 5.44 |
| 新規生成数 | 1.57 | 1.55 |
| 分割数（小成分の解放を含む） | 2.26 | 2.30 |

時間は`Clusterizer::update`全体。GNG本体・ROS変換・描画は対象外。
平面所属率・CPU時間はほぼ維持、クラスタ数は減少。過去測定との時間差ではなく今回の交互試行間で比較。
全150フレームで1本接続救済32回。最終フレームは82→79クラスタ、30ノード以下の断片は61→56。
クラスタ総数の減少は、実シーンの全領域での分割正解を示す結果ではない。
長い同一平面の統合、段差・直交面・密度差のある平行面の分離は、正解形状既知の回帰テストで確認。

## 条件・検証

- 入力：`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の`/lidar_points`先頭150件。
- 取得時の`at128.yaml`をコピーしてCPU GNGへ設定。入力voxel 0.5 m、learning 4000、ノード上限20,000。評価区間の平均ノード数19,143.90。
- `capture.py`はbagを読み取り専用で処理。ROSノード・publisher・TF変換なし。ライブViewerと同じ時刻の再現ではない。
- 平面設定：前後で同じ共有ROS既定値、rho種順序再利用OFF。通常接続要求2本・距離上限なし。
- 相互照合を直接接続エッジの端点へ限定。別途、各側全体の統合後平面へのRMS比を検査。各評価点群の局所間隔平均で正規化し、比0.15を適用。
- 接続端点の累積統計を既存エッジ走査で作成。対ごとの全点再走査・追加の固有値分解なし。連鎖統合時は更新された成分平面との照合。
- 救済は`enable_fragment_merge: true`、`max_fragment_nodes: 30`、`max_fragment_edge_ratio_th: 2.5`、`max_fragment_residual_ratio_th: 0.10`、`min_fragment_merge_frames: 3`。通常接続要求が2本の場合だけ適用。連続確認前の候補では通常なら省略する平面評価が必要。
- 同じ永続ID対の連続適合を記録。条件不適合・接続消失・reset・空入力で履歴を破棄。接続なしの成分・未所属点を挟む成分の橋渡しなし。
- ビルド：コンテナ内g++、C++17、`-O3 -DNDEBUG`、Eigen3。CPU固定なし、既存ROS稼働中。ROS用ビルドは計測後。待機時間を除くスレッドCPU時間で比較。
- 変更前の接触部照合版ソースを`fragment_before/`へ保存。今回の比較では追加設定・統計のABIを揃えるため現行ヘッダと旧実装を使用。変更後はワークスペース実装。先行比較のソース・ログも保持。
- 38回帰テスト成功。変更前は救済の連続確認・中断・添字変更・設定反映4件が失敗、残り34件成功。0.1/1/10倍、長い橋・段差・緩い適合・大きい断片・要求3本の拒否も確認。
- 既存の幅20 m・内部小面幅2 m・法線差1°、0.1/1/10倍とクラスタ順反転、6フレームのID安定も成功。
- 同じ内部小面の高さ10 cm時と、接触部が一致する30°傾斜小面は分離。既存の段差・直交面・密度差・連鎖統合・スケール追従も成功。
- 各方式の3試行で、全150フレームの非時間出力列が一致。
- 救済OFF（`--disable-fragment-merge`）でも、全150フレームの非時間集計列が変更前と一致。ノード単位の完全一致検証とは別。
- GUI上の見た目、車・段差の正解ラベルを使った実bagの誤統合率は未検証。
- 救済追加前の診断：後半100フレームで接続不足18.47対、各側・接触部残差18.76対/フレームで拒否。診断自体の追加では非時間集計が一致。
- 同診断の最終フレームで、小片を含む隣接44対のうち幾何条件を通る4対が1本接続で残存。ID対66/399、21/152、285/399、139/448。接続長比約1.34〜2.15。小断片61個中19個は他平面への直接接続なし。
- 未所属点を1個挟む2対（176/223、285/439）の位置条件適合は救済追加前の診断結果。法線・時間安定性未評価、今回の統合対象外。画面との対応は未確認。
- 追加前の診断資料は`artifacts/plane_consistency_20260924/fragment_diagnosis/`、追加後の最終点群・接続・所属と集計は`fragment_after/last_frame.json`・`statistics.csv`に保存。

現行版の診断（コンテナ内、下記の取得済み`frames.bin`を使用）：

```bash
timeout 100 bash benchmarks/plane_consistency_20260924/build.sh fragment_after
timeout 45 artifacts/plane_consistency_20260924/fragment_after/replay \
  artifacts/plane_consistency_20260924/frames.bin --diagnose \
  > artifacts/plane_consistency_20260924/fragment_after/statistics.csv \
  2> artifacts/plane_consistency_20260924/fragment_after/last_frame.json
timeout 30 python3 benchmarks/plane_consistency_20260924/diagnose_fragments.py \
  artifacts/plane_consistency_20260924/fragment_after/last_frame.json
```

診断用ビルド・実行プロセスは全終了。診断によるROSノード・再生の起動停止なし。

コンテナ内での再現コマンド（既存ROS出力への干渉なし）：

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src
timeout -s INT -k 5 120 python3 benchmarks/plane_consistency_20260924/capture.py \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --config artifacts/plane_consistency_20260924/at128.yaml \
  --output artifacts/plane_consistency_20260924/frames.bin --frames 150
for method in fragment_before fragment_after; do
  timeout 90 bash benchmarks/plane_consistency_20260924/build.sh "$method"
  artifacts/plane_consistency_20260924/"$method"/tests
done
for trial in 1 2 3; do
  for method in fragment_before fragment_after; do
    timeout 45 artifacts/plane_consistency_20260924/"$method"/replay \
      artifacts/plane_consistency_20260924/frames.bin \
      > artifacts/plane_consistency_20260924/"$method"/final_"$trial".csv
  done
done
python3 benchmarks/plane_consistency_20260924/summarize.py --prefix fragment_
```

`fragment_before/tests`の追加4件の失敗は回帰検出の確認結果。保存済み入力の比較では取得コマンドを省略。
ROS用Releaseビルドは`artifacts/plane_consistency_20260924/ros_build`へ分離。
`ais_gng_component_cpu`・`plane_cluster_incremental_node`を再ビルドし、平面38・非平面6・曲面74テストが成功。
CTestでは`LD_LIBRARY_PATH=/ros2_ws/src/artifacts/plane_consistency_20260924/ros_build:$LD_LIBRARY_PATH`を指定し、ビルドしたライブラリを優先。
`bash benchmarks/plane_consistency_20260924/install.sh fragment_runtime_before`でライブラリ2本・実行ファイル1本をinode単位で反映。
反映直前のバイナリは`fragment_runtime_before/`へ保存済み。先行修正時のバックアップも保持。

配布版の分離ドメイン試験：設定宣言値と共有YAMLの照合、間隔5 mm・5 cm・2 mの720ノード長平面、1,706ノードの内部小面統合と高さ10 cm時の分離。内部小面は各6フレーム連続で確認。169ノードの1本接続対は最初の2フレームで分離、3フレーム目で統合、以降も安定。

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 timeout -s INT -k 15 90 \
  python3 /ros2_ws/src/benchmarks/plane_consistency_20260924/smoke.py
```

初回反映版のログは`fragment_smoke.log`、ビルドは`fragment_ros_build.log`、CTestは`fragment_ctest.log`。
最終確認で通常ビルド先の更新を検出。再上書きせず、更新後の配布版で118テスト・ROS試験を再実行して成功。ログは`fragment_current_runtime_ctest.log`・`fragment_current_runtime_smoke.log`。CTestの探索先はこの再試験のみ`/ros2_ws/install/ais_gng/lib`を優先。
初回ROSビルドは整数パラメータの戻り型と`std::max`の引数型不一致で失敗。`std::int64_t`型指定後に成功（`fragment_ros_build_type_error.log`）。
先行修正のCTest旧ライブラリ参照、Python 3.10の配列初期化エラーは解消済み。失敗記録は`contact_ctest_old_library.log`・`contact_smoke_array_error.log`。
先行修正時のDDSドメイン上限・Durability不一致は対応済み。今回もドメイン173・Transient Localを使用。
取得・比較・単体試験・ROS試験プロセスはすべて終了済み。既存のbag再生・GNG・Viewerの停止なし。
試験ノードPID 193463・193475、再試験PID 196297・196309は正常終了。bag・gatewayは開始時と同一PID。検証中にGNG・平面ノード・Frontendの外部再起動を確認し、その後の稼働状態を保持。今回の停止・再起動操作なし、ROSデーモンの新規起動なし。
再取得時のGNG乱数は固定していないため、保存済み`frames.bin`のSHA-256で同一入力を識別。
