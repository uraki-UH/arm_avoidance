# 平面クラスタの整合化・局所スケール追従と内部小面統合の比較

## 要約

2026-09-24、交差点bagから生成した同じGNG入力150フレームを修正前後へ入力。
先頭50フレームを除く100フレームの平均を、交互3試行の中央値で比較。
生データ・変更前ソース・ビルド・ログはGit対象外の`artifacts/plane_consistency_20260924/`。
最新の集計値と入力SHA-256は[contact_summary.json](contact_summary.json)。
先行の面幅・保持整合化は[summary.json](summary.json)、スケール追従は[scale_summary.json](scale_summary.json)に保存。

| 項目（1フレームあたり） | 全域の相互照合版 | 接触部照合版 |
| --- | ---: | ---: |
| 平面処理CPU時間 [ms] | 13.374 | 13.094 |
| 平面所属点数 | 11,582.88 | 11,522.08 |
| 平面所属率 [%] | 60.50 | 60.19 |
| 未所属点数 | 7,561.02 | 7,621.82 |
| 出力クラスタ数 | 107.51 | 80.68 |
| 解放点数 | 5.02 | 5.43 |
| 新規生成数 | 1.50 | 1.57 |
| 分割数（小成分の解放を含む） | 2.10 | 2.26 |

時間は`Clusterizer::update`全体。GNG本体・ROS変換・描画は対象外。
平面所属率はほぼ維持、クラスタ数は減少。CPU時間の差は約2.1%で試行間の揺らぎあり。
クラスタ総数の減少は、実シーンの全領域での分割正解を示す結果ではない。
長い同一平面の統合、段差・直交面・密度差のある平行面の分離は、正解形状既知の回帰テストで確認。

## 条件・検証

- 入力：`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の`/lidar_points`先頭150件。
- 取得時の`at128.yaml`をコピーしてCPU GNGへ設定。入力voxel 0.5 m、learning 4000、ノード上限20,000。評価区間の平均ノード数19,143.90。
- `capture.py`はbagを読み取り専用で処理。ROSノード・publisher・TF変換なし。ライブViewerと同じ時刻の再現ではない。
- 平面設定：前後で同じ共有ROS既定値、rho種順序再利用OFF。接続要求2本・距離上限なし・各距離比の設定変更なし。
- 相互照合を直接接続エッジの端点へ限定。別途、各側全体の統合後平面へのRMS比を検査。各評価点群の局所間隔平均で正規化し、比0.15を適用。
- 接続端点の累積統計を既存エッジ走査で作成。対ごとの全点再走査・追加の固有値分解なし。連鎖統合時は更新された成分平面との照合。
- ビルド：コンテナ内g++、C++17、`-O3 -DNDEBUG`、Eigen3。CPU固定なし、既存ROS稼働中。ROS用ビルドは計測後。待機時間を除くスレッドCPU時間で比較。
- 変更前はスケール追従済みの全域相互照合版を`contact_before/`へ保存。変更後はワークスペースの実装。先行比較のソース・ログは`before/`・`scale_before/`等に保持。
- 33回帰テスト成功。変更前は新規の内部小面統合1件が失敗、残り32件成功。幅20 m・内部小面幅2 m・法線差1°、0.1/1/10倍とクラスタ順反転、6フレームのID安定を確認。
- 同じ内部小面の高さ10 cm時と、接触部が一致する30°傾斜小面は分離。既存の段差・直交面・密度差・連鎖統合・スケール追従も成功。
- 各方式の3試行で、全150フレームの非時間出力列が一致。
- GUI上の見た目、車・段差の正解ラベルを使った実bagの誤統合率は未検証。

コンテナ内での再現コマンド（既存ROS出力への干渉なし）：

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src
timeout -s INT -k 5 120 python3 benchmarks/plane_consistency_20260924/capture.py \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --config artifacts/plane_consistency_20260924/at128.yaml \
  --output artifacts/plane_consistency_20260924/frames.bin --frames 150
for method in contact_before contact_after; do
  timeout 90 bash benchmarks/plane_consistency_20260924/build.sh "$method"
  artifacts/plane_consistency_20260924/"$method"/tests
done
for trial in 1 2 3; do
  for method in contact_before contact_after; do
    timeout 45 artifacts/plane_consistency_20260924/"$method"/replay \
      artifacts/plane_consistency_20260924/frames.bin \
      > artifacts/plane_consistency_20260924/"$method"/final_"$trial".csv
  done
done
python3 benchmarks/plane_consistency_20260924/summarize.py --prefix contact_
```

`contact_before/tests`の追加1件の失敗は回帰検出の確認結果。保存済み入力の比較では取得コマンドを省略。
ROS用Releaseビルドは`artifacts/plane_consistency_20260924/ros_build`へ分離。
`ais_gng_component_cpu`・`plane_cluster_incremental_node`を再ビルドし、平面33・非平面6・曲面74テストが成功。
CTestでは`LD_LIBRARY_PATH=/ros2_ws/src/artifacts/plane_consistency_20260924/ros_build:$LD_LIBRARY_PATH`を指定し、ビルドしたライブラリを優先。
`bash benchmarks/plane_consistency_20260924/install.sh contact_runtime_before`でライブラリ2本・実行ファイル1本をinode単位で反映。
反映直前のバイナリは`contact_runtime_before/`へ保存済み。先行修正時のバックアップも保持。

配布版の分離ドメイン試験：共有YAML設定受付、間隔5 mm・5 cm・2 mの720ノード長平面、1,706ノードの内部小面統合と高さ10 cm時の分離。内部小面は各6フレーム連続で確認。

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 timeout -s INT -k 15 90 \
  python3 /ros2_ws/src/benchmarks/plane_consistency_20260924/smoke.py
```

最新ログは`contact_smoke.log`、ビルドは`contact_ros_build.log`、CTestは`contact_ctest.log`。
初回CTestは旧インストール先の優先で失敗。`ldd`で確認し、上記探索パスの指定後に成功。失敗ログは`contact_ctest_old_library.log`。
初回ROS試験はPython 3.10の`array.array.clear()`非対応で失敗。スライス削除へ変更後に成功。失敗時も子ノード停止済み（`contact_smoke_array_error.log`）。
先行修正時のDDSドメイン上限・Durability不一致は対応済み。今回もドメイン173・Transient Localを使用。
取得・比較・単体試験・ROS試験プロセスはすべて終了済み。既存のbag再生・GNG・Viewerの停止なし。
試験PIDの終了と、開始時のbag・gateway・GNG・平面ノードの同一PID稼働を確認。Frontendは外部操作で再起動、今回の操作による停止なし。
再取得時のGNG乱数は固定していないため、保存済み`frames.bin`のSHA-256で同一入力を識別。
