# 平面クラスタの生成・保持・面幅判定と局所スケール追従の比較

## 要約

2026-09-24、交差点bagから生成した同じGNG入力150フレームを修正前後へ入力。
先頭50フレームを除く100フレームの平均を、交互3試行の中央値で比較。
生データ・変更前ソース・ビルド・ログはGit対象外の`artifacts/plane_consistency_20260924/`。
最新の集計値と入力SHA-256は[scale_summary.json](scale_summary.json)。
先行の面幅・保持整合化の比較は[summary.json](summary.json)に保存。

| 項目（1フレームあたり） | 2 cm上限版 | 局所スケール追従版 |
| --- | ---: | ---: |
| 平面処理CPU時間 [ms] | 12.936 | 13.413 |
| 平面所属点数 | 7,110.15 | 11,582.88 |
| 平面所属率 [%] | 37.14 | 60.50 |
| 未所属点数 | 12,033.75 | 7,561.02 |
| 出力クラスタ数 | 105.02 | 107.51 |
| 解放点数 | 4.37 | 5.02 |
| 新規生成数 | 1.71 | 1.50 |
| 分割数（小成分の解放を含む） | 2.27 | 2.10 |

時間は`Clusterizer::update`全体。GNG本体・ROS変換・描画は対象外。
所属は増加。CPU時間は約3.7%増のため、この変更は高速化ではない。
総数の減少や実シーンの全領域での分割正解を示す結果ではない。
長い同一平面の統合、段差・直交面・密度差のある平行面の分離は、正解形状既知の回帰テストで確認。

## 条件・検証

- 入力：`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の`/lidar_points`先頭150件。
- 取得時の`at128.yaml`をコピーしてCPU GNGへ設定。入力voxel 0.5 m、learning 4000、ノード上限20,000。評価区間の平均ノード数19,143.90。
- `capture.py`はbagを読み取り専用で処理。ROSノード・publisher・TF変換なし。ライブViewerと同じ時刻の再現ではない。
- 平面設定：各版の共有ROS既定値、rho種順序再利用OFF。接続要求2本は変更なし。
- 変更後：局所中央値、距離上限なし、取り込み比0.15、保持比0.30、厚み比0.15、両側の相互RMS比0.15。
- ビルド：コンテナ内g++、C++17、`-O3 -DNDEBUG`、Eigen3。CPU固定なし、既存ROS・別ビルド稼働中。待機時間を除くスレッドCPU時間で比較。
- 変更前は先行修正後の2 cm上限版を`scale_before/`へ保存。変更後はワークスペースの実装。先行比較の変更前`f0eea5e4`は`before/`に別保存。
- 31回帰テスト成功。変更前は追加6件が失敗、既存25件成功。0.1〜100倍の生成・保持・取り込み、長い橋エッジ、密度差、段差、明示した距離上限を確認。
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
for method in scale_before scale_after; do
  timeout 90 bash benchmarks/plane_consistency_20260924/build.sh "$method"
  artifacts/plane_consistency_20260924/"$method"/tests
done
for trial in 1 2 3; do
  for method in scale_before scale_after; do
    timeout 45 artifacts/plane_consistency_20260924/"$method"/replay \
      artifacts/plane_consistency_20260924/frames.bin \
      > artifacts/plane_consistency_20260924/"$method"/final_"$trial".csv
  done
done
python3 benchmarks/plane_consistency_20260924/summarize.py --prefix scale_
```

`scale_before/tests`の追加6件の失敗は回帰検出の確認結果。保存済み入力の比較では取得コマンドを省略。
ROS用Releaseビルドは`artifacts/plane_consistency_20260924/ros_build`へ分離。
`ais_gng_component_cpu`・`plane_cluster_incremental_node`を再ビルドし、平面31・非平面6・曲面74テストが成功。
`bash benchmarks/plane_consistency_20260924/install.sh scale_runtime_before`でライブラリ2本・実行ファイル1本をinode単位で反映。
反映直前のバイナリは`scale_runtime_before/`へ保存済み。先行修正時のバックアップは`runtime_before/`に保持。

配布版の分離ドメイン試験（CPUと独立平面ノードの共有YAML設定受付、間隔5 mm・5 cm・2 mのノイズ付き長平面を各720ノード・1クラスタで出力）：

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 timeout -s INT -k 15 90 \
  python3 /ros2_ws/src/benchmarks/plane_consistency_20260924/smoke.py
```

最新ログは`scale_smoke.log`、ビルドは`scale_ros_build.log`、CTestは`scale_ctest.log`。
先行修正時はDDSドメイン234のポート上限と試験publisherのDurability不一致を修正。今回の試験はドメイン173・Transient Localで成功。
取得・比較・単体試験・ROS試験プロセスはすべて終了済み。既存のbag再生・GNG・Viewerの停止なし。
作業中に外部操作で既存GNGが停止・再起動。試験PIDの終了と、開始時のbag・gatewayの同一PID稼働を確認。
再取得時のGNG乱数は固定していないため、保存済み`frames.bin`のSHA-256で同一入力を識別。
