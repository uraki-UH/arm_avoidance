# 重心・共分散による近接平面の統合（2026-09-29更新）

## 要約

接続エッジだけでなく、重心・共分散の広がりの重なりから近接平面を統合候補へ追加。
候補の小平面全体を大平面へ統合し、既存の多数ノード側のIDを維持。
ノードを部分的に奪う処理、ノード挿入条件、ROSパラメータの追加はなし。

- 接平面2軸で「重心差の絶対値」が「双方の標準偏差の合計＋既存の許容幅」内なら候補。内包は不要。
- 候補の大小は所属ノード数で判定。等数同士も対象とし、同一対の重複を除外。
- 法線整合は既存normal_alignment_degを使用。
- 小平面全体から大平面へのRMS距離は既存merge_smaller_side_residual_ratioで確認。
- 統合後の面幅・残差増加・各側の残差など、既存の幾何判定も維持。
- 連鎖統合時は統合済みの共分散で重なりを再確認。
- 面外エッジによる既存の分断根拠が残る小平面は近接候補から除外。
  根拠が消えた内部平面は再統合可能。
- 共分散は外形ではなく、穴・凹形状を含む厳密なhull接触の保証なし。1標準偏差の広がりによる近似。

重心X順のソートと双方の広がりを含む区間検索で遠方対を省略。所属ノードの再累積・再固有値分解による候補探索なし。
区間内候補が集中する場合は最悪クラスタ数の二乗。全体ノード数から独立した一定時間の保証なし。
分断根拠の集計は既存の隣接エッジ走査内で実施。

## 条件・検証

- 2026-09-29：平面テスト71件成功。接続なしの内部・はみ出しパッチ統合、大平面ID保持、全109ノード所属を確認。
- 床・壁、同規模平面、平行段差、遠方面、既存の傾斜面非統合・分断履歴テストを確認。
- 現行の近接版の実bag表示・処理時間は未検証。以下の性能値は2026-09-28の内包版の履歴。
- 旧仕様の「接続まで別平面」の期待値を更新。面外根拠が残る対象の保護を追加後、全件成功。
- 保存済み交差点由来の同一60グラフ、最初20入力を除外、CPU 4固定、前後各5回。
- 平面CPU時間平均7.048→6.682 ms、中央値6.631→6.677 ms。
  導入前平均は8.470 msの試行の影響あり。高速化とは主張しない。
- ROS配信・GNG学習を除く固定グラフ再生。実行中GNG全体への費用と画面上の対象断片は未検証。
- 性能試験10回成功、予測10秒・実測4.78秒、cleanup_ok確認。
- ビルド・install成功。起動したビルド・試験は全て終了、既存ROSの停止・再起動なし。

結果正本：artifacts/voxel_plane_lookup_20260928/inheritance_perf/containment_batch/report.json。
条件：[containment_cases.json](../../../benchmarks/voxel_plane_lookup_20260928/containment_cases.json)。
既存gng_cpu_container内、ROS環境読込み後の起動コマンド：

```bash
timeout 180 cmake --build /ros2_ws/build/ais_gng --target test_plane_cluster_incremental -j2
timeout 60 /ros2_ws/build/ais_gng/test_plane_cluster_incremental
cd /ros2_ws/src
python3 /tmp/inheritance_perf_run_batch.py benchmarks/voxel_plane_lookup_20260928/containment_cases.json --output artifacts/voxel_plane_lookup_20260928/inheritance_perf/containment_batch --repeats 5 --timeout-sec 20 --max-total-sec 180 --estimate-sec 1
timeout 60 cmake --install /ros2_ws/build/ais_gng
```
