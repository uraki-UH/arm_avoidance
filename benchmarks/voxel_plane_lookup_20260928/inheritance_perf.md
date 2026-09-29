# 新規ノードの平面所属継承のコスト比較（2026-09-28）

## 要約

生成世代配列を再利用し、重複した履歴配列65,536 byteと毎入力の書込みを撤去。
新規点フラグはresize後に全要素を上書きし、先行ゼロ初期化を省略。
所属継承・法線補完の判定は維持。大幅な処理時間改善は未確認。

| 固定入力比較 | 変更前 | 変更後 |
| --- | ---: | ---: |
| 平面処理CPU時間・10試行平均 [ms/入力] | 7.149 | 7.094 |
| 出力全フィールドの指紋 | 基準 | 全60フレーム×10試行一致 |

差は約0.055 msで、試行平均自体が約6.6〜9.1 msの範囲にばらつくため速度向上とは断定不可。
局所間隔計算と新規点の最近傍選択を同じループへ統合する試作は不採用。
全エッジへ条件分岐が入る同案は5試行で6.858→6.964 msとなり、撤回。
ユーザーの稼働中ROSでの費用増加の原因は未特定。

所属継承の導入有無だけを変えた追加比較（各5試行）：
GNG平均49.568→49.182 ms、平面7.145→7.176 ms。
平均ノード19,378→19,372、直接追加836.47→837.79個/入力、早期削除213.70→210.36個/入力。
GNG試行平均は導入前47.40〜54.54 ms、導入後48.14〜52.39 msで、2〜3 ms増加は未再現。
同じGNG本体へ、コミット81a74ea216bbcc473fe7057e4440e28ff2f8b18eの平面実装と現行実装を個別リンク。
ROS側の追従重点サンプリング連携なしのため、稼働中ROSの増加を否定する結果ではない。
ROSログのGNG欄はgng_execとログ捕捉初期化の経過時間であり、平面計算本体は別区間。

## 条件・検証

- 保存済み交差点点群から通常GNGで60フレーム分のノード・生成世代・エッジを保存。
- ノード上限20,000、入力voxel 0.5 m、設定は同ディレクトリのparameters.txt / plane_parameters.txt。
- 同一グラフ列をCPU 4固定で再生。最初の20入力を除外した40入力の平均、前後順を交互に実行。
- 固定再生ではGNG学習、ROS配信、Viewer描画、モデル転送を計測対象外。
- 初回の点群からの全処理比較は学習内部のrandom_deviceによりノード列が不一致。
  同一seedでも完全再現とは扱えず、固定グラフ再生へ変更。
- 全処理比較の平面時間は6.648→6.679 msで、有意な改善の根拠なし。
- 固定再生はnode_inputの同一ビルド環境内バイナリ。環境をまたぐ永続互換形式ではない。
- 前後5回の全処理比較は予測40秒・実測33.63秒、固定5回は予測10秒・実測4.67秒。
- 最終固定10回は予測20秒・実測9.79秒。各バッチ全件成功・cleanup_ok確認。
- 最終案の回帰69件成功、ビルド・install成功。起動したビルド・試験は全て終了済み。
- 導入有無の追加10試行は予測40秒・実測36.33秒、全件成功・cleanup_ok確認。本番ソース変更なし。
- 既存ROSの停止・再起動なし。結果は次回GNG起動からの適用。

結果正本：artifacts/voxel_plane_lookup_20260928/inheritance_perf/以下の
batch/report.json（全処理）、replay_batch/report.json（統合案）、history_batch/report.json（最終案）。
before.cppは試験開始時の所属継承ありソースの保存版。

既存gng_cpu_container内、/ros2_ws/srcでのコマンド：

```bash
bash benchmarks/voxel_plane_lookup_20260928/build_inheritance.sh
python3 /tmp/inheritance_perf_run_batch.py benchmarks/voxel_plane_lookup_20260928/inheritance_replay_cases.json --output artifacts/voxel_plane_lookup_20260928/inheritance_perf/history_batch --repeats 10 --timeout-sec 20 --max-total-sec 180 --estimate-sec 1
timeout 180 cmake --build /ros2_ws/build/ais_gng --target test_plane_cluster_incremental -j2
timeout 60 /ros2_ws/build/ais_gng/test_plane_cluster_incremental
timeout 60 cmake --install /ros2_ws/build/ais_gng
```

runnerはrun-benchmark-batchスキル同梱スクリプトのコピー。再試験時は新規outputディレクトリが必要。
追加比較の起動コマンド（同コンテナ・作業ディレクトリ、全試験終了済み）：

```bash
python3 /tmp/inheritance_perf_run_batch.py /tmp/inheritance_gng_cases.json --output artifacts/voxel_plane_lookup_20260928/inheritance_perf/gng_impact_batch --repeats 5 --timeout-sec 30 --max-total-sec 300 --estimate-sec 4
```

結果はgng_impact_batch/report.json。初回ビルドはGitの所有権検査で失敗し、
git -c safe.directory=/ros2_ws/srcで対象を明示した読取りに修正後、ビルド成功。グローバルGit設定変更なし。
