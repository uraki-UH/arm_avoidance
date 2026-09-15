# 2026-09-14 - 固定GNGの候補経路索引

## Summary

候補経路探索を固定グラフの配列索引・再開可能な最短経路木へ変更。保存済みToPoDualArmの5開始候補・8終点で、初回と安全状態更新後の探索が各5回とも10 ms未満。危険終点を例外許可する条件も検証済み。ROS配信・描画を含む全体遅延や最悪時間の保証ではない。

## Changed

- 候補専用ノードのモデル読込時に、有向隣接と非負の固定関節コストを連続配列へ変換。毎探索のGNGコンテナ参照・関節距離計算を省略。
- 開始候補別に距離・親・確定状態・キューを保持。既到達ゴールは経路復元のみ、追加ゴールには探索を再開。
- 危険・衝突終点を許可する場合は開始候補と終点の組ごとの独立木。危険な別終点を中継点として通過する変更なし。
- 未探索の木を独立ジョブへ分割。CPU数を上限とするワーカーへ分配し、同じ木の同時更新なし。再利用のみの呼出しではワーカー起動なし。

## Behavior Impact

- 各計画でノードの通過可否を照合。活性・自己衝突・環境衝突・危険状態から算出した通過可否に変化があれば全木を失効。現在の開始候補・例外終点に不要な木も破棄。
- `static_edge_cost`対応の評価器だけ索引使用を許可。非対応、非有限・負コスト、隣接危険度ペナルティ有効時は従来方式。実行系の経路探索は変更なし。
- 重み・トポロジーは索引構築後固定の契約。変更時は`prepare_static_graph`が必要。安全状態だけの変更に索引再構築は不要。
- Dijkstraのエッジ順・同点処理、既存候補評価式・選定順を維持。探索近似、候補数削減、周期制限の追加なし。

## Topics / Params / Messages

変更なし。Docker内ビルド済み。稼働中の旧ノードへ適用する場合は該当launchの再起動が必要。作業中に既存ノードの停止・再起動操作なし。

## Verification

16論理CPUのホスト上の`gng_cpu_container`。保存済み`ToPoDualArm10000/gng.bin`、5開始候補、8終点。各条件5回、毎回索引を新規構築し、初回・10回の再利用・安全ノード1個の衝突化後を分けて測定。比較対象は同一バイナリ内の前回方式。危険終点条件はテスト内で8終点を危険状態にした制御条件で、実運用の同時刻スナップショットではない。

| 測定対象 | 安全終点8個 | 危険終点8個（終点のみ許可） |
| --- | --- | --- |
| 前回の計画内エッジキャッシュ | 64.73〜71.02 ms | 175.07〜252.68 ms |
| 起動時索引準備 | 31.27〜37.74 ms | 31.34〜40.51 ms |
| 初回探索 | 2.99〜3.91 ms | 6.33〜8.49 ms |
| 安全状態変更後 | 2.35〜2.93 ms | 6.07〜8.01 ms |
| 再利用50回中の最大 | 0.240 ms | 0.222 ms |
| 初回の総CPU時間 | 12.87〜16.11 ms | 67.13〜74.73 ms |

- 探索計測は`plan_from_each_start`の通過可否生成、失効・初期化、スレッド起動・終了、経路復元を含む。起動時索引準備、候補評価、ROS変換・配信、Viewer描画は含まず。
- C++8件成功。32安全条件×20ランダムグラフで従来個別探索と比較。同点・ゼロコスト・危険終点・衝突・無効ID・重複ゴール等を検証。再開、再利用時の探索回数0、安全状態変更、明示的な索引再構築も検証。
- 実GNGの全40経路は初回・再利用・安全更新後とも従来個別探索に一致。
- domain 218のROS結合テスト成功。候補ロボットの基準座標、混在候補の保持、静止入力での再計画なし、関節変更・明示要求、領域外・空入力でのクリア、復帰時の再計画、関節目標配信なしを確認。
- 読み取り専用プローブでは実運用の6候補すべてが危険ラベルだった時点を確認。その後の同時刻スナップショット取得は候補ID配信元不在により失敗し、未取得ファイルを使う試験も失敗。実運用データによる遅延値とは扱わず、不要な一時読込コードを除去して上記の制御条件で再検証。

実行コマンド（すべて終了済み）:

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 360s cmake --build /ros2_ws/build/gng_vlut_system --target test_candidate_metric_availability topological_map_planning -j2 && timeout -s INT -k 5s 120s /ros2_ws/build/gng_vlut_system/test_candidate_metric_availability'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 90s /ros2_ws/build/gng_vlut_system/test_candidate_metric_availability --gtest_filter=candidate_path_batch.actual_robot_graph --gtest_repeat=5'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && GNG_PLANNING_DANGER_GOALS=1 timeout -s INT -k 5s 90s /ros2_ws/build/gng_vlut_system/test_candidate_metric_availability --gtest_filter=candidate_path_batch.actual_robot_graph --gtest_repeat=5'
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 25s 180s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py'
```

結合テスト内の起動コマンド（隔離domain 218、全子ノード停止済み）:

```bash
ros2 run gng_vlut_system safety_monitor_node --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml -p gng_model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin -p vlut_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vlut.bin -p base_frame:=ToPoDualArm/base_link -r topological_map:=/ToPoDualArm/Tmap_static
ros2 launch gng_vlut_system grasp_joint_candidates.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

読み取り専用プローブは`docker exec -i gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 12s python3 -'`で有限実行。再試行時の上限は18秒。`candidate_goal_label_probe`・`candidate_snapshot_probe`は終了時に破棄済み。既存ROSへのpublish・停止操作なし。

## Risk / Notes

- 10 ms未満は上記検証条件の探索実測。入力規模、開始候補数、例外終点数、CPU競合、OSスケジューリングによる最悪時間の保証なし。
- 並列化は瞬間的に複数コアを使用。総CPU時間は待ち時間より長く、システム全体のCPU改善率は未測定。危険終点の個別木は安全ゴール共有より計算・保持量が大きい。
- 保持量はグラフ・候補数・各探索キューの大きさに依存。全候補・全姿勢間の経路事前計算なし。
- Plannerの同時呼出し・探索中のグラフ更新は非対応。既存ノードのmutex内で実行。重み・隣接変更を安全ラベル更新として扱わないこと。
