# 2026-09-14 - 候補経路探索の共有

## Summary

候補経路生成の開始候補ごとに、同一の安全条件を持つゴールへのDijkstra探索を共有。安全な5開始候補・8ゴールなら、40回の個別探索から5回の共有探索への変更。

## Changed

- `plan_to_each_node`から、単一ゴール用と共通の探索実装を使用。全対象到達またはキュー枯渇まで探索を継続。
- 探索用の距離・親ノード管理を`std::map`からノードID添字の配列へ変更。
- 開始候補ごとの結果を保持し、既存のゴール順・開始候補順・評価式で選定。同点時の先着選定を維持。
- `plan_from_each_start`で開始候補間の有向エッジ有効判定と基礎コストを共有。実際に調べたエッジだけの遅延キャッシュで、開始候補が1つの場合は生成なし。

## Behavior Impact

- 衝突回避有効時の危険・衝突終点の例外許可、および隣接危険度ペナルティ有効時は個別探索。別ゴールの例外許可を通過許可へ転用せず。
- 個別探索と共有探索の安全制約・エッジコストは共通。現在の候補専用ノードは隣接危険度ペナルティ無効。実行系の既定の個別探索は維持。
- 計画間のキャッシュなし。安全ラベル・グラフ更新時に以前の経路を流用せず。配信周期・候補数の変更なし。
- エッジキャッシュは1回の呼出し内だけに限定。次の計画では有効判定・関節コストを再評価。安全制約・終点別の隣接危険度ペナルティはキャッシュ対象外。

## Topics / Params / Messages

変更なし。Docker内の計画共有ライブラリをビルド済み。既存ノードの再起動操作なし。稼働中の旧版への反映には該当launchの再起動が必要。

## Verification

- C++テスト7件に成功。32通りの安全条件×20ランダムグラフで、同点・ゼロコスト・重複ゴール・非活性ノード等を含む個別経路との一致を確認。危険・衝突終点からの誤通過防止、衝突開始点からの脱出、無効ID、空入力、到達不能、エッジ更新を確認。開始候補間のキャッシュ使用結果も個別経路と一致。
- キャッシュ寿命テストで、2開始候補・2ゴールに共通する4有向エッジの基礎コスト評価が計4回であることを確認。次の計画でのエッジ無効化、危険ノード化、関節コスト変更、隣接関係変更、単一開始候補・空入力への追従も確認。
- 合成10,000ノード: 個別201.69 ms、共有31.37 ms、キュー取り出し回数371,393から72,798。経路一致。
- 保存済み`ToPoDualArm10000/gng.bin`の安全な5開始候補・8ゴールで全40経路一致。追加3回は個別1,346.29 / 1,370.62 / 1,446.57 ms、共有280.38 / 286.57 / 290.68 ms。中央値比約4.8倍。
- エッジキャッシュ追加後の同一実行内比較3回: 前回の共有方式260.733 / 284.842 / 271.269 ms、キャッシュ方式94.065 / 117.014 / 107.392 ms。中央値271.269 msから107.392 msへ約2.5倍、約60%短縮。キャッシュ作成・破棄を含み、全40経路一致。変更直前の共有方式3回は256.259 / 267.311 / 293.793 ms。
- domain 218で既存ROS結合テストに成功。候補プレビュー、領域内候補だけの経路評価、静止入力時の再探索なし、関節更新・明示要求、領域外・空入力でのクリア、復帰時の再計画、関節目標配信なしを確認。

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 360s cmake --build /ros2_ws/build/gng_vlut_system --target test_candidate_metric_availability topological_map_planning -j2 && timeout -s INT -k 5s 120s /ros2_ws/build/gng_vlut_system/test_candidate_metric_availability'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 60s /ros2_ws/build/gng_vlut_system/test_candidate_metric_availability --gtest_filter=candidate_path_batch.actual_robot_graph --gtest_repeat=3'
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 25s 180s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py'
```

検証スクリプト内の起動コマンド:

```bash
ros2 run gng_vlut_system safety_monitor_node --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml -p gng_model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin -p vlut_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vlut.bin -p base_frame:=ToPoDualArm/base_link -r topological_map:=/ToPoDualArm/Tmap_static
ros2 launch gng_vlut_system grasp_joint_candidates.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

すべて隔離ドメイン内。検証launch・子ノード・計測コマンドは終了済み。

## Risk / Notes

- 上記時間は変更後の個別探索と共有探索の比較。旧版バイナリ全体とのCPU比較ではなく、配列化単独の寄与は未測定。
- 危険な終点が多い場合や実行系のペナルティ有効時は共有の効果が限定的。全体CPU使用率と実画面描画の改善率は未測定。
- Dijkstraの前提となる非負・探索中不変のエッジコストを維持。結果保持量は開始候補数・ゴール数・経路長に依存。
- エッジキャッシュは探索中に参照した隣接行のサイズに比例した追加メモリを使用。グラフ・評価器は同一の計画呼出し中に変更しない前提。既存計画ノードはmutex内で探索を実施。
