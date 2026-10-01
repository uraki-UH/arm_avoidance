# 経路計画と目標タスクの差し替え

## 要約

把持候補の経路計画と、Gazebo回避の動作方策を別の部品として管理。

```text
把持: ROS候補入力 → 候補検証・評価 → graph_planner → 候補経路・評価のROS出力
回避: 観測・安全状態 → motion_flags → 動作選択 → 動作部品 → 共通安全検査 → 軌道出力
```

| 管理場所 | 責務 |
| --- | --- |
| `src/core/planning/graph_planner.hpp` | 計画要求・結果・設定型と抽象インターフェース |
| `src/core/planning/planners/dijkstra_graph_planner.hpp` | 一括探索・個別探索の具体実装 |
| `src/core/planning/graph_planner_factory.hpp` | 実装名と生成処理の対応 |
| `src/core/planning/topological_map_avoidance_helpers.hpp` | 候補検証・採点・経路出力の共通部品 |
| `src/nodes/planning/topological_map_planning_node.cpp` | 把持候補のROS入出力。計画専用 |
| `src/core/tasks/goal_task.hpp` | 目標要求部品のライブラリ。現行計画専用ノードからの使用なし |
| `scripts/avoidance_motion.py` | 回避動作の判定フラグ・優先順位・差替え部品 |
| `scripts/gng_avoidance_planner.py` | 回避探索・復帰方策・共通の区間安全検査 |

計画器の選択は起動時の読み取り専用ROSパラメータ`graph_planner`。

- `gng_dijkstra`: 一括探索と固定索引。
- `gng_dijkstra_reference`: 開始・目標の組合せごとの個別Dijkstra。比較用。
- 未知の名前は起動エラー。

追加方法: `graph_planner<graph_type>::plan`を実装し、factoryへ登録。
要求は開始ID列・目標ID列・危険終端の許可。結果は開始ID→目標ID→経路ID列。
経路は開始・終端を含み、到達不能は欠損または空列。
返却IDの有効性、接続性、設定された安全条件の充足は各実装の責務。

旧実行付き`topological_map_avoidance_node`と試行・目標指令配信処理は廃止。
`goal_task_components`は現行ノードのパラメータではない。
回避・復帰・保持の差替え方法は[点群回避の部品構成](pointcloud_avoidance.md#回避の判定と動作部品)を参照。

## 条件・検証

- 現契約はGNGノード列の経路計画。時刻付き関節軌道や連続空間のRRT・軌道最適化をそのまま受け入れる契約ではない。
- 把持候補ノードは候補経路・評価の配信に限定。関節目標の実行なし。
- 把持→搬送→開放の工程管理・キャンセル・復旧を扱うタスク実行器は今回の目標選択部品に含まない。
- グラフ寿命と更新排他は利用側の責務。固定索引利用中の形状・エッジ変更には計画器の再生成が必要。
- 旧入口の転送ヘッダー、互換名、動的pluginロードは追加なし。新実装追加時には再ビルドが必要。
- 差し替え試験：独自計画器の候補評価への注入、経路なし時の結果消去、独自タスクの順序・選択。
- 回帰試験：一括／個別探索の経路・危険終端・衝突更新、既存の安全条件・実グラフ比較。
- 通常ビルドとCTestの目標要求・候補評価2対象が成功。ビルド・試験プロセスは全終了。
- 実ROS起動・Gazebo・実機動作は未検証。既存プロセスへの停止操作なし。

検証コマンド（`gng_cpu_container`内でROS Humbleとworkspaceの環境を読込み）：

```bash
colcon build --packages-select gng_vlut_system --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
ctest --test-dir /ros2_ws/build/gng_vlut_system -R '^(test_goal_tasks|test_candidate_metric_availability)$' --output-on-failure
```
