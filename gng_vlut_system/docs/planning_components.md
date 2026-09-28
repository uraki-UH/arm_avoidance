# 経路計画と目標タスクの差し替え

## 要約

2026-09-28時点。共有計画ノードのGNG経路探索と、通常運転の目標選択を独立した部品へ分離。
既定の探索方式・目標候補優先順・危険目標許可・候補評価式を維持。

```text
ROS入力・状態管理（nodes/planning）
  → 目標タスク（core/tasks）
  → 開始・目標候補の検証と評価（core/planning）
  → graph_planner → plannersの具体実装
  → 既存の接続経路・安全先読み・追従・ROS出力
```

| 管理場所 | 責務 |
| --- | --- |
| `src/core/tasks/goal_task.hpp` | タスク入力・要求型、目標移動・安全退避、順序付き構成と生成 |
| `src/core/planning/graph_planner.hpp` | 計画要求・結果・設定型と抽象インターフェース |
| `src/core/planning/planners/dijkstra_graph_planner.hpp` | Dijkstraの設定、既存一括探索・個別探索の接続 |
| `src/core/planning/graph_planner_factory.hpp` | 実装名と生成処理の対応。追加プランナーの登録場所 |
| `src/core/planning/topological_map_avoidance_helpers.hpp` | 候補検証・採点。具体計画器への依存なし |
| `src/nodes/planning/topological_map_planning_node.cpp` | ROS設定・入出力・実行状態の所有 |

起動時のROSパラメータ。両項目とも読取り専用で、動作途中の変更は拒否。

```yaml
graph_planner: gng_dijkstra
goal_task_components: [requested_goal, safe_retreat]
```

- `gng_dijkstra`：従来の一括探索。計画専用ノードでは固定索引も利用。
- `gng_dijkstra_reference`：開始・目標の組合せごとの個別Dijkstra。比較用であり別アルゴリズムではない。
- `requested_goal`：外部選択済みの目標候補を要求。
- `safe_retreat`：指定目標がなく、現在位置が危険で、既存の退避許可が有効な場合に安全候補を要求。
- 未知の名前は起動エラー。空のタスク配列は通常運転で新規目標を選択しない設定。
- 先に要求を返したタスクを採用。計画失敗時に次のタスクへ進む処理はなし。
- 試行モードは既存の試行目標選択を使用し、`goal_task_components`の対象外。

追加方法：プランナーは`graph_planner<graph_type>::plan`を実装し、factoryへ登録。
ノードや候補評価の変更は不要。要求は開始ID列・目標ID列・危険終端の許可。
結果は開始ID→目標ID→経路ID列。経路は開始・終端を含み、到達不能は欠損または空列。
返却IDの有効性、接続性、設定された安全条件の充足は各実装の責務。
タスクは`goal_task::select`を実装し、生成関数へ登録して構成配列に追加。
タスク入力は指定候補・安全候補・開始状態・退避許可、出力は目標候補と診断名。
独自実装の所有権注入も可能で、共有ノードへの型分岐追加は不要。

## 条件・検証

- 現契約はGNGノード列の経路計画。時刻付き関節軌道や連続空間のRRT・軌道最適化をそのまま受け入れる契約ではない。
- 試行の状態遷移、IK接続、実行追従、再計画、安全監視は既存ノード・補助部品に保持。
- 把持→搬送→開放の工程管理・キャンセル・復旧を扱うタスク実行器は今回の目標選択部品に含まない。
- グラフ寿命と更新排他は利用側の責務。固定索引利用中の形状・エッジ変更には計画器の再生成が必要。
- 旧入口の転送ヘッダー、互換名、動的pluginロードは追加なし。新実装追加時には再ビルドが必要。
- 差し替え試験：独自計画器の候補評価への注入、経路なし時の結果消去、独自タスクの順序・選択。
- 回帰試験：一括／個別探索の経路・危険終端・衝突更新、既存の安全条件・実グラフ比較。
- 初回の新規試験は、到達不能の欠損キーと空列を同一視しない比較で失敗。契約に合わせて試験を修正。
- 通常ビルド成功、CTest 2対象・計12試験成功。ビルド・試験プロセスは全終了。
- 実ROS起動・Gazebo・実機動作は未検証。既存プロセスへの停止操作なし。

検証コマンド（`gng_cpu_container`内でROS Humbleとworkspaceの環境を読込み）：

```bash
colcon build --packages-select gng_vlut_system --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
ctest --test-dir /ros2_ws/build/gng_vlut_system -R '^(test_goal_tasks|test_candidate_metric_availability)$' --output-on-failure
```
