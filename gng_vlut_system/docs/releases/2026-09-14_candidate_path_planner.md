# 2026-09-14 - 候補経路生成と回避実行の分離

## Summary

`grasp_joint_candidates.launch.py`から回避ノード起動を除外し、候補経路生成専用ノードへ変更。

## Changed

- `topological_map_path_planner_node`による候補経路・選択経路・評価指標・現在手先姿勢の配信。候補ロボットプレビューの維持。
- 入力変化または明示更新要求に限定した計画更新。同じ候補・関節角度・安全ラベルの反復受信による再探索なし。
- モデル読込み・探索・評価処理を`topological_map_planning_node.cpp`へ移動し、2つの起動入口で共有。実装全体の複製なし。

## Added

- 経路生成専用の実行ファイル・ROSノード`topological_map_path_planner_node`。
- 追加ペナルティ無効時の安全制約、実launchでの実行系分離・静止時更新抑制・明示更新の回帰テスト。

## Fixed

- 候補生成時にも経由点追従・退避・先読み再計画へ入る問題の解消。
- `publish_hz`、`avoid_collisions`、`strict_goal_collision_check`のlaunch引数を専用ノードへ直接転送。

## Removed

- 候補生成launchからの`topological_map_avoidance.launch.py`包含と`replan_on_path_collision`引数。
- 候補探索での隣接衝突・危険ノード数による追加コスト。衝突・危険ノードの通過制約の削除ではない。

## Behavior Impact

- 同じ起動コマンド・既存YAML・出力トピックのままで利用可能。
- 追従・退避・trial・制御指令の処理なし。関節指令・control claimのpublisher生成なし。
- `allow_danger_goal`と既存の目標姿勢スコアは維持。近傍追加ペナルティの除去により経路選択が変わる場合あり。
- 既存の`topological_map_avoidance.launch.py`は独立した実行系として維持。今回、実行系の先読み判定そのものの変更なし。

## Topics / Params / Messages

- 出力トピック名・メッセージ形式の変更なし。`publish_hz`は入力変化確認と計画更新の上限周波数として使用。
- `request_trajectory_update`は維持。`request_trial_goal_advance`は回避ノードのみ。
- 専用ノードの実行禁止は起動クラスで固定。YAMLや実行時パラメータによる実行系への切替なし。

## Verification

Dockerの`gng_cpu_container`内、`/opt/ros/humble/setup.bash`と`/ros2_ws/install/setup.bash`をsource後に実施。

```bash
cd /ros2_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select gng_vlut_system --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1 --event-handlers console_direct+
ctest --test-dir /ros2_ws/build/gng_vlut_system -R 'test_candidate_metric_availability|test_grasp_candidate_reachability' --output-on-failure
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15s 180s python3 -u /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py
```

- Releaseビルド成功。C++回帰テストと既存到達性テスト成功。追加テストの補助グラフに不足していたインターフェースを補完後、再ビルド・再実行で成功。
- 実GNG入力で候補経路・評価・候補ロボット配信、回避ノードとtrialサービスの不在、関節指令・control claimのpublisher不在を確認。
- 静止入力2秒で評価再配信なし。明示更新・現在関節角度変更・候補の領域内復帰による再計画、領域外と空候補での旧出力消去を確認。
- C++テストで近傍ペナルティの有無による経路差、危険ゴール許可、危険ノード通過禁止、衝突による経路遮断を確認。
- 検証用プロセスはすべて停止済み。既存ROSプロセスのPIDとコンテナ起動状態を維持し、ROS daemonの新規残留なし。

結合テスト内の起動コマンド（隔離domain 218、終了時にプロセスグループごと停止）:

```bash
ros2 run gng_vlut_system safety_monitor_node --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml -p gng_model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin -p vlut_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vlut.bin -p base_frame:=ToPoDualArm/base_link -r topological_map:=/ToPoDualArm/topological_map_static
ros2 launch gng_vlut_system grasp_joint_candidates.launch.py params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

## Risk / Notes

- 実環境で入力が連続変化する場合のCPU負荷は未測定。候補と開始ノードの組合せごとのDijkstra探索は既存方式のまま。
- 静的自己干渉フラグへの環境ラベル上書きは既存の[タスク候補](../TASK_CANDIDATES.md)として継続。今回の変更対象外。
