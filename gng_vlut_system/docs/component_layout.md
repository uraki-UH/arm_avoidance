# 軌道計画・回避・把持の部品配置

## 要約

部品の配置と責務。計算部品を`src/core`、ROS入出力を`src/nodes`へ配置。
既存部品の移動であり、全機能の独立ライブラリ化やROS依存の完全除去は未実施。
`planning`に混在していた関節実行・局所回避・把持補正を責務ごとに分離。

| 責務 | 計算・共通処理 | ROS入出力 |
| --- | --- | --- |
| 共通描画 | [libs/arrow_visualization](../../libs/arrow_visualization)：矢印Marker・状態色・購読補助 | GNG、把持表示、Viewerから共用 |
| 到達域 | [core/reachability](../src/core/reachability)：URDFサンプリング | [生成・表示仕様](reachability_maps.md)。経路GNGから独立 |
| 目標タスク | [core/tasks](../src/core/tasks)：指定目標・安全退避の選択部品 | 共有計画ノードから利用 |
| 軌道計画 | [core/planning](../src/core/planning)：グラフ探索、経路コスト、探索索引 | [nodes/planning](../src/nodes/planning)：目標選択・共有計画ノード |
| 回避 | [core/avoidance](../src/core/avoidance)：従来アーム回避の補助処理 | [nodes/avoidance](../src/nodes/avoidance)：`arm_avoid_node` |
| 制御・実行 | [core/control](../src/core/control)：指令補間・速度制限・制御インターフェース | [nodes/control](../src/nodes/control)：目標関節実行・仮想関節ドライバー・指令仲裁 |
| 関節運動状態 | [joint_motion_state.py](../scripts/joint_motion_state.py)：位置・速度・加速度・ジャークの状態と任意の微分計算 | [joint_motion_observer.py](../scripts/joint_motion_observer.py)：標準状態の観測。既定は未起動。[仕様・起動](joint_control.md#任意の運動状態観測) |
| 環境シナリオ | [config/simulation](../config/simulation)：共通物体定義とシナリオ別配置 | [simulation_scenario.py](../launch/simulation_scenario.py)：Harmonic SDF・Isaac USD生成。[選択・追加方法](dual_arm_simulation.md#環境シナリオの管理) |
| シミュレータ接続・表示 | 標準`JointState`・`JointTrajectory`・TF、[共有effort設定](../launch/dual_arm_effort_config.py) | [dual_arm_gz.launch.py](../launch/dual_arm_gz.launch.py)：Harmonic、[dual_arm_isaac.py](../launch/dual_arm_isaac.py)：Isaac標準制御（物理検証待ち）、[gng_viewer_bridge.launch.py](../launch/gng_viewer_bridge.launch.py)：実測購読。接続引数は[標準トピック仕様](dual_arm_simulation.md#標準ros-2トピックとviewerの接続) |
| 把持補正 | [core/grasping](../src/core/grasping)：点群に基づく幅・姿勢補正 | [nodes/grasping](../src/nodes/grasping)：補正入力、IK連携、結果配信 |
| 把持候補・把持物体 | [grasping_system/include](../../grasping_system/include)：candidate・rigid・graph・voxel | [grasping_system/src](../../grasping_system/src)：候補生成等 |
| 衝突判定 | [core/collision](../src/core/collision)：形状・自己・環境の干渉検査 | 利用元の計画・安全監視ノード |
| 点群・ボクセル索引 | [core/indexing](../src/core/indexing)：深度履歴索引・到達領域集計。[共有点群ストア](../../voxel_idx/include/point_cloud_store.hpp)はvoxel_idx | 点群入力・把持補正・ベンチマークの各ノード |
| 占有と安全状態 | [core/safety_engine](../src/core/safety_engine)：VLUT・認識・状態更新 | 安全監視・環境入力の各ノード |
| ロボットモデル | [core/robot_model](../src/core/robot_model)、[core/kinematics](../src/core/kinematics) | モデル配信・各利用ノード |

制御・回避・把持で移動した実装は次の7件。既存アルゴリズム・数値・ROS APIは維持。

- `target_joint_state_executor_node.cpp`：`nodes/planning` → `nodes/control`。
- `virtual_joint_state_driver_node.cpp`：`nodes/planning` → `nodes/control`。
- `arm_avoid_node.cpp`：`nodes/planning` → `nodes/avoidance`。
- `grasp_candidate_refiner_node.cpp`：`nodes/planning` → `nodes/grasping`。
- `grasp_candidate_refinement.hpp`：`nodes/planning` → `core/grasping`。
- `arm_avoid_helpers.hpp`：`core/planning` → `core/avoidance`。
- `joint_state_mux_node.cpp`：`nodes/bridge` → `nodes/control`。

旧ヘッダー2件は利用元なしを確認して削除。新旧配置の互換入口なし。
新しい利用元は新配置を直接include。CMakeのソース指定と把持補正テストも新配置へ更新。
実行ファイル名・コンポーネント名・launch・トピック・パラメータの変更なし。

**追加の冗長コード検査（2026-09-28）**

- 3ノードで重複した名前別関節値変換と、2ノードの関節順序対応を[共通部品](../src/core/control/joint_state_utils.hpp)へ集約。
- 重複名は末尾の有効値、現在角欠損はゼロ、目標角欠損は現在角という既存動作を保持。指令対象名の選定は各ノード側。
- 未参照の`nodes/bridge`・`nodes/self_recognition`・`nodes/safety_monitor`の旧CMake 3件、計115行を撤去。
- 旧CMakeには存在しないmainファイルの参照あり。有効なビルド定義は`src/CMakeLists.txt`へ統一。互換用分岐なし。

| 残る整理候補 | 確認した根拠・分離案 |
| --- | --- |
| 物体テンプレート照合 | `object_template_matcher_node.cpp`は2,354行で型・検索・幾何評価・ROSを同居。照合計算の`core`抽出候補 |
| 計画・回避の共有ノード | `topological_map_planning_node.cpp`は1,818行で試行・安全先読み・経路出力を同居。状態遷移と計画評価の境界整理が必要 |
| launch設定読込み | 3ファイルの`load_root_params`がほぼ同形。欠損キーで空辞書を即返すため、直下`ros__parameters`形式の扱いも要確認 |

深度履歴索引と到達領域集計は`nodes/bridge`から`core/indexing`へ移動し、名前空間を`robot_sim::indexing`へ統一。
`world_point_bucket_index.hpp`は共有点群ストアの転送だけだったため削除。利用元は`point_cloud_store.hpp`と`voxel_idx`を直接参照。
旧ヘッダー・型別名は保持せず、移動対象の計算処理はinclude・名前空間以外の変更なし。

**配置だけでは分離できない範囲**

`topological_map_planning_node.cpp`は計画専用と回避実行の共有実装として維持。
経路探索と通常目標選択は[差し替え部品](planning_components.md)へ分離。試行状態と実行はノード側。
`topological_map_avoidance_helpers.hpp`にも経路・評価出力・試行・安全先読みが混在。
これらの分割には、共有状態と制御権・再計画の境界を整理した上での個別検証が必要。
`grasping_system`全体を移動せず、既存の把持表現パッケージとして維持。
ロボットモデル依存のIK連携は`gng_vlut_system`側の把持ノードに配置。
Pythonシミュレーションは`scripts`内を維持。制御の共通入口は[動作スムージング](motion_smoothing.md)。

## Pythonテストの管理単位

同じ機能のテストを機能単位のファイルへ集約。テスト関数・パラメータ化ケースは維持。

| テストファイル | 対象 |
| --- | --- |
| `test/test_avoidance_motion.py` | 動作選択・部品差替え・復帰継続・停止優先・指令周期 |
| `test/test_pointcloud_avoidance.py` | 機体設定・Viewer環境入力・外部点群・座標変換・鮮度検査 |
| `test/test_dual_arm_launch.py` | 機種別launch設定・頭部深度センサー・光学座標 |
| `test/test_dual_arm_collision_geometry.py` | URDF衝突形状・外接球包囲・自己干渉・退避余裕 |

設定検査の`robot_config`フィクスチャは点群回避テスト内で共有。テストモジュールを経由した設定フィクスチャのimportなし。launchファイルの読込みは起動設定テスト内の`load`へ集約。
この4ファイルは同名のCTestターゲットへ登録。キーボード操作・UDP通信・ROSノードを起動する統合試験は独立のまま維持。

```bash
ctest --test-dir /ros2_ws/build/gng_vlut_system -R '^(test_avoidance_motion|test_pointcloud_avoidance|test_dual_arm_launch|test_dual_arm_collision_geometry)$' --output-on-failure
```

## 条件・検証

- 対象：`gng_vlut_system`のC++部品配置・共通関節値変換・旧CMake・回帰テスト・構成資料。
- 作業前の未コミット変更を保持。基準ブランチの変更・同期・自動commitなし。
- 初回配置変更時の制御2ファイルと共通ヘッダー2ファイルは移動前とバイト単位で一致。制御ノードは後続の重複除去で更新。
- 残るノード2ファイルの変更はinclude先のみ。
- 起動方法は従来どおり。ソースファイルを直接参照する外部スクリプトは新配置への更新が必要。
- 通常ビルド成功。既存PCL_ROOT/CMP0074警告あり、ビルドエラーなし。
- 関節値変換・把持補正・Python補間・C++速度制限のCTest 4対象成功。関節値変換は順序・欠損・重複名・空入力・NaNを検証。
- 索引移動後の通常ビルドと索引・把持補正のCTest 2対象成功。索引対象には点群バケット・深度履歴・到達領域・ボクセル化の既存試験を包含。
- ビルド・試験プロセスは全終了。[進捗記録](progress.md)にも結果を記録。
- 実Gazebo動作の再検証は対象外。新規ROSノード起動と既存プロセス停止は未実施。

検証コマンドはコンテナ内でROSとworkspaceの環境を読み込んだ後に実行。

```bash
colcon build --packages-select gng_vlut_system --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
ctest --test-dir /ros2_ws/build/gng_vlut_system -R '^(test_reachability_voxel_accumulator|test_joint_state_utils|test_grasp_refinement|test_motion_smoothing(_cpp)?)$' --output-on-failure
```
