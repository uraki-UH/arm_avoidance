# 動作スムージングの管理場所

## 要約

計画・回避・把持を含む全体の管理場所は[部品配置](component_layout.md)を参照。

2026-09-28に既存の動作補間・速度制限を実行環境ごとの共通入口へ集約。
Python・C++・単体HTMLは配布形態が異なるため、異言語をまたぐ実行時依存は追加なし。
この文書を変更箇所の入口とし、デモや実行ノード内への補間式の再追加を回避。

| 対象 | 実装の正本 | 呼出し元 |
| --- | --- | --- |
| Gazeboの5次補間区間・ROS軌道生成 | [motion_smoothing.py](../scripts/motion_smoothing.py) | 通常双腕デモ・局所回避デモ。GNG/LiDARデモは回避デモから継承 |
| C++の関節速度制限 | [motion_smoothing.hpp](../src/core/control/motion_smoothing.hpp) | 関節目標実行ノード・仮想関節ドライバー |
| 単体HTMLの3次補間 | [単体HTML](../../ToPo-FUZZY_Manipulation_v1.html)の`motion_smoothing_cubic_ratio` | 旧IK補間・汎用関節補間・基部と開口の補間、計3箇所 |

Pythonの`quintic_duration`は速度制限から補間時間、`quintic_max_step`は時間から変位上限を算出。
両端の速度・加速度ゼロの5次補間に固有の最大速度係数は同モジュールで一元管理。
`make_rest_to_rest_trajectory`は始終点のROSメッセージ生成のみ。実補間はJointTrajectoryController。
加速度・ジャークの個別制限、更新間の速度継承、衝突回避機能の追加はなし。
C++は`motion_smoothing::calculate_velocity_scale`を直接使用。利用元のない旧APIは削除。
HTMLは単体ファイル配布を維持。共通関数は同じHTML内に配置し、外部JSの読込みは不要。

設定の正本は[通常デモYAML](../config/dual_arm_gazebo_demo.yaml)と[回避YAML](../config/dual_arm_avoidance_demo.yaml)。
動作時間・速度・周期の設定名と値は維持。PythonヘルパーはCMakeで実行スクリプトと同じ場所へinstall。
新しいデモでも同ヘルパーをimportし、メッセージ生成や最大速度係数を複製しない構成。

把持候補のEMA・SLERPは観測ノイズ除去であり、[認識側](../../grasping_system/src/top_grasp_surface_estimator_node.cpp)で管理。
回避候補の評価は[幾何探索](../scripts/dual_arm_avoidance_geometry.py)、GNG経路選択は各計画処理で管理。
これらの最適化と、決定後の関節指令の補間を区別。物理設定は[双腕シミュレーション仕様](dual_arm_simulation.md)。

## 条件・検証

- ROS 2 Humble、`gng_cpu_container`、通常の`/ros2_ws`ビルド先。
- Pythonは共通処理4件・既存URDF動作3件・回避幾何1件の計8件成功。
- C++の速度上限・関節比率・停止姿勢の試験1件成功。
- CMake登録済みのC++・Python両テスト成功。install先から共通モジュールと通常・回避・GNGデモのimport成功。
- GNGデモimport時に既存SciPyとNumPyの版不一致警告あり。読込み成功、GNG実行の互換性評価は対象外。
- HTMLの補間係数1,001点が旧式と完全一致。3呼出し元の移行を確認。
- `colcon build --packages-select gng_vlut_system --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON`成功。
- CMakeの既存PCL_ROOT/CMP0074警告あり。ビルドエラーなし。
- 初回pytestはPYTHONPATHの上書きでrclpy読込み失敗。既存ROSパスへの追記へ修正後、全件成功。
- シミュレーションの実動作・揺れの改善は未検証。今回の変更は既存方式の集約。
- ROSノード・Gazeboの新規起動なし。既存プロセスの停止・再起動なし。ビルド・試験プロセスは全終了。

再検証はコンテナ内でROS環境を読み込んだ後、次のコマンドを使用。

```bash
export PYTHONPATH=/ros2_ws/src/gng_vlut_system/scripts:$PYTHONPATH
python3 -m pytest -q /ros2_ws/src/gng_vlut_system/test/test_motion_smoothing.py /ros2_ws/src/gng_vlut_system/test/test_dual_arm_gazebo_motion.py /ros2_ws/src/gng_vlut_system/test/test_dual_arm_avoidance_geometry.py
ctest --test-dir /ros2_ws/build/gng_vlut_system -R '^test_motion_smoothing(_cpp)?$' --output-on-failure
```

補間メッセージの端点条件・時刻表現・速度上限の維持を回帰試験の対象として保持。
