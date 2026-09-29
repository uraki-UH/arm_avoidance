# 2026-09-28 - 部分関節指令とViewer・Gazebo・Dynamixel出力の統合

## 1. 要約

関節名付きの角度指令を共通muxで仲裁し、出力先をViewer・Gazebo・Dynamixelから選択できる構成へ統合。
左グリッパだけを指定して腕・右グリッパを保持し、リーダーの腕指令と同時に使用可能。

- `joint_control.launch.py`を共通起動口として追加。
- `gng_viewer_bridge.launch.py`の標準起動を共通経路へ移行。
- `dual_arm_gng_lidar_demo.launch.py`へ`enable_external_control`・`enable_dynamixel_leader`を追加。
- 関節単位の部分更新、親・mimicの同一所有者化、優先順位、入力失効を実装。
- 単発目標の保持と解除サービスを追加。未指定関節のゼロ上書き、無効入力による部分適用を防止。
- Dynamixel入力の校正表を出力側でも共用し、ID重複をまとめて角度を逆変換。
- 実機フォロワー時の読取り入力を操作指令へ戻さず、Viewerへフォロワー実測値を表示。

現行仕様・起動手順・グリッパ指令例：[共通関節指令経路](../joint_control.md)。

## 2. 条件・検証

| 対象 | 検証結果 |
| --- | --- |
| C++ mux | 9件成功。部分更新・優先度・関節ごとの失効・無効値・解除・mimic競合 |
| Pythonモデル・逆変換 | 12件成功。角度／速度制約・校正・ID重複・未知値・実測mimic偏差 |
| Viewer経路＋Dynamixel模擬入力 | 最終9項目成功。入力→校正→仲裁→表示、グリッパ上書き・解除・入力途絶 |
| 実機形式のROS出力 | 最終7項目成功。左グリッパのみではID18だけに出力、実測失効時の送信停止 |
| Gazebo形式のROS出力 | 最終7項目成功。未指定関節保持・全独立関節の軌道生成・実測失効 |
| 通常のViewer launch全体 | 9項目成功。既存の起動経路から共通制御へ接続 |
| 実Gazebo | 左グリッパ0.2 radへ追従。mimic約-0.2 rad、右グリッパ保持 |

- 実Gazeboでグリッパ以外の関節角変動は最大0.001608 rad。試験実時間は約10.1秒。
- ROS_DOMAIN_ID=88、ROS_LOCALHOST_ONLY=1、Gazeboポート11488で既存処理と分離。
- 実機向け指令は検証専用トピックで受信。USB通信・トルク操作・実モータ駆動は未実施。
- 外部操作モードは自動回避デモの指令元を停止して共通指令へ切替。外部操作への自動回避適用は対象外。
- 以前のODE警告の根本修正、通信遅延下の実機追従性能、機種別の校正は未検証。
- 既存の未コミット変更を保持し、対象workspaceでmuxとテストをビルドしてinstallまで実施。

再現用テストコマンド（Docker内、ROS環境source後、出力先は未作成のパスを指定）：

```bash
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 \
  /ros2_ws/src/gng_vlut_system/test/check_joint_control_ros.py \
  --backend viewer --enable-dynamixel-input --viewer-stack --output /tmp/joint_control_viewer_check
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 \
  /ros2_ws/src/gng_vlut_system/test/check_joint_control_gazebo.py \
  --output /tmp/joint_control_gazebo_check
```

実際の全起動コマンド：[commands.json](../../../artifacts/joint_control_unification_20260928/commands.json)。
ログ・最終結果：[検証成果物](../../../artifacts/joint_control_unification_20260928/)。
全試験プロセスは停止済み。専用ROSドメインの残留0、専用ポートのLISTENなし。
既存プロセスへの停止操作なし。[停止確認](../../../artifacts/joint_control_unification_20260928/cleanup.json)。
