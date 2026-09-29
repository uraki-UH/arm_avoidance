# 共通の関節指令経路

## 構成

関節名と角度を指定する入口を共通化し、仲裁後の出力先だけを切り替える。

```mermaid
flowchart LR
  A[手動の部分関節指令] --> M[関節単位の共通mux]
  B[グリッパの単発目標] --> M
  C[Dynamixel実測値からのリーダー入力] --> M
  D[既存の計画・退避ノードのclaimと指令] --> M
  M --> O[共通の制約・補間・出力]
  O --> V[Viewerの仮想姿勢]
  O --> G[GazeboのJointTrajectory]
  O --> H[DynamixelのGoal Position]
  G --> F[フォロワー実測値]
  H --> F
  F --> W[Viewer表示]
```

`joint_control.launch.py` がmuxと出力ノードを一組だけ起動する。
出力先は `backend:=viewer / gazebo / dynamixel`。既定はviewer。
同じロボット名前空間で複数の出力先を同時起動しない。
`gng_viewer_bridge.launch.py`もこの共通launchを使用する。
既存の`virtual_joint_state_driver`・`target_joint_state_executor`・UDP送信ノードは旧経路であり、共通経路と併用しない。

## 指令の統合

トピックはロボット名前空間配下。入力型は`sensor_msgs/msg/JointState`、角度はrad。

| 入力 | 優先度 | 保持・失効 | 対象 |
| --- | ---: | --- | --- |
| `joint_commands` | 50 | 単発目標を解除まで保持 | 指定した関節のみ |
| `gripper_commands` | 200 | 単発目標を解除まで保持 | URDFの独立グリッパ関節のみ |
| `leader_joint_states` | 100 | 最終受信から0.5秒で関節ごとに失効 | 指定した関節のみ |
| 既存の`control_claims`登録元 | claim指定値 | 既定1.0秒、`command_timeout_sec`で変更 | claimの範囲内 |

- 同じ指令元の部分更新は、指定しなかった関節の目標を上書きしない。
- 腕のリーダー追従とグリッパの単発目標を合成可能。
- 競合判定は関節単位。既存契約どおりexclusive優先、その次にpriorityの大きい指令。
- 同順位・同モードの場合は入力トピック名の辞書順。受信タイミングによる揺れを回避。
- 親とmimicは同じ関節として仲裁。高優先度の親目標と低優先度のmimic目標の競合を解消。
- 未知関節・名前重複・配列長不一致・NaN/Inf・claim範囲外・矛盾するmimic目標はメッセージ全体を拒否。
- 無効入力や未指定関節からゼロ目標を生成しない。
- `target_joint_states`は保持した目標の確認用、`active_joint_commands`は現在有効な目標。どちらもmuxの出力専用。
- 新しい出力ノードは`active_joint_commands`を購読。失効したリーダー目標へ進み続けず、下位の有効指令または保持へ移行。
- 実機・Gazeboで有効指令がなくなった関節は実測姿勢を保持。Viewerは直前の表示姿勢を保持。
- 手動目標解除は`joint_control/release_manual`、グリッパ目標解除は`joint_control/release_gripper`。型は`std_srvs/srv/Trigger`。
- 解除後も次の単発指令を受付。独自のclaim登録元は`enabled=false`で解除し、再開時に再登録。

## 起動

以下はDocker `gng_cpu_container` 内でROS環境をsourceした後の例。
Dynamixelを使う場合、USB接続・トルク・制御モードを設定した`dynamixel_handler`を別途起動し、`/dynamixel/state/present`を配信する。
共通launchからドライバのトルク・制御モードは変更しない。

Viewerを実機の手動操作へ追従させる場合：

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py enable_dynamixel_input:=true
```

Gazeboをフォロワーにする場合。同じデモの従来起動を終了してから切り替える。

```bash
ros2 launch gng_vlut_system dual_arm_gng_lidar_demo.launch.py \
  enable_external_control:=true enable_dynamixel_leader:=true
```

このモードでは自動回避デモの指令ノードを起動せず、共通経路がコントローラへの指令を担当する。
LiDAR・GNG/VLUTの観測処理は維持。外部指令に自動回避を掛ける機能ではない。
ロボット名前空間は通常`sim_topo_dual_arm_max`。
Dynamixelを接続せずROS指令だけで操作する場合は`enable_dynamixel_leader:=false`。

Dynamixel実機をフォロワーにしてViewerへ実測姿勢を表示する場合：

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py joint_control_backend:=dynamixel
```

この場合、内蔵する`dynamixel_joint_state_bridge`は実機状態の読取り専用で、実測値を操作指令へ再投入しない。
同じ名前空間に従来のDynamixel bridgeを重複起動しない。
別のリーダーを使う場合、その実測値を当該フォロワーの`leader_joint_states`へ渡す。
既存の外部状態変換を使う場合は`enable_dynamixel_input:=false`で内蔵入力を停止可能。

既存の制御経路が別launchで稼働しているときにViewerだけ追加する場合は、`joint_control_backend:=external`で二重起動を回避する。
`params_file`・`urdf_path`・ロボット名・入力名前空間は対象機種に一致させる。

## グリッパだけの指令

Gazeboの左グリッパを0.2 radへ動かす例。腕や右グリッパの角度は送らない。

```bash
ros2 topic pub --once /sim_topo_dual_arm_max/gripper_commands sensor_msgs/msg/JointState \
  '{name: [L_gripper_joint], position: [0.2]}'
```

リーダー側へグリッパの制御を戻す例：

```bash
ros2 service call /sim_topo_dual_arm_max/joint_control/release_gripper std_srvs/srv/Trigger '{}'
```

同様に`joint_commands`で腕の一部だけを指定できる。リーダーと同じ関節が競合した場合は上表の優先度を適用する。

## 出力・校正・実測

- 共通モデル：URDFの可動範囲・速度・mimicを正本として使用。未指定関節は保持。
- Viewerのみ`enable_direct_tracking:=true`で直接表示可能。物理出力ではURDFと`max_joint_velocity`の速度制限を適用。
- Gazebo：独立した全コントローラ関節の実測値が揃ってから`dual_arm_controller/joint_trajectory`へ送信。未指令関節は実測姿勢で初期化。
- Dynamixel：目標を受けたモータIDだけへ`/dynamixel/command/goal`を送信。型は`DynamixelGoal`。mimicと親が同じIDでも一度だけ送信。
- 対応表は既存の`dynamixel_joint_state_bridge.yaml`を共用。`mapping_file`、Viewer起動では`dynamixel_mapping_file`で差替え可能。
- 逆変換は`position_deg = degrees(joint_angle_rad) / joint_scale - joint_offset_deg`。読み取り側と同じ校正の逆変換。
- 独立関節へのID重複、矛盾した親・mimicの校正、ID未割当関節への実機指令は拒否。
- 現在の対応表にはwaist_jointのIDなし。実機で腰を操作する場合は校正済み対応を追加する。
- グリッパの倍率を含む校正とモータの許容角・制御モードは機種ごとの確認が必要。実機送信は今回未実行。
- フォロワーの状態は`joint_states`から取得し、`viewer_joint_states`へ表示。物理出力でこの2トピックを同一にする設定は拒否。
- 実測失効時は送信を停止。これはトルクOFFではなく、モータへ既に渡した目標の取り消しも行わない。
- mux停止時も`max_command_age_sec`（既定0.5秒）で古い目標を失効。実測期限は`max_state_age_sec`（既定1秒）。
- 状態確認は`joint_control/status`。物理シミュレーションの安定性や従来のODE警告修正は別の課題。

## 検証

純粋な仲裁・変換テストは`test_joint_command_mux`・`test_joint_command_model`。
ROS通信試験は`test/check_joint_control_ros.py`、実Gazebo試験は`test/check_joint_control_gazebo.py`。
試験専用ROS_DOMAIN_ID=88が必要。実機向け指令は専用トピックへ変更し、USBドライバを起動しない。
実施結果は[リリースノート](releases/2026-09-28_unified_joint_control.md)を参照。
