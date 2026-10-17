# Dynamixel・シミュレータの追従

既定はLong、実機出力OFF。USBドライバは別起動。
首の電流制御は[dynamixel_neck_torque.md](dynamixel_neck_torque.md)を参照。

## s・r・fの共通追従構成

s=Simulator、r=実機リーダー、f=実機フォロワー。

```bash
ros2 launch gng_vlut_system robot_follow.launch.py profile:=r_display
```

設定は[robot_follow.yaml](../config/robot_follow.yaml)を参照。
ブラウザ「ROS2連携 → リーダー・フォロワー接続」で選択・適用。
描画追従は実測角の表示、力学追従はMuJoCoの目標入力。選択だけで物理・実機出力は開始しない構成。

実測変換は[dynamixel_joint_state_input.yaml](../config/dynamixel_joint_state_input.yaml)、実機監視は[topodualarm_hardware_test.yaml](../config/topodualarm_hardware_test.yaml)を参照。
既存の実測変換・Simulator・Viewerを重複起動しないこと。同じ対象への入力は1個、循環経路は禁止。

実機出力には明示許可と確認済み電流上限が必要。GUIは出力準備→保持確認→追従開始。
構成変更・入力失効・切断後は停止し、復旧だけでは再開しない。sからfへ渡す値は物理実測であり、MuJoCo目標値ではない。

## 起動と操作

ROS環境をsourceした対話TTYで:

```bash
ros2 launch gng_vlut_system dynamixel_sim_control.launch.py enable_gazebo:=true
```

設定は[topodualarm_hardware_test.yaml](../config/topodualarm_hardware_test.yaml)を参照。

| キー | 操作 |
| --- | --- |
| H | 実機出力の有効化／ソフト停止。保持目標の読返し後、選択IDのtorque ON |
| J / K | 選択した単関節の+1° / −1°。URDF座標での符号 |
| F | Gazebo補間目標への追従ON/OFF。追従OFFは現在姿勢保持 |
| A | Gazebo回避ON/OFF |
| Space | 実機とGazebo両方へのソフト停止要求 |
| E | 選択IDのtorque OFF・位置指令遮断ラッチとGazebo停止。腕の支持が必要 |
| R | 実機の実測停止確認後、ラッチ解除・出力OFF。再開には改めてH |
| B | Gazeboの実測停止確認後、ラッチ解除・保持。回避再開には改めてA |
| Ctrl+C | 両系統への停止要求後、端末と新launchの終了 |

保持停止とトルクOFFは別。受付成功と実測停止も別。
EによるトルクOFFには腕の支持が必要。OFF報告は最新レジスタ読取り・電源遮断の保証ではない。

## 通常動作の電流制限

許容電流は機種・支持・荷重に基づく明示設定。未設定では有効化不可。
電流目標の制限であり、関節トルク・接触力の直接制限ではない。
EEPROMのCurrent Limit・モード・ゲインは自動変更しない構成。

## 実機小動作の準備

実機試験は未実施。ID・符号・原点・支持・独立停止手段を確認。
制御監視は同時刻の位置・速度を持つfresh_joint_statesとstatus／goal／extra。キャッシュ済みpresentは代用不可。
対象・小動作範囲は[設定](../config/topodualarm_hardware_test.yaml)を参照。

```bash
ros2 launch gng_vlut_system dynamixel_sim_control.launch.py \
  allow_hardware_output:=true 'joint_names:=[L_joint7]' \
  max_current_ma:=${DYNAMIXEL_MAX_CURRENT_MA:?確認済みの許容電流mAを設定}
```

保持準備後に単関節小動作・停止を確認。PC・USB断時の独立非常停止を保証する機能ではない。

## Gazebo指令への追従

```bash
ros2 launch gng_vlut_system dynamixel_sim_control.launch.py \
  enable_gazebo:=true allow_sim_follow:=true
```

実機出力許可は別設定。転送元はcontroller_stateのreference.positionsで、Gazebo実測ではない。
Hで保持準備後、姿勢差を確認してF。実機側とGazebo側の停止解除は独立。
設定・制約は[pointcloud_avoidance.md](pointcloud_avoidance.md)・[dual_arm_simulation.md](dual_arm_simulation.md#gazeboのソフト停止)を参照。

## 重力補償・保持付きの手動操作

```bash
ros2 launch gng_vlut_system dynamixel_hand_guiding.launch.py
```

設定は[dynamixel_hand_guiding.yaml](../config/dynamixel_hand_guiding.yaml)・[dynamixel_adaptive_hold.yaml](../config/dynamixel_adaptive_hold.yaml)を参照。
対象はfollower／leader／neck、方式はgravity／adaptive_hold。
初期は出力なしの計算プレビュー。実機有効化には対象別の質量・重心・符号・電流換算の校正が必要。

adaptive_holdの操作検出には操作許可信号か、校正済み外力推定が必要。Dynamixelの生電流を外力として直接入力しないこと。
重力補償はcurrentモード、位置追従はcur_positionモードで別用途。同じhandlerへの同時指令は禁止。
異常・終了時は所有軸のゼロ電流・OFFを要求。落下防止・実機での重力支持は未検証。

## 実機リーダー・フォロワー制御

### 起動と設定

```bash
ros2 launch gng_vlut_system dynamixel_joint_state_input.launch.py
ros2 launch gng_vlut_system dynamixel_leader_follower.launch.py
```

実測入力はViewer入口が配信済みなら追加不要。
設定は[dynamixel_joint_state_input.yaml](../config/dynamixel_joint_state_input.yaml)・[dynamixel_leader_follower.yaml](../config/dynamixel_leader_follower.yaml)を参照。
初期は出力OFF。実機出力は校正・実測静止・保持目標等の読返し後に有効化。
グリッパー・首の実機追従は対象外。ドライバの起動・終了時のトルク設定は別管理。

### 操作

H=保持準備、F=追従、Space=保持停止、E=対象軸OFF、R=停止解除・出力OFF。
終了時は所有出力のOFF要求。通信復旧だけで追従を再開しないこと。

### 実機フォロワーの手動姿勢確認

実機出力OFFで /follower/joint_states をブラウザの実測表示へ入力。
設定は[Max／Longの対応表](../../hardware_drivers/dynamixel/dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge_max_ids_31_52.yaml)を参照。
present入力は描画用。制御監視はfresh入力を使用し、表示のクランプを実測値の補正として扱わないこと。

### ブラウザの物理フォロワー

「ROS2連携」でシミュレーション／ROSリーダー姿勢／位置制御を選び、明示開始。
腕は開始時からの角度差、グリッパーは校正済み開度に追従。実機出力OFFでも使用可能。
設定・停止条件は[ROS連携](../../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md#姿勢tfのwebsocket送受信)を参照。

### 検証

隔離ROS・疑似モータ・ブラウザ／MuJoCoの接続を検証済み。実USB／実機駆動は未検証。
再現入口は[test_robot_follow_model.py](../test/test_robot_follow_model.py)・[test_dynamixel_leader_follower.py](../test/test_dynamixel_leader_follower.py)。
