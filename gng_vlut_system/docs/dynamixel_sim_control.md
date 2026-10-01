# ToPoDualArmのDynamixel単独動作・Gazebo指令転送

パンチルトだけを元の角度へ戻さず動かす用途: [ID51・52専用の電流抵抗・終了時OFF](dynamixel_neck_torque.md)。以下の位置保持・Gazebo追従とは別launch。

通常動作の対象: XM430-W350・XM540-W270、電流ベース位置制御・速度基準profile。初期対象はユーザー指定の左腕7関節（ID41〜47）。旧launch・旧設定は保持。

## 起動と操作

コンテナ内の対話端末で、ROSとworkspaceをsource後に起動。

```bash
ros2 launch gng_vlut_system dynamixel_sim_control.launch.py enable_gazebo:=true
```

端末表示はGazebo状態行と操作キーの2行のみ。状態変化時に更新。起動前の`ros2 launch`自身の案内も消す場合の実環境入力付きコマンド:

```bash
ros2 launch gng_vlut_system dynamixel_sim_control.launch.py \
  enable_gazebo:=true gui:=true allow_hardware_output:=false \
  input_config:=/ros2_ws/src/gng_vlut_system/config/viewer_environment_gazebo_input.yaml \
  >/dev/null 2>&1
```

Gazebo状態は標準出力ではなく操作TTYへの直接出力。実機状態・操作結果・ノードの診断はROSログ側へ保存。キー操作と停止処理は従来どおり。停止時の理由は同じ状態行の`理由=`に表示。Aで開始条件を満たさず拒否された場合も停止へ遷移。点群余裕不足などの原因解消後、静止確認とBによる解除を経て、holdからAで再要求。


起動内容: ToPoDualArm左腕の点群・GNG回避用Gazebo・Viewer配信・共通操作端末。初期状態は保持、実機出力OFF。`allow_hardware_output:=false`ではDynamixel指令publisherの生成なし。既存のUSBドライバ・RealSense・Viewerサーバーの起動／再起動なし。

TF受信: `topofuzzy_bridge_node`の入力・出力フレームが同一の場合、`/tf`・`/tf_static`の購読なし。既定ToPoDualArmのGNG配信ではGazebo再起動によるTF時刻後退の影響なし。異なるフレーム間の変換時のみ従来のTF Bufferを使用。修正の反映には配信元の`gng_viewer_bridge.launch.py`の再起動が必要、Gazebo側だけの再起動では旧プロセスが残留。[検証条件](releases/2026-10-01_topofuzzy_tf_restart.md)。

Gazebo開始姿勢: 左肩`L_joint1 = −π/4 rad`（−45°）、ほかの独立関節0。正面斜め下45°へ左腕を伸ばした姿勢で保持し、Aで回避開始。回避開始時の実測姿勢が復帰目標であり、この姿勢での開始時は同じ斜め下45°へ復帰。開始姿勢から既に点群へ接触している場合は開始拒否。稼働中のGazeboへの即時反映なし。

前方伸展設定の回避: `max_joint_velocity: 1.2` rad/s、`control_period_sec: 0.05` s。前者は5次補間の指令速度上限であり、常時その速度で動く指定ではない。実速度は退避量・経路・物理追従・Gazebo実時間比に依存。実機の小動作・電流・追従の制限とは別設定。調整後はGazebo launchの再起動が必要。

物理追従の制限: 比較当時の`physics_solver: quick`でToPoDualArmの大きな追従遅れを確認。同一左肩指令の`world`比較では最大実測速度0.127 → 1.183 rad/sに改善したものの、別の水平姿勢・停止試験でODE数値異常と速度超過を確認し、設定への採用は見送り。トルク上限・ゲインの変更なし。[条件・結果](releases/2026-10-01_gazebo_avoidance_speed.md#続報-トルク設定と物理ソルバーの追従比較)。

計画元: `enable_native_planner: true`では既存`topological_map_avoidance_node`のC++退避計画。Gazeboの関節状態と環境側`Tmap_static`が入力、`/sim_ToPoDualArm/native_avoidance_target`が出力。自律退避先は自身と直接隣接がすべて安全なノードへ限定し、その中から関節距離順32個を候補化。現在ノードが安全でも隣接に危険・衝突があれば退避対象。退避先の隣接悪化時は再選定、現在位置の隣接危険だけによる退避経路の反復取消しはなし。経路途中はノード自身の危険・衝突を禁止し、隣接までの安全を必須とするのは退避先。Python側で関節速度・点群との区間余裕を検査し、使用できない中間目標には局所補正。点群目標距離だけの達成では退避終了せず、現在姿勢の最寄りノードと直接隣接の安全も必要。入力失効・停止ラッチは維持。

外部環境の更新: 初回`Tmap_static`から固定トポロジーを取得後、時刻・トポロジー照合値付き`gng_node_states_stamped`で安全ラベルを更新。新しい環境入力launchの起動が必要。学習グラフ差替え時はGazeboも再起動。

前方伸展設定の復帰判定: 実測関節に最も近いGNGノードと、辺で直接つながる隣接ノードのラベルがすべて安全であること。衝突・危険・ラベル欠測時は復帰不許可。旧`min_return_clearance_th`の固定距離条件は廃止。点群接近時は0.05 mを目標に退避を優先（離散化許容1 mm）。停止距離は共通設定の`clearance_margins.min_clearance_th`（現設定0.01 m）。退避後、隣接安全と復帰経路の停止余裕を確認して`returning`へ移行し、Aで保存した実測姿勢へ復帰。復帰中は隣接ラベルと経路を再検査し、点群余裕の減少だけで退避補正へ戻す処理は撤去。関節誤差0.015 radで`monitoring`へ移行。Space停止後の自動復帰なし。

停止マージンの設定箇所: [pointcloud_avoidance_common.yaml](../config/pointcloud_avoidance_common.yaml)の`clearance_margins`。点群の開始・停止・経路下限を`min_clearance_th`へ統一。自己干渉・床・作業台の停止余裕は`min_internal_clearance_th`、計画時の内部余裕は`min_planning_clearance_th`。現在値は順に10・5・10 mm。[適用範囲と移行方法](pointcloud_avoidance.md)。

実環境入力設定の点群待機: `viewer_environment_gazebo_input.yaml`は`enable_obstacle_auto_resume: true`。実行中の点群余裕不足では`obstacle_wait`へ移行し、その時点の姿勢を保持。点群余裕50 mmとGNG最寄り・直接隣接の安全が`resume_clear_sec: 0.3`秒継続後、B・Aの再操作なしで同じ回避実行を再開。復帰先は当初Aで保存した姿勢。端末表示は`回避=障害物待ち（離れたら自動再開）`。Space・入力失効・自己干渉・関節異常は従来の停止ラッチ対象、B解除後にAで明示再開。開始時点での距離不足は引き続き拒否。既存の`software_stop`の自動解除はなく、変更反映はGazebo launch再起動後。

現時点の制限: 隣接安全ラベルと実測点群の近接が一致しない試験ケースあり。固定距離条件の修正だけで回避・復帰の通し成功を保証する状態ではない。[失敗条件・速度測定](releases/2026-10-01_avoidance_latency.md)。

既定の機体設定: `pointcloud_avoidance_topodualarm_left_forward.yaml`。旧ゼロ姿勢は`robot_config:=/ros2_ws/src/gng_vlut_system/config/pointcloud_avoidance_topodualarm.yaml`で選択可能。共通`pointcloud_avoidance.launch.py`単独起動では前方伸展設定を`robot_config`へ明示。初期位置は`overrides.initial_joint_positions`、単位rad・m、未指定は0、mimic関節は親関節から算出。Gazeboモデル生成中の物理一時停止時だけ位置を設定し、物理開始後は従来の有限トルクモータで追従。

`dual_arm_gazebo_demo.launch.py enable_auto_start:=true`はデモ自動再生用。新launchには実機の自動開始なし。

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

停止要求の受付と実測停止を別表示。実機停止確認は新しい位置・速度の受信と低速継続0.3秒に基づく判定。速度レジスタが0でも、位置差分に移動があれば静止継続時間をリセット。ソフト停止は保持目標への変更とラッチであり、torque OFFではない。R・終了時にも自動torque OFFなし。

GazeboのB解除: 全回転関節0.01 rad/s・直動関節0.001 m/sの低速条件をシミュレーション時間で0.25秒継続し、実測・操作端末の鮮度と処理待ち完了も確認。拒否理由は停止理由を残したまま状態行の`B=`へ表示。GazeboのA・Bは実機状態配信の鮮度に非依存。Gazebo自身の解除条件は維持。

Eは`allow_hardware_output:=true`の場合に送信。H前・実測欠測中でも利用可能。E後はSpace・Gazebo停止通知・終了処理でも保持指令の再送なし。OFFを繰返し要求し、Rによる解除後も出力OFF。再有効化はHの明示操作と通常の開始条件が必要。

`has_torque_off_report`は既存`state/status`のOFF報告。キャッシュを含むため、通信成功した最新torqueレジスタ読取りの保証なし。実測静止、電源遮断とは別表示。EでもPC・ROS・USB故障時の独立非常停止保証なし。

トルクOFF専用サービス（対象launchの稼働と出力許可が必要）:

```bash
ros2 service call /hw_ToPoDualArm/torque_off std_srvs/srv/Trigger '{}'
```

## 通常動作の電流制限

`max_current_ma`は全選択ID共通の許容電流[mA]。既定0は未設定を表し、Hを拒否。EによるトルクOFFは未設定でも利用可能。電流値の自動推定・実機推奨値の仮置きなし。

実機の支持・荷重・各軸に対する電流上限を確認後、launch引数またはhardware設定へ明示。電流ベース位置制御のみ受け付け、モードの自動変更なし。2.69 mA単位で上限内の非ゼロGoal Currentへ変換し、保持目標・低速profile・電流目標の読返し後にtorque ON。運転中の電流目標読返し失効・上限逸脱、電流制御モードの変更はtorque OFFラッチ。

EEPROMのCurrent Limit・PWM Limitへの書込みなし。関節トルク[N m]や接触力[N]の直接制限ではなく、電流目標の制限。停止時の脱力・荷重落下と、通常保持に必要な電流の双方を考慮した設定が必要。

Gazebo状態表示: 通常の保持中は`gazebo | mode=hold | 回避=開始待ち`。停止指令の保持中のみ`停止解除待ち(B) | 静止=確認済み`または`静止=未確認`を追加。停止解除済みの場合の静止未確認表示は省略。状態変更時の直下に`A:回避/保持  B:停止解除→保持  Space:両方停止  Ctrl+C:停止して終了`を併記。`mode=avoid`は選択モードであり、回避の実行・停止完了の保証とは別。

回避状態表示: `準備中`・`開始待ち`・`実行中`・`完了`・`異常`。`開始待ち`は回避処理の未実行であり、ロボット静止や開始条件の成立保証とは別。停止解除待ちの場合はB、その後Aで明示開始。未受信または0.5秒を超える診断の失効は`未受信/失効`。ROS上の`avoidance`・`idle`・`stopped`などの値は変更なし。準備前のAは開始せず、再操作を案内。

## 実機小動作の準備

現行実装の実機試験: 未実施。対象ID・符号／原点・支持条件・独立した電源停止手段の確認後に実施。

1. 更新済み`dynamixel_handler`の起動と`/dynamixel/fresh_joint_states`受信の確認。2026-10-01の最終確認では既存ドライバから受信済み。今後ドライバを再起動する場合は、既存設定の`term/torque_auto_disable: true`による全対象torque OFFに備えた支持が必要。
2. `method/split_read: false`、位置・速度の同時読取り、goal・status・extraの周期読取り。`fresh_joint_states`は今回の通信で位置・速度を同時取得できたIDだけの配信。nameはモータID、positionはrad、velocityはrad/s、frameは`dynamixel_motor`。通信開始時刻の実時間stamp付き。従来のキャッシュ済み`state/present`による代用なし。
3. 選択関節の対応表と実物姿勢の照合。マッピング既定値は`dynamixel_joint_state_bridge_ids_31_52.yaml`。全軸の校正済み保証なし。対象外IDへの位置・torque指令なし。
4. 以下の出力許可付き起動後、対象IDの表示を確認してH。`hold`になってからJを1回、Space、実測停止、Rの順で小動作・停止を確認。

```bash
ros2 launch gng_vlut_system dynamixel_sim_control.launch.py \
  allow_hardware_output:=true 'joint_names:=[L_joint7]' \
  max_current_ma:=${DYNAMIXEL_MAX_CURRENT_MA:?確認済みの許容電流mAを設定}
```

この例ではGazeboの追加起動なし。単独試験時のGazebo停止サービス未応答は、実機側の停止表示とは別扱い。

制限設定: [topodualarm_hardware_test.yaml](../config/topodualarm_hardware_test.yaml)。実機目標速度2°/s、有効化時姿勢からの範囲±5°、Gazebo開始姿勢差1°、実機追従偏差3°。Xシリーズの分解能に合わせたprofile速度1.374°/s、加速度21.4577°/s²。0指定による制限無効化なし。複数関節のJ/Kは拒否。

SpaceはPC・ROS・USBが応答する条件のソフト停止。PC停止・USB断・出力ノード強制終了時の独立非常停止保証なし。実測を受け取れない間は「停止未確認」。復旧後は現在姿勢を保持し、旧目標への追従再開なし。

## Gazebo指令への追従

回避対象は左腕7関節（既定）、グリッパーID48・右腕・首は出力対象外。実機の小動作・停止確認後、`allow_sim_follow:=true`を追加。Hで保持を確立し、Gazeboと実機の姿勢差を確認してF。転送元は`/sim_ToPoDualArm/dual_arm_controller/controller_state`の`reference.positions`。Gazebo実測角や軌道の最終点の直接転送ではない。

```bash
ros2 launch gng_vlut_system dynamixel_sim_control.launch.py \
  enable_gazebo:=true allow_hardware_output:=true allow_sim_follow:=true \
  max_current_ma:=${DYNAMIXEL_MAX_CURRENT_MA:?確認済みの許容電流mAを設定} \
  'joint_names:=[L_joint1,L_joint2,L_joint3,L_joint4,L_joint5,L_joint6,L_joint7]'
```

転送中も実機用の速度・範囲・追従偏差制限を適用。Gazebo stamp停止、制御／安全状態の失効、実機実測失効、操作端末heartbeat失効は停止ラッチ。初期設定は小動作試験用で、腕全体の回避軌道の転送には未検証。

実環境Tmap入力の既存Gazeboへ接続する場合は、[環境Tmapの手順](realsense_gazebo.md#更新中の環境tmapをgazeboへ接続)の`pointcloud_avoidance.launch.py`を`enable_keyboard:=false`付きで起動し、新launchは`enable_gazebo:=false`（既定）で追加。両端末のROS_DOMAIN_IDを一致。新端末のA／Space／BでGazeboを操作。Viewerは引き続き`sim_ToPoDualArm`のGazebo実測姿勢。

実環境入力を含めて新launchから一括起動する場合（初期状態は保持・実機出力OFF）:

```bash
ros2 launch gng_vlut_system dynamixel_sim_control.launch.py \
  enable_gazebo:=true \
  input_config:=/ros2_ws/src/gng_vlut_system/config/viewer_environment_gazebo_input.yaml
```

入力未指定の場合はGazebo内の模擬点群による左腕回避。上記実環境入力では既存RealSense・自己除去・Tmap更新の稼働が必要。Aによる回避開始、Fによる実機転送は別操作。

最新の試験結果と未検証範囲: [リリース記録](releases/2026-10-01_dynamixel_sim_control.md)。
