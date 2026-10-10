# ToPoDualArmのDynamixel単独動作・Gazebo指令転送

パンチルトの電流制御: [ID51・52の重力補償・減衰](dynamixel_neck_torque.md)。手動操作の共通launchで`target:=neck`を指定可能。

通常動作の対象: XM430-W350・XM540-W270、電流ベース位置制御・速度基準profile。初期対象はユーザー指定の左腕7関節（ID41〜47）。旧launch・旧設定は保持。

## s・r・fの共通追従構成

sはブラウザの操作対象Simulator、rは実機リーダー、fは実機フォロワーです。共通launchで入力変換・経路管理・実機監視とSimulatorを起動し、接続はブラウザで選択します。

```bash
# コンテナ内。既存USBドライバの利用、初期は実機出力OFF
ros2 launch gng_vlut_system robot_follow.launch.py \
  params_file:=topo_dual_arm_max_long.yaml profile:=r_display
```

ブラウザは `http://127.0.0.1:8877/?model=long`。更新後は再読み込みし、「ROS2連携 → s・r・fの接続構成」を使用します。接続先は既定 `http://127.0.0.1:8879`。

| profile | 接続 |
| --- | --- |
| `manual` | sの手動操作、fの出力経路なし |
| `r_display` / `f_display` | r / f → sの描画姿勢 |
| `r_dynamics` / `f_dynamics` | r / f → sのMuJoCoモータ目標 |
| `r_to_f` | r → f |
| `s_to_f` | sの現在姿勢 → f |
| `r_display_f` | r → sの描画姿勢、r → f |
| `r_dynamics_f` | r → sのMuJoCoモータ目標、sの物理実姿勢 → f |

描画追従は校正済み絶対角。力学追従は有限トルクのPD駆動・接触計算で、選択後に「sの力学を開始」を押します。力学・実機追従の腕は既定で開始時からの角度差です。実機へ渡すsの姿勢は物理の現在角で、MuJoCo目標角そのものではありません。

[robot_follow.yaml](../config/robot_follow.yaml)の各profileに `simulator_source`、`simulator_mode`、`follower_source`、`follow_mode` を指定します。`follow_mode: absolute` は絶対角、`relative` は開始時からの角度差。1つの対象に入力元は1つ、f → s → fの循環は拒否します。sの内部入力トピックは固定、r/fの実測トピックは `roles` で指定し、共通入力変換の出力へ反映します。YAMLの編集反映には共通launchの再起動が必要です。起動後の登録済みprofileの変更はブラウザまたは `/robot_follow/manager` の `profile` パラメータで可能です。

共通入力は既定でr/fとも位置・速度の同時実測 `fresh`。既存の読取り専用readerからfの描画だけを確認する場合は `follower_input_type:=present` を指定できます。フォロワー実機制御の監視は引き続きhandlerの `fresh_joint_states`・status・extra・goalで、presentによる代用なし。共通launchはUSB接続・ドライバ起動を含みません。

既存実測配信の利用時は `enable_joint_state_input:=false`。既存Simulatorの再利用をせず別途管理する場合は `enable_simulator:=false`。GNGとロボットをViewerへ送る場合は `enable_viewer:=true`。このViewerは共通入力を重複起動せず、ブラウザの `/sim/joint_states` とTFを使用します。同じ管理launchや同じ実測入力の多重起動は拒否対象です。管理中のsの状態送信は1ブラウザだけです。

実機出力の許可は `allow_hardware_output:=true` と確認済み `max_current_ma:=...` の明示指定。ブラウザの「fの操作」で、「fの出力を準備」→ 保持確認 →「fの追従を開始」の順に操作します。停止ラッチがある場合は、実測静止を確認して「停止解除・出力OFF」を先に押します。選択だけでトルクON・追従開始は行いません。端末操作は `enable_keyboard:=true`。出力設定は `hardware_config`、実測入力のバス・校正は `input_config` で変更します。
既定の実機対象は腕14関節、開始時姿勢から5°・速度2°/s・加速度30°/s²の小動作範囲です。グリッパー・首の実機駆動は対象外です。既存の電流目標・保持目標の読返し、静止・可動域・出力競合の確認を維持します。

構成変更は世代番号を更新し、旧世代のs姿勢・f目標を破棄してfを停止します。停止解除と再準備・再開始が必要です。実測入力とブラウザの状態取得は既定0.3秒で失効、端末heartbeatは0.4秒で失効。入力・通信の復旧だけで実機の再開は行いません。sの送信端切断はs追従中のfへ停止要求、力学入力失効はsを保持してfの目標配信を止めます。「fを停止」は保持による停止、「fのトルクOFF」は支持が外れる可能性のあるトルク解除です。

構成検査は `python3 -B -m pytest -q -p no:cacheprovider gng_vlut_system/test/test_robot_follow_model.py`。疑似サーボの再検証は、コンテナ内の隔離domain223・実機とは異なる `/fixturefollow/dynamixel` を使用します。

```bash
ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1 \
  python3 -B /ros2_ws/src/gng_vlut_system/test/check_robot_follow_control.py
```

ブラウザ・実MuJoCoの再検証はSimulatorフォルダで `node tests/robot-follow.browser.mjs`。既存アプリサーバー8877、Chrome、更新済みgng_vlut_systemのビルドが必要です。隔離domain224・専用ブリッジ・専用ブラウザを使用し、試験終了時に所有プロセスを停止します。実USB・実モータでの新構成の駆動は未検証です。

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

回避実行中の点群接近: `obstacle_wait`で接近時の姿勢保持。自動再開設定によらず、距離不足だけでは`software_stop`への移行なし。

- 自動再開ON: `viewer_environment_gazebo_input.yaml`の`enable_obstacle_auto_resume: true`。点群余裕50 mm・GNG最寄りと直接隣接の安全が0.3秒継続後、同じ回避実行の再開。復帰先は当初Aで保存した姿勢
- 自動再開OFF: 保持継続。Aでホールド→点群余裕の回復→Aで新規回避開始。Bによる停止解除は不要
- Space・入力失効・自己干渉・関節異常: 停止ラッチの維持。B解除後、Aで明示再開

開始時の距離不足: 引き続き拒否。既存停止ラッチの自動解除なし。変更反映: Gazebo launchの再起動。

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

## 重力補償・保持付きの手動操作

共通入口: `dynamixel_hand_guiding.launch.py`。対象と制御方式はlaunchオプションで選択。
`target:=follower`（既定）は右ID31〜37・左ID41〜47、`target:=leader`は右ID1〜7・左ID11〜17、`target:=neck`はID51・52。
グリッパーとリーダー首ID21・22は対象外。対象変更は終了後の再起動。
既定モデル: `topo_dual_arm_max_long`。設定: [dynamixel_hand_guiding.yaml](../config/dynamixel_hand_guiding.yaml)。

`control_mode:=gravity`（既定）: 自重の支持と小さな速度抵抗による手動操作。
`control_mode:=adaptive_hold`: 静止時の保持、操作中の保持解除、調整後の角度での再保持。
`init/torque_auto_enable`はhandler起動時のON設定であり、柔らかさ・重力補償の制御モードとは別。

```bash
ros2 launch gng_vlut_system dynamixel_hand_guiding.launch.py

# 保持付きのリーダー手動操作。実機出力なし
ros2 launch gng_vlut_system dynamixel_hand_guiding.launch.py \
  target:=leader control_mode:=adaptive_hold

# 首も同じ入口。既存dynamixel_neck_torque.launch.pyも共通入口へ転送
ros2 launch gng_vlut_system dynamixel_hand_guiding.launch.py \
  target:=neck control_mode:=adaptive_hold

# s・r・f共通launchからの追加起動。手動操作の出力許可は独立
ros2 launch gng_vlut_system robot_follow.launch.py \
  profile:=r_display hand_guiding_target:=leader hand_guiding_control_mode:=adaptive_hold
```

共通入口の既定: `allow_hardware_output:=false`。YAMLの許可より優先、実機指令publisherなし。
腕の校正設定は`config_file`、保持設定は`mode_config_file`で指定。後者の既定は[dynamixel_adaptive_hold.yaml](../config/dynamixel_adaptive_hold.yaml)。
保持ゲイン・上限・外力閾値はプレビュー用仮値、実機の推奨値ではない。軸順配列またはスカラ共通値に対応。
重力補償モードでは保持設定の読込み・操作入力の購読・保持計算なし。
更新済みhandlerの`/dynamixel/fresh_joint_states`を購読し、`/dynamixel_hand_guiding/status`へ状態と支持トルクを配信。
トルク換算未設定時の`current_ma`はnull。校正・可動域・速度条件の不一致は`error`へ表示。
USBの新規接続なし。既存3 Mbps handlerの継続利用、同じUSBへの二重起動禁止。

重力補償モードの計算: URDF質量・重心による支持トルク`g(q)`と、モータ速度に対する粘性抵抗。
モータ電流は`I = (ramp * scale * g(q) - damping_gain * motor_velocity) / torque_nm_per_ma`。
`scale`は対応表の角度換算符号、位置目標・位置PID・積分項なし。
首用と共通のゼロ電流読返し→トルクON報告→補償立上げ、終了時OFF要求の経路。

保持付き制御は上記に保持トルクを追加。開始時の現在角度を保持目標へ取り込み、操作検出で保持の強さを`release_ramp_sec`で減少。
操作終了と静止が`min_hold_duration_sec`続いた時点で、その角度を新しい保持目標へ一度だけ取り込み、`hold_ramp_sec`で保持を復帰。
保持中の目標角度は固定で、垂れへの追従なし。操作中も重力補償と既存の減衰は継続。
保持トルクは`hold_gain_nm_per_rad * (保持目標 - motor角度)`を`max_hold_torque_nm`で制限。支持電流を確保した残りの電流範囲だけを保持に使用。
解除・保持の外力閾値はヒステリシス付き。外力入力では右腕・左腕を別グループ、首は2軸を1グループとして判定。

操作入力は`interaction_source`で選択:

- `manual`（既定）: `/dynamixel_hand_guiding/allow_guiding`の`std_msgs/msg/Bool`。`true`で対象全体の手動操作、`false`で静止確認後の保持。配信元1個による継続送信が必要
- `external_effort`: `/dynamixel_hand_guiding/external_effort`の`sensor_msgs/msg/JointState`。`header.frame_id=dynamixel_motor`、`name`は対象motor IDの文字列、`effort`はモータ座標の外力トルク[N m]、現在時刻のstamp。位置・速度配列は空で可

外力入力には力センサまたは校正済み推定器が必要。Dynamixelの生電流や重力を含む総トルクの直接接続は不可。外力推定器自体は未実装。
操作許可中の静止だけでは保持へ戻らず、ゆっくりした微調整にも対応。入力の欠測・失効・配信元重複は実機出力停止対象。
プレビューでは`has_fresh_interaction`・`is_guiding`・`hold_blend`・`hold_target_rad`を状態トピックへ配信。保持目標はmotor座標、グループ配列は右／左の順、首は1要素。

実機有効化に必要な確認・設定:

- 質量・重心・関節符号・原点の実機校正。基台直立、waist固定0、グリッパー角度0、把持物なしの計算条件
- `torque_nm_per_ma`: モータ出力トルク／電流[N m/mA]の校正済み正数14要素。カタログのストール値からの自動換算なし
- `max_current_ma`: 機種・支持・荷重に対応した許容電流[mA]14要素
- 配列順: 右7軸、左7軸。`damping_gain`の単位は[N m/(rad/s)]、首用の[mA/(rad/s)]とは別
- 腕の`calibration_target`と`target`の一致、`has_verified_calibration: true`、launchの`allow_hardware_output:=true`。未校正・対象不一致・未設定時の実機有効化拒否
- リーダーの質量・重心・原点・電流換算はフォロワーとは別の校正対象。必要に応じて`urdf_path`・`mapping_file`・`driver_namespace`を指定
- 首の保持付き制御は校正済み重力補償の有効化と`torque_nm_per_ma`の2軸設定が必要。首の設定は[dynamixel_neck_torque.yaml](../config/dynamixel_neck_torque.yaml)
- 対応機種: XM430-W350・XM540-W270。`current`モード、Reverse無効、Goal更新時自動トルクON無効、開始前トルクOFFと静止
- 同じhandlerへ指令する位置保持・リーダー追従・首制御等との同時使用禁止。モータモード・Current Limit・ゲインの自動書換えなし

`robot_follow.launch.py`では`hand_guiding_target:=none`が既定、追加ノードなし。選択肢は`leader`・`neck`。
`hand_guiding_config`・`hand_guiding_mode_config`・`hand_guiding_interaction_source`で設定、`allow_hand_guiding_hardware_output:=true`で手動操作だけの出力を許可。
フォロワーと手動操作の両方を実機出力する場合、別のhandler名前空間が必要。同じバスの場合はノード起動前の構成拒否。

モード0の電流指令とモード5の電流上限は別の用途。[ROBOTISのGoal Current仕様](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102)。
モードを変更する場合は腕を機械的に支持し、トルクOFFでの確認が必要。未確認の値による一括モード変更・一括トルクONなし。

運転条件: 実測位置・速度の失効0.2 sec、状態・電流目標の失効1.5 sec、URDF可動域・手動速度・電流上限の監視。
支持電流が上限を超える場合、補償不足を隠す飽和継続ではなく異常終了。
異常・Ctrl+C時は所有した14軸へのゼロ電流とトルクOFF要求。通信復旧だけでの自動再開なし。
終了・立上げ・通信異常時の落下防止保証なし。腕の機械的支持と独立した停止手段の準備が必要。
完全な重力補償だけで任意位置の静止を保証する方式ではなく、モデル誤差・摩擦・追加荷重によるドリフトの可能性。

検証範囲: 静的位置エネルギー勾配・MuJoCo支持トルクとの一致、仮想1軸MuJoCoによる補償誤差下の保持・外力操作・新角度での再保持、3対象の共通launchと隔離疑似handlerによる解除・再保持・操作入力失効時のゼロ電流と対象軸OFF。
実機の柔らかさ・重力支持・電流係数: 未校正・未検証。既存実機のトルク変更なし。

## 実機リーダー・フォロワー制御

Gazeboなしの専用起動。既定モデルは`topo_dual_arm_max_long`、設定は[dynamixel_leader_follower.yaml](../config/dynamixel_leader_follower.yaml)。実機駆動・実物のID／符号／原点の校正は未検証。

| 部位 | リーダーID | フォロワーID | 既定の追従対象 |
| --- | --- | --- | --- |
| 右腕7軸 | 1〜7 | 31〜37 | 対象 |
| 左腕7軸 | 11〜17 | 41〜47 | 対象 |
| グリッパー | 8・18 | 38・48 | 入力換算は設定で有効化可能、実機出力は対象外 |
| 首 | 21・22 | 51・52 | 対象外 |

グリッパーの新機構は、右ID8が反時計回り、左ID18が時計回りで閉じる入力換算に対応。モータの開閉端をURDFの開度へ線形換算し、可動域外は開閉端で飽和。LongのURDFは0 radが閉、約0.785398 radが開。旧対応表は保持し、新換算は共通入力設定の`leader.enable_gripper_input: true`時の入力だけに適用。フォロワーID38・48の校正は別途必要で、`joint_names`への追加は不可。リーダー・首への指令なし。モータモードの自動変更なし。

新入力は原点未確認のため既定OFF。[dynamixel_joint_state_input.yaml](../config/dynamixel_joint_state_input.yaml)の`leader`節で、`right_gripper_open_deg`／`right_gripper_closed_deg`、`left_gripper_open_deg`／`left_gripper_closed_deg`によるモータの開閉端を設定。仮置きは0°で開、右+180°／左−180°で閉。実際の閉じ切りが150°なら、閉じ端を右+150°／左−150°へ変更。開いた状態の実測原点を確認したうえで有効化。

有効時の`/leader/joint_states`は腕14軸とグリッパー2軸の16軸。mimicはモデル側で展開。ID8・18を含む全入力の鮮度を満たす場合だけ配信し、グリッパー欠測時の旧開度による鮮度更新なし。実機指令の対象は`joint_names`の腕だけ。

### 起動と設定

初回のビルド:

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select gng_vlut_system
source /ros2_ws/install/setup.bash
```

USBを専有する`dynamixel_handler`は別起動。既存handlerがある場合は再利用、同じUSBへの二重起動は禁止。Wizardとの同時通信も避けること。既存の起動方法は`ros2 launch dynamixel_handler dynamixel_handler_launch.xml`。ドライバの起動・終了時設定は別管理のため、トルク自動ON無効・終了時の脱力に対する支持条件の確認が必要。

共通実測入力を先に起動。Long設定のViewer launchが入力トピックの変換・配信を所有します。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=topo_dual_arm_max_long.yaml
```

Viewerを使用しない場合は`ros2 launch gng_vlut_system dynamixel_joint_state_input.launch.py`だけを起動。
両launchの同時起動は避け、入力を別途配信済みの場合はViewerに`enable_dynamixel_joint_state_input:=false`を指定。
実機追従を使う場合に、別の対話端末で次を追加。

```bash
ros2 launch gng_vlut_system dynamixel_leader_follower.launch.py
```

既定は`allow_hardware_output: false`。実機指令publisherの生成なし、Hによる有効化も拒否。共通入力は制御ノードとは独立し、フォロワー未接続でもリーダーだけの角度配信とブラウザ物理フォロワーの使用が可能。

リーダー共通入力は`/<leader_driver_namespace>/fresh_joint_states`からID・符号・原点・グリッパー開度を換算し、関節名・rad・rad/s・最古の構成関節の実測時刻を`/leader/joint_states`へ配信。`header.frame_id`は校正済み入力を示す`dynamixel_leader`。これはTF座標系の指定とは別の入力種別です。部分欠測・過去時刻の再送による鮮度更新なし。

追従制御は校正済みリーダートピックを購読し、ID・角度の再換算や`/leader/joint_states`の再配信を行いません。フォロワーの制御監視は引き続き`fresh_joint_states`とhandlerの状態topicを使用。キャッシュ済み`state/present`による制御監視の代用なし。

フォロワー表示は既存の読取り専用readerにも対応する`input_type: present`が既定で、`/<driver_namespace>/state/present`から`/follower/joint_states`へ配信。関節対応表の固定腰・mimicを含み、Max／Longでは21関節です。位置・速度の同時実測を使う場合は入力設定の`follower.input_type: fresh`へ変更。表示用present経路を実機制御監視へ接続する構成ではありません。

共通入力の接続先・対応表・リーダーグリッパー校正は`dynamixel_joint_state_input.yaml`に集約。従来の制御設定にあった`enable_leader_gripper_input`と`leader_*_gripper_*_deg`は、入力設定の`leader.enable_gripper_input`と`leader.*_gripper_*_deg`へ移動。両端末の入力設定の重複なし。必要な実機監視topicは既存の「実機小動作の準備」と同じ。

YAMLの主な設定:

| 設定 | 用途 |
| --- | --- |
| `joint_names` | 実機出力対象関節。初回は1関節での符号・追従・停止確認 |
| `driver_namespace` | フォロワーhandlerのtopic接頭辞。既定`/dynamixel` |
| `leader_driver_namespace` | リーダー・フォロワーの同一バスID重複検査用。共通入力側の接続先と対応した値 |
| `mapping_file` / `leader_mapping_file` | 実機出力の逆換算／リーダーIDの照合。共通入力側の対応表と整合した設定 |
| `enable_relative_follow` | 既定true。追従開始時からのリーダー角度差をフォロワーへ反映 |
| `allow_hardware_output` | 実機出力許可。変更だけではトルクONなし |
| `max_current_ma` | 許容電流。既定0は未設定・H拒否。機種・支持・荷重に基づく明示設定が必要 |
| `max_velocity` / `max_acceleration` | 指令速度・profile加速度。既定2°/s・30°/s² |
| `max_excursion` | 有効化時姿勢からの試験範囲。既定5° |
| `enable_torque_off_on_exit` | 既定true。自分で有効化した出力の終了時OFF要求。腕の支持が必要 |

実機出力の使用条件: XM430-W350・XM540-W270、`cur_position`モード、速度基準profile、goal更新による自動トルクON無効。許容電流・実測静止・競合publisherの不在・保持目標／profile／電流目標の読返しを確認後にトルクON。Current Limit・ゲイン・Operating Modeの自動書換えなし。[ROBOTISの電流ベース位置制御仕様](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11)。

### 操作

- H: 現在の実測姿勢の保持準備と対象IDのトルクON。準備完了後は`hold`
- F: リーダー追従ON/OFF。OFFは現在姿勢の保持
- Space: 保持して追従停止。入力復旧だけでは再開なし
- R: 実測静止確認後の停止解除・出力OFF。再開にはH→F
- E: 選択したフォロワーIDのトルクOFFと指令停止
- Ctrl+C: 停止と所有出力のトルクOFF要求後、専用ノードの終了。既存handlerの停止なし

相対追従の式: `q_target = q_follower_start + q_leader - q_leader_start`。Fを押した時点の実測を基準に保存し、開始時の姿勢差による急動作を防止。`enable_relative_follow: false`では校正済み絶対角への追従、開始姿勢差1°の確認が必要。

リーダーまたはフォロワーの必要関節の実測失効0.3秒、操作端末heartbeat失効0.4秒、可動域・試験範囲・追従偏差超過で停止ラッチ。電流目標の失効・上限逸脱は既存のトルクOFF処理。Ctrl+C時のOFF要求は対象IDだけで、OFF報告を最大1秒待機。通信・PC故障時の電源遮断保証なし。実物の支持と独立停止手段が必要。

### 実機フォロワーの手動姿勢確認

2026-10-09実機確認: `/dev/ttyUSB0`、Protocol 2.0、3 Mbps、ID31〜38・41〜48・51・52の18台。読取り専用配信と、シミュレータの直接表示による確認。WizardはDisconnect、同じUSBのhandler・readerは二重起動禁止。

```bash
ros2 launch gng_vlut_system dynamixel_current_pose.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max_long.yaml \
  baudrate:=3000000 \
  output_topic:=/follower/joint_states \
  enable_viewer:=false enable_viewer_output:=false
```

既存handlerが`/dynamixel/state/present`を配信中の場合は、上記に`enable_reader:=false`を追加。USBへの新規接続なし、既存実測の変換だけの起動。handlerを併用しない場合だけ読取り専用readerを有効化。

ブラウザを再読み込みし、「ROS2連携 → 姿勢・TFの送受信」の受信トピックを`/follower/joint_states`へ変更。「ロボット → 姿勢の入力元 → ROS実測の直接表示」を選択。物理開始・リーダー追従は不要。実機への位置・トルク指令なし。手動操作前に実機の支持とトルクOFFを確認。終了はCtrl+C。

`output_topic`は実測JointStateの配信先、`enable_viewer_output:=false`は既存Viewer用の追加配信を無効化。初期姿勢publisherとの混在防止。

Max／Longの既定換算: [dynamixel_joint_state_bridge_max_ids_31_52.yaml](../../dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge_max_ids_31_52.yaml)。旧設定と手編集済みのID31〜52設定は保持。`mapping_file`の明示指定は自動選択より優先。実機表示launchの既定モデルはLong。Viewer launchでは機体YAMLの`dynamixel_mapping_file`を使用。

肩の下垂姿勢: 右ID32の+90°→`R_joint2`の+90°、左ID42の+90°→`L_joint2`の−90°。肩の旧±90°オフセットによる相殺なし。他の腕軸・首の符号と原点は現行の手編集済み設定を継承。

フォロワーグリッパー: 閉じ原点0°、右ID38の正方向・左ID48の負方向で開き、換算係数は右+1・左−1。mimicは親の反転角。ROSの実測値は丸め込みなし。シミュレータの直接表示はグリッパーとmimicだけをURDF開閉端へ飽和し、45°付近の端点超過でも表示更新を継続。腕の可動域外は警告と前回表示の保持。リーダー制御の可動域検査は維持。

反映: 関節変換launchの再起動とブラウザの再読み込み。稼働中ノードのパラメータはYAML編集だけでは更新されない構成。USBのhandler・readerは既存プロセスを利用し、変換ノードの重複配信を避けた切替。実機駆動の全軸校正は未検証。

### ブラウザの物理フォロワー

実機出力OFFのままで使用可能。ブラウザ側から実機への指令なし。

1. 更新済みROS／物理ブリッジを起動。既存版の更新時はSimulatorフォルダで`bash start_ros.sh --restart`、ブラウザ再読み込み
2. 「ロボット → 操作対象ロボット → 姿勢の入力元」で「リーダー → 物理フォロワー」を選択。入力は`/leader/joint_states`
3. 「物理」で「関節動力学（基台固定）」を選択し、「物理を開始」

腕は最初の有効なリーダー実測からの角度差、グリッパーは換算済みの開度をMuJoCoモータ目標へ入力。グリッパーは初回入力から開度に追従。描画は物理の実姿勢で、リーダー角度の直接代入なし。URDFの可動域・指令速度・トルク上限、PD駆動・接触計算は既存の物理経路。実機の電流制限とは別。

入力・実測・目標送信は描画周期から独立。実時刻stampの進行と鮮度を検査。ブラウザと物理側で入力失効1秒を監視し、失効後は保持目標・停止状態を維持。同一stampの再送や通信復旧だけでの再開なし。再開は「停止」→「物理を開始」、新たな実測姿勢を基準に保存。可動域外・接続切断・通信OFFでも追従停止。ページ非表示では物理セッション終了。

詳細: [Simulatorの関節通信](../../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md#姿勢tfのwebsocket送受信)。

### 検証

実機通信なしの隔離ROSで両腕14軸のID分離、保持目標→トルクONの順序、相対追従、実測失効停止・非自動再開を検証。実USB／実モータでの駆動は未検証。

```bash
# コンテナ内、実機と異なるROS domain97での疑似モータ試験
cd /ros2_ws/src/gng_vlut_system
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -B -m pytest -q -p no:cacheprovider test/test_dynamixel_leader_follower.py
```

ブラウザ・実MuJoCoの通し試験はSimulatorフォルダで`node tests/leader-follower.browser.mjs`。隔離domain98・専用サーバー・Chromeの使用、終了時に所有プロセスを停止。実物への停止・安全性の保証とは別検証。
