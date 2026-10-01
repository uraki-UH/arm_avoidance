# 双腕シミュレーションの制御・物理設定

機体設定を切り替えてGazebo点群・自己除去・GNG/VLUT回避を起動する構成は[共通点群回避](pointcloud_avoidance.md)を参照。ToPoDualArmの保存済み左腕データにも対応。

## ToPoDualArmの統合回避デモ

対象: `urdf/dual_arm_urdf/dual_arm_robot.urdf`の既存ToPoDualArm。選択引数: `robot:=topodualarm`。名前空間: `/sim_ToPoDualArm`。

```bash
docker exec -it gng_cpu_container bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/local_setup.bash
ros2 launch gng_vlut_system dual_arm_control.launch.py robot:=topodualarm
```

操作: 起動後はホールド。Aで回避開始／ホールド復帰、Spaceでソフト停止、停止後のLで解除後ホールド、Ctrl+Cで終了。Lのリーダー追従開始には新鮮な`/leader/joint_states`が必要。USBドライバの自動起動なし。

- 回避方式: URDF外接球と左右の模擬前腕カプセルによる幾何探索。Gazebo状態による入力で、実センサー点群・GNG/VLUT回避とは別方式。
- 既定設定: `ToPoDualArm.yaml`、`topodualarm_gazebo_demo.yaml`、`topodualarm_avoidance_demo.yaml`。ToPoDualArmの寸法・直動グリッパーに対応。max系の既定GNG/VLUTデモは維持。
- 保存済みGNG: `ToPoDualArm10000`は左腕用。14関節角を必要とする双腕GNGデモへの流用なし。
- 物理設定: ODE quick、有限力／有限トルクの位置追従。直動関節の`effort`は軸方向力 [N]、回転関節はトルク [N m]。
- 計画時の余裕: `min_planning_clearance_th: 0.01` [m]。自己・床・台への物理追従誤差の吸収用。実測監視の停止判定は既存の0.005 mを維持。
- 停止確認: 回転速度0.01 rad/s、直動速度0.001 m/sを各判定値とする0.25秒の継続静止。`safety/status`の`max_velocity_rad_sec`と`max_linear_velocity_m_sec`に分離。
- Viewer表示名: `sim_ToPoDualArm`。既存Viewerサーバーへの配信。実機実測表示とは別系統。
- 実機UDP: 未対応。`robot:=topodualarm`での`udp_config`指定は拒否。Dynamixelへの駆動指令なし。
- ToPoDualArmの別起動: [Dynamixel小動作・Gazebo指令転送](dynamixel_sim_control.md)。左腕ID41〜47、実機出力は既定OFF。既存UDP経路への変更なし。
- 起動依存: Dockerfile記載の`gazebo_ros2_control`、`controller_manager`、`joint_state_broadcaster`、`joint_trajectory_controller`。古いコンテナでは追加導入と`gng_vlut_system`の再ビルドが必要。

通し試験コマンド（コンテナ内、未使用の出力先を指定）:

```bash
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 \
  python3 /ros2_ws/src/gng_vlut_system/test/check_topodualarm_avoidance.py \
  --output /ros2_ws/src/artifacts/topodualarm_trial
```

## 2026-10-01: max実機追従・Viewer・回避の作業計画

- 対象: `topo_dual_arm_max`、別実機のリーダー、UDP接続のフォロワー
- 到達点: リーダー実測値による直接描画、実機追従＋フォロワー実測姿勢による描画、Gazebo追従・回避試験
- 操作窓口: `dual_arm_control.launch.py`へ集約。回避と追従は排他、開始元はホールドのみ
- 状態: 計画・手順の文書化。今回の実機送信・Gazebo追加試験なし
- 実装順: [当日タスク](TASK_LIST.md#max-operation-20261001)。以下の未実装項目は現在の起動だけでは利用不可

### 現在の接続と不足部分

| 機能 | 現状 | 追加・確認対象 |
| --- | --- | --- |
| リーダー → Gazebo | `/leader/joint_states`の購読あり | maxの実機入力元・関節校正。USBドライバの自動起動なし |
| リーダー → Viewer | 統合launchの直接表示経路なし | リーダー実測による描画。フォロワー・Gazebo物理実行への依存なし |
| Gazebo → 実機 | 補間済み目標のUDP送信あり、既定OFF | 受信機仕様、独立19関節の順序・符号・原点、停止・watchdog |
| Gazebo → Viewer | `/sim_topo_dual_arm_max/joint_states`の表示あり | 現行URDFでの追従・更新の通し確認 |
| 実機 → Viewer | UDP応答の内部保持のみ、ROS配信なし | 実測JointState配信、表示元の分離、未受信・失効表示 |
| 回避 | Gazebo LiDAR・GNG/VLUTの接続あり | 現行URDFでの回避・停止試験。実センサーによる実機回避は未統合 |

### 必須の描画経路

- リーダー直接表示: リーダー実測 → Viewer。表示だけの利用では追従・実機送信OFF、フォロワー未接続でも利用可能な構成
- 実機追従表示: リーダー実測 → 統合制御・Gazebo補間目標 → UDP → フォロワー駆動 → フォロワー実測 → Viewer

表示元の選択: `リーダー実測` / `フォロワー実測` / 既存の`Gazebo実測`。選択機能は新設予定、現在のlaunch引数ではない。
表示元と制御モードは独立。表示切替によるLの追従開始、Hの送信許可、停止解除はなし。既存のホールド経由条件を維持。
画面には選択元と受信状態を明示。フォロワー実測表示へのリーダー値・指令値の代入、失効時の別ソースへの自動切替なし。
追従OFF・ホールド・停止中も、選択元の新鮮な実測受信が続く間は描画更新を継続。

Viewerの主対象: 統合launchの`robot_viewer_bridge_node`から配信する[ToPoFuzzy-Viewer](../../ToPoFuzzy-Viewer/README.md)。
HTMLシミュレータの[ROS軌道再生](../../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md)は別経路。
HTML側への実測追従表示も未接続で、指令軌道の再生を実機姿勢の代用としない方針。

### 実装方針

1. 入出力の確定: リーダーの機種・デバイス・実測topic、フォロワーの受信firmware・IP/port・関節対応表の確認。旧14関節設定・模擬`ENABLE`/`STOP`の流用なし。
2. 実測表示の先行: `/leader/joint_states`からの直接描画と、指令送信OFFでのフォロワー受信・描画経路の追加。UDP実測を校正後のradへ変換し、独立19関節の`JointState`として配信。受信機側の送信開始に駆動許可が必要な場合は、読取り専用経路の確保が先決。
3. 表示元の分離: リーダーは`/leader/joint_states`、Gazeboは`/sim_topo_dual_arm_max/joint_states`、フォロワーは`/follower/joint_states`を予定。フォロワーtopicと実機表示名`hw_topo_dual_arm_max`は新設予定。リーダー直接表示には別の表示名を使用。実機表示への別ソース・初期ゼロ姿勢の混入、リーダー入力への折返しを禁止。
4. 受信健全性: 全関節・有限値・可動域・受信元の検査と、実時間による鮮度監視。実測経路は`use_sim_time=false`相当、Gazeboのpauseから独立。機器時刻なしの場合は受信時刻と明示し、失効中の姿勢を「現在の実機姿勢」として再配信しない設計。
5. 統合起動: リーダー直接表示と実機追従表示を同じlaunchの選択肢として整備。表示だけの構成ではGazebo物理実行・実機出力を起動条件から分離。送信は引き続きHの明示許可。リーダードライバは機種別設定と起動時書込みの確認後に統合。既存Viewerサーバーの重複起動なし。

実機への送信元: Gazebo JTCの`desired.positions`。フォロワー実測表示の入力: フォロワーのエンコーダ実測。
両者の別記録が必要。Gazeboの実測角・送信した角度・受信機による目標値の返信は、実機エンコーダ値とは別物。

### 現時点の起動方法（実機送信なし）

前提: 元ワークスペース`/home/uraki/uraki_ws`を`/ros2_ws/src`へマウント済みの`gng_cpu_container`、ビルド済みROS環境、max用`gng.bin`・`vlut.bin`。
GNG/VLUTと使用URDFの整合確認が必要。既存の同名Gazebo・制御ノードとの重複起動不可。

```bash
docker exec -it gng_cpu_container bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/local_setup.bash
ros2 launch gng_vlut_system dual_arm_control.launch.py robot:=max \
  leader_topic:=/leader/joint_states
```

- UDP: `udp_config`未指定、ソケット生成・実機送信なし
- リーダー: 別途、新鮮な実測`JointState`の配信が必要。入力なしではLによる追従開始不可
- Dynamixel変換: 必要時のみ`leader_mapping_file:=<校正済みYAML>`を追加。変換元は`leader_input_topic`、USBドライバ起動とは別
- Viewer: 既存サーバー・ブラウザへ接続し、`sim_topo_dual_arm_max`の表示を確認。リーダー直接表示・フォロワー実測表示の選択は上記追加実装後
- キー入力: 起動端末を前面、Enter不要。[状態別のキー一覧](#1つのlaunchでの統合操作)を使用

確認用の別端末（同じコンテナ・ROS環境・`ROS_DOMAIN_ID`）:

```bash
ros2 topic info -v /leader/joint_states
ros2 topic echo --once /sim_topo_dual_arm_max/control/status
ros2 topic echo --once /sim_topo_dual_arm_max/safety/status
ros2 topic echo --once /sim_topo_dual_arm_max/avoidance/status
```

実機用起動設定: 未確定。[UDPの必須条件](#gazebo目標のudp出力)と受信機の実仕様を照合後に設定作成。
`allow_remote_udp:=true`だけでの有効化、模擬YAMLのIPだけを変えた運用は不可。

### 運用テストと完了条件

1. 表示経路の独立確認:

   - リーダー直接表示: フォロワー未接続・実機送信OFF・Gazebo物理実行なしで、リーダー実測と描画の全関節を照合。
   - フォロワー実測表示: 指令送信OFFで受信データと描画を照合。駆動無効・機械支持などの確認後に、リーダー／Gazeboと独立したフォロワー姿勢変化を確認。
   - 表示切替: 模擬入力でリーダーとフォロワーに異なる姿勢を入力し、選択元だけへの描画追従を確認。L/H/停止状態は不変、受信停止時は失効表示、自動代替なし。

2. Gazebo追従: ホールド→L→追従→L→ホールド。追従中Aは拒否、切替中の連打も拒否。ViewerはGazebo実測へ追従。Space後は実測停止を確認し、Lで解除→ホールド、もう一度Lで追従。UDPの自動再開なし。
3. 模擬UDP: [切替単体試験](#切替条件の単体試験)と[localhost通し試験](#gazebo目標のudp出力)。H前の無送信、送信目標との照合、Space・入力失効時の遮断、再許可条件の確認。出力先は未使用のディレクトリ。
4. 実機追従: 受信側停止・watchdogと、PC/ROS/通信に依存しない停止手段の動作確認後。人のいない可動範囲・支持条件・実機に適した速度制限で小範囲から確認。ホールド・静止・初期姿勢差の確認→H→L。描画元はフォロワー実測を選択し、受信実測との一致を確認。リーダーとの追従遅れ・姿勢差を隠す補完なし。実測と指令の偏差、Lでの保持、停止後の非再開を別々に判定。
5. Gazebo回避: 実機UDP OFF。ホールド→Aで左右の模擬前腕接近→退避、Aでホールド。回避中Lの拒否、点群更新、自己除去、GNG/VLUT入力、最小距離、探索失敗時停止を確認。見た目の移動だけでは合格判定なし。
6. 実機回避への移行: 実センサーのtopic・TF・時刻、フォロワー実測に基づく自己除去・衝突判定の接続後。最初は人腕ではなく試験物体。Gazebo内の模擬前腕への回避指令転送と、実機周辺の障害物への回避を別試験として記録。

回避指令速度の既定値: 2.5 rad/s。UDP指令速度の既定上限: 0.3 rad/s。
実機回避前に両経路の速度設定の整合が必要。監視の無効化や閾値の緩和だけによる通過判定は不可。
人腕接近試験: 独立した停止手段・受信側停止の確認前は未実施。Spaceはソフト停止要求であり、実機の停止完了保証ではない。

既知の阻害要因: 現行URDFのmaxで状態更新失効、max_longでSpace後の実測停止未確認。
[前回のGazebo通し試験](releases/2026-10-01_gazebo_software_stop.md)は0/2回成功。最新の切替条件は単体試験のみで、実機追従前に通し再検証が必要。

記録項目: 使用URDF・GNG/VLUT・設定、実行コマンド、入力/実測topic、指令角・実測角・受信時刻、姿勢偏差［rad］、表示遅延［ms、測定可能範囲のみ］、最小距離［m］、停止要求・停止確認の時刻、成功/失敗/未検証、起動プロセスの終了確認。
合格基準未定の項目は測定結果のみ。パケット送信成功と実機停止成功の混同なし。

## 1. 要約

2026-09-28更新：`JointTrajectoryController → 位置目標 → 有限トルクODEモータ`へ変更。
[専用hardware](../src/simulation/bounded_gazebo_system.cpp)が位置誤差×`motor_position_gain`から目標速度を生成し、
URDF上限×`motor_limit_scale`（既定0.95）で速度を制限。ODEの`fmax`にも同率のトルク上限を設定し、追従遅れを許容。
位置・速度の直接書き換えなし。Gazeboを一時停止で生成し、hardwareの準備後に物理計算を開始。
点群回避YAMLの`physics_solver: world`が直接解法を選択。通常デモと未指定の回避はquick。

モータ内部はODEの拘束計算で、実機モータのトルクPID・減速機・通信遅延を同定したモデルではない。
実測`effort`は子リンクの関節反力を軸へ射影した値で、ストッパ・接触反力も含む。駆動上限と反力を区別。
根拠：[ODEのモータパラメータ・関節反力実装](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEJoint.cc)。

## 2. 条件・検証

| 項目 | 現在値・扱い | 設定場所 |
| --- | --- | --- |
| エンジン | Gazebo Classic 11 / ODE。通常quick（200反復、SOR=1）、点群回避world。CFM=1e-8 | [world](../worlds/dual_arm_demo.world) |
| 重力 | `(0, 0, -9.81)` m/s² | 同上 |
| 物理刻み / 更新目標 | 0.001 s / 1000 Hz。実時間速度は計算負荷に依存 | 同上 |
| 制御周期 | controller_manager 1000 Hz | [Gazebo launch](../launch/dual_arm_gazebo_demo.launch.py) |
| controller状態 / action監視 | 50 Hz（統合操作100 Hz） / 20 Hz。JointStateは物理周期 | 同上 |
| 制御関節 | 腕14 + 腰1 + 首2 + グリッパー2 = 19、指2関節はmimic | 同上 |
| 指令 / 状態 | position目標 / position, velocity, effort。指mimicも独立した有限トルク駆動 | 同上 |
| 固定基部 | worldへ固定。浮遊・転倒の評価対象外 | [URDF生成](../launch/robot_gazebo_spawn.launch.py) |
| 質量・慣性・重心 | URDFの`inertial`、単位kg・kg m²・m | [max URDF](../../urdf/topo_dual_arm_max/topo_dual_arm_max.urdf)、[long URDF](../../urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf) |
| 元URDFの合計質量 | max / longとも6.042 kg、25リンクにinertial。実測値の確認なし | 同上 |
| 生成時の補完 | 慣性のない固定接続のうち可動関節の親に0.001 kg、対角慣性1e-6 kg m²を追加 | URDF生成。元URDFは変更なし |
| 衝突 / 表示メッシュ | 各URDFのcollision / visual、STLのscaleは0.001 | 各URDF |
| 関節摩擦・粘性 | `dynamics`指定なし。実機に合わせた同定なし | 各URDF |
| 接触摩擦・反発・剛性 | 接触補正速度0.1 m/s、接触層1 mm。モータのストッパ離脱係数fudge_factor=0 | world / 生成URDF |
| 自己接触の物理設定 | self_collideの明示設定なし。回避デモの幾何監視とは別 | 生成URDF |
| 作業台 | 中心(0.70, 0, 0.20)m、寸法(0.35, 0.8, 0.4)m、static | world |
| 通常デモ速度 | 補間指令0.15 rad/s。実測速度の厳密な上限ではない | [通常デモYAML](../config/dual_arm_gazebo_demo.yaml) |
| 人腕の物理扱い | staticなカプセルの位置を更新。人体の関節・トルク・接触力モデルは未導入 | Gazebo launch / 回避ノード |
| 回避デモ | 前腕半径0.045m・長さ0.35m、目標余裕0.12m、停止距離0.035m | [回避YAML](../config/dual_arm_avoidance_demo.yaml) |

URDFの関節上限は腕6 N m・3 rad/s、腰10 N m・2 rad/s、首3 N m・3 rad/s、
グリッパー20 N m・1 rad/s。角度上下限は左右・max/longで異なるため各URDFを正本とする。
これらはURDF宣言値。駆動には95%を使用（腕5.7 N m・2.85 rad/s）し、数値積分・反力測定のずれに余裕を確保。
実機の定格・瞬時最大値との対応は未確認。
longもmaxと同じ質量・慣性値を持つため、寸法変更に応じた慣性の妥当性は未検証。

コントローラYAMLとGazebo用URDFはlaunchが`/tmp/dual_arm_gazebo_demo_*`へ生成し、終了時に削除。
変更する正本は上表のlaunch・world・元URDF。生成ファイルの直接編集は不要。
幾何回避の指令速度は0.45 rad/s、点群・GNG回避は2.5 rad/s。指令周期0.15 s、モータ位置ゲイン60 /s。
回避ノードは実測速度・反力のURDF上限超過または欠落で停止。監視・試験の丸め許容差は各単位1e-6。
元の位置直接設定で観測した4.555 rad/sは旧方式の記録。現行方式の結果・失敗試行は下記へ統合。
目標余裕は軟らかい評価項で、厳密な下限ではない。[3秒試験の条件・結果](releases/2026-09-28_dual_arm_gng_lidar.md)。

## Gazeboのソフト停止

対象: `bounded_gazebo_system`を使用するmax / max_longのシミュレーション。実機ドライバへの接続なし。
検証状態: maxの保持中速度逸脱と通常回避デモの入力失効が未解決。[検証結果・制限](releases/2026-10-01_gazebo_software_stop.md)。

| 名前空間内のAPI | 型 | 用途 |
| --- | --- | --- |
| `safety/stop` | `std_srvs/srv/Trigger` | 停止要求のラッチ |
| `safety/reset` | `std_srvs/srv/Trigger` | 条件付きの停止ラッチ解除 |
| `safety/is_stop_latched` | `std_msgs/msg/Bool` | ラッチ状態。実停止確認とは別 |
| `safety/status` | `std_msgs/msg/String` | 実測停止確認を含むJSON診断 |

停止例（起動済みmaxのシミュレーションのみ）:

```bash
ros2 service call /sim_topo_dual_arm_max/safety/stop std_srvs/srv/Trigger '{}'
```

停止中は全可動関節・mimicの停止要求適用時の姿勢を有限トルクで保持し、controllerの位置指令を遮断。
Gazeboのpause・位置の直接固定・トルクOFFは不使用。`success=true`は要求受付であり、停止完了の保証ではない。
確認先は`safety/status`の`is_stop_applied`と`is_stopped`。`stopped`の判定条件は全関節の速度0.01 rad/s以下が
シミュレーション時刻で0.25 s継続、物理更新の実時間経過0.5 s以内、位置・速度の有限値。
物理停止・実測失効時は`stop_unconfirmed`。物理更新前の要求は`stop_requested`。
`demo/status`・`avoidance/status`の`state=stopped`はデモ側の指令停止であり、実測停止完了とは別。

解除順序: 全位置controllerのdeactivate → 新鮮な`stopped`確認 → `safety/reset` → controllerのactivate → 新規指令。
active中のresetと、ラッチ中のactivateは拒否。既存の軌道を捨てた再activationと、デモの明示開始が必要。
通常・回避デモはラッチ受信で自動開始を無効化。回避デモの人カプセル移動も停止するため、人が接近し続ける試験とは別。
外部制御モードでは外部指令元も停止・旧目標解除したうえで再開。ソフト停止はPC/ROS/Gazeboが応答する条件の機能であり、実機非常停止の代替ではない。

### 1つのlaunchでの統合操作

```bash
ros2 launch gng_vlut_system dual_arm_control.launch.py robot:=max_long
```

maxは`robot:=max`。起動した端末を前面にして操作、Enter不要。Docker内は`docker exec -it`の対話端末が必要。

| 現在の状態 | A | L |
| --- | --- | --- |
| ホールド | 回避開始 | リーダー追従開始 |
| 回避 | ホールドへ | 拒否、回避継続 |
| リーダー追従 | 拒否、追従継続 | ホールドへ |
| ソフト停止 | 拒否 | 停止解除→ホールド |
| 切替・解除処理中 | 拒否 | 拒否 |

共通: Spaceでソフト停止・追従/UDP OFF、Ctrl-Cで停止要求後のlaunch全体終了。HはUDP出力ON/OFF（ONは設定指定・保持・静止確認後のみ）。
初期状態: ホールド。回避と追従の直接切替・同時実行なし。現在のモードを同じキーでOFF→ホールド確認→別の開始キー。拒否した操作の後追い実行なし。
停止後のL: 新鮮な実測停止・端末接続・未完了要求なしを確認後、controller停止→ラッチ解除→再activation・実測静止確認→ホールド。
解除中のSpace・実測/端末失効で解除取消。解除だけではリーダー入力不要、追従開始には新鮮な入力と再度のLが必要。
UDP出力の自動再開なし。旧R・Q・Sキーは無効、終了はCtrl-C。ROSサービスからの直接切替も同条件で拒否。
`control/reset` APIは解除後保持の互換用途として維持。実測・操作端末・追従入力の更新失効時は停止要求。要求受付と実測停止は別表示。
現行URDFでの通し確認: 未完了。max_longはSpace後の実測停止未確認、maxは起動中の状態更新失効。[条件・記録](releases/2026-10-01_gazebo_software_stop.md)。

リーダー入力: `leader_topic:=/leader/joint_states`、関節名・rad単位の`JointState`、進行する実測時刻が必要。
追従指令速度上限: 0.3 rad/s。実機からの既存入力を使用し、このlaunchからUSBドライバの起動なし。
必要時のみ`leader_mapping_file:=<校正済みYAML>`でDynamixel状態→JointStateの変換を追加。
入力元: `leader_input_topic:=/leader/dynamixel/state/present`。旧機種の対応表の無条件流用は不可。

UDP出力: 既定OFF。`udp_config`未指定時はソケット生成なし、HのON要求を拒否。
既存Dynamixelドライバは読取り目的でも設定書込みがあり、送信停止だけでは既存の位置目標を取り消せないため統合起動対象外。
人の腕を接近させる実機試験の準備完了を意味しない。Gazebo側の未解決点は上記のまま。

再現試験: [2機種の試験一覧](../test/dual_arm_control_cases.json)、同一疑似端末からのキー入力と模擬リーダー。
GUI・Viewer・実機入力は無効。Aの回避結果は追従・停止の合否と分離。

```bash
docker exec -e ROS_DOMAIN_ID=96 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -c '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/local_setup.bash
cd /ros2_ws/src
python3 -B skills/run-benchmark-batch/scripts/run_batch.py gng_vlut_system/test/dual_arm_control_cases.json --output artifacts/dual_arm_control_20261001/first --repeats 1 --timeout-sec 210 --max-total-sec 450 --estimate-sec 90 --continue-on-error
'
```

再実行時は未使用の出力先が必要。`command.json`はlaunch引数、`report.json`は実測判定・終了確認。

### Gazebo目標のUDP出力

入力: `dual_arm_controller/controller_state`の`reference.positions`。Gazeboコントローラの補間済み目標であり、実測角や軌道終点の直接転送ではない。
指定順の独立19関節をCSV化し、各値は`round((rad × 180 / π × scale + offset_deg) × 10)`の整数、末尾カンマ付き。
旧UDP実装の整数切捨てとは最大1カウントの差。関節不足・未知名・mimic・非有限値・URDF範囲外は拒否、ゼロ補完なし。

模擬宛先の設定例: [dual_arm_udp_loopback.yaml](../config/dual_arm_udp_loopback.yaml)。次の起動だけでは送信なし。
模擬受信機からの実測受信後、保持状態でHによる明示ONが必要。

```bash
ros2 launch gng_vlut_system dual_arm_control.launch.py robot:=max_long \
  udp_config:=/ros2_ws/src/gng_vlut_system/config/dual_arm_udp_loopback.yaml
```

- 実測形式: `agl,整数,...`の19角度。設定したIP・送信元portとの一致が必要
- 指令更新上限: 50 Hz、指令・実測の受信有効期間: 各0.5 s
- 指令速度上限: 0.3 rad/sとURDF値の小さい方。量子化許容差: 0.001 rad
- 開始時姿勢差: 0.05 rad、追従偏差: 0.2 rad。超過時の自動姿勢合わせなし
- OFF・Space・異常・終了: 設定済み停止パケットの送信と、角度送信の遮断。Lで追従再開後もUDPはOFF

統合操作時の`/clock`・controller状態配信: 各100 Hz（シミュレーション時間基準）。低速実行時の時刻粒度対策で、UDPの0.5 s監視期限は維持。

実機宛先には`allow_remote_udp:=true`に加え、確認済みの19関節順・符号・offset、`enable_packet`・`stop_packet`、
`is_mapping_verified`・`is_receiver_stop_verified`・`is_receiver_watchdog_verified`と`receiver_watchdog_sec`の明示が必要。
フラグは利用者による確認宣言であり、実装による安全認証ではない。停止仕様は受信側の旧目標破棄・再許可前の指令拒否を含む確認が必要。
設定例の`ENABLE`・`STOP`は模擬受信機専用。旧14項目の`slave/master`設定や実機firmwareへの流用は不可。

受信CSVには機器時刻・連番・停止ACKがないため、受信時刻による鮮度のみの確認。UDP到達・順序・物理停止の保証なし。
`control/status.udp.is_physical_stop_confirmed`は常にfalse、端末の実測停止表示はGazeboのみ。
実機送信・機種別校正・受信側停止は未検証。旧`topoarm_hardware.launch.py`の無条件ゼロ送信・再送実装は不使用、既存ファイルの変更なし。

UDP再現試験: 上記バッチ実行の一覧を`gng_vlut_system/test/dual_arm_udp_cases.json`、出力先を`artifacts/dual_arm_udp_20261001/clock_retry`へ変更。
localhost模擬受信機を試験内で起動・終了。H前の無送信、補間目標との全CSV照合、Space遮断、L再開後のUDP OFF、実測途絶を対象とする有限試験。

キー削減後の試験コマンド（再実行時は未使用の出力先へ変更）:

```bash
docker exec -e ROS_DOMAIN_ID=96 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/local_setup.bash
cd /ros2_ws/src
python3 -B skills/run-benchmark-batch/scripts/run_batch.py gng_vlut_system/test/dual_arm_udp_cases.json --output artifacts/dual_arm_keys_20261001/udp_box_retry --repeats 1 --timeout-sec 210 --max-total-sec 450 --estimate-sec 90 --continue-on-error
'
```

### 切替条件の単体試験

ROS通信・Gazebo・実機接続なし。停止解除・ホールド経由・相互切替拒否・UDP許可条件の回帰。

```bash
docker exec gng_cpu_container bash -lc '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/local_setup.bash
cd /ros2_ws/src/gng_vlut_system
PYTHONDONTWRITEBYTECODE=1 timeout 120s python3 -B -m pytest -q -p no:cacheprovider \
  test/test_dual_arm_udp_output.py test/test_dual_arm_control_udp.py \
  test/test_dual_arm_control.py test/test_dual_arm_control_keyboard.py \
  test/test_dual_arm_mode_model.py test/test_joint_command_model.py \
  test/test_gazebo_stop_keyboard.py test/test_dual_arm_limits.py
'
```

### キーボード停止

既存launchを使用する場合は、Gazeboと同じROS環境の別端末で起動:

```bash
ros2 run gng_vlut_system gazebo_stop_keyboard.py --namespace sim_topo_dual_arm_max
```

max_longは`--namespace sim_topo_dual_arm_max_long`。この端末を前面にしてSpace / Sで停止要求、Enter不要。
Q / Ctrl-Cは停止要求後に入力監視を終了。解除・再開キーなし。Gazebo画面のグローバルショートカットではない。
要求の待機上限は2 s。終了コード0は受付成功のみ、2は未確認・失敗。実測停止表示は新しい`safety/status`に基づく別判定。
PC停止・端末未選択・通信断への停止保証なし。ソフト停止本体の未解決点は上記のまま。

Docker内の既定ROS環境を使用する場合:

```bash
docker exec -it gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/local_setup.bash; ros2 run gng_vlut_system gazebo_stop_keyboard.py --namespace sim_topo_dual_arm_max'
```

`ROS_DOMAIN_ID`・`ROS_LOCALHOST_ONLY`の独自設定時はGazeboと同じ値の指定が必要。
入力層の再現試験（模擬サービス、Gazebo駆動なし）:

```bash
python3 -B gng_vlut_system/test/test_gazebo_stop_keyboard.py
docker exec -e ROS_DOMAIN_ID=97 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash; source /ros2_ws/install/local_setup.bash; cd /ros2_ws/src; timeout --signal=TERM --kill-after=5 90 python3 -B gng_vlut_system/test/check_gazebo_stop_keyboard.py --output artifacts/gazebo_stop_keyboard_20261001/pty'
```

再実行時は`--output`を未使用パスへ変更。試験用domain97の既存ノード検出時は実行中止。

### ソフト停止の再現試験

実行先: 既存`gng_cpu_container`。試験用domain 96・localhost・Gazeboポート11369、GUIと実機入力は無効。
初期姿勢から左肩へ0.04 radの指令後、停止・保持・不正な解除の拒否・明示解除・新規指令を検証。
判定対象は全21関節（mimic込み）、軌道入力はtopic。action経路と実機は未検証。

```bash
docker exec -e ROS_DOMAIN_ID=96 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -c '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/local_setup.bash
cd /ros2_ws/src
python3 -B skills/run-benchmark-batch/scripts/run_batch.py gng_vlut_system/test/gazebo_software_stop_direct_cases.json --output artifacts/gazebo_software_stop_20261001/direct_diagnostics --repeats 2 --timeout-sec 600 --max-total-sec 2450 --estimate-sec 85 --continue-on-error
'
```

保存済み出力への上書き不可。再実行時は`--output`を未使用パスへ変更。
通常回避デモの試験一覧は`gazebo_software_stop_cases.json`、直接指令の一覧は`gazebo_software_stop_direct_cases.json`。
通常デモ試行の出力は`demo_retry`、各1回、各600 s・全体1250 s上限。初回直接指令の出力は`direct`、同じ回数・上限。
各`command.json`は起動したlaunchの全引数、`ownership.json`は試験プロセスの所有情報、`report.json`は物理判定と終了確認。

**有限トルク駆動への移行前の補間・把持経路調査（2026-09-28）**

同日の共通化後の管理場所・互換性・検証は[動作スムージング](motion_smoothing.md)を参照。

| 経路 | 実装済みの処理 | 調整箇所 |
| --- | --- | --- |
| 通常Gazeboデモ | 始終点の速度・加速度ゼロを指定した5次補間。最大関節変位に応じた区間時間の延長 | [動作送信](../scripts/dual_arm_gazebo_demo.py)の`tick`、通常デモYAMLの`segment_duration_sec`・`max_joint_velocity` |
| 回避Gazeboデモ | 初期姿勢への復帰候補と各腕関節の正負ステップを比較する局所探索、自己・床・台の外接形状確認 | [幾何探索](../scripts/dual_arm_avoidance_geometry.py)の`choose_step` |
| 回避の指令生成 | 実測位置から短区間の5次補間、始終点の速度・加速度ゼロ。直近確認時は区間時間と更新間隔に`control_period_sec`を使用、更新可否はROS時刻で判定 | [指令生成](../scripts/dual_arm_avoidance_demo.py)の`publish_target`・`tick`、回避YAML |
| ROS上面把持候補 | TCP位置のEMA、姿勢のSLERP、グリッパーyaw反転の整合、連続確認・短期欠測保持 | [候補追跡](../../grasping_system/src/top_grasp_surface_estimator_node.cpp)の`updateTrackSnapshot`。`candidate_position_ema_alpha`・`candidate_orientation_ema_alpha`は既定0.35 |
| 単体HTMLのURDF動作 | `u²(3−2u)`の3次補間。汎用IKは位置・方向誤差とゼロ姿勢からの関節角二乗による探索 | [単体HTML](../../ToPo-FUZZY_Manipulation_v1.html)の`v322InterpolateJointValues`・`evaluateRobotIkCandidate`・`solveGenericRobotIk` |

通常デモの区間時間は`max(segment_duration_sec, 1.875 × 最大関節変位 / max_joint_velocity)`。
5次補間の最大速度係数1.875を考慮した時間設定であり、加速度・ジャークの個別上限設定はなし。
導入済み`joint_trajectory_controller 2.54.0`のヘッダーで既定`splines`と5次補間の条件を確認。
[Humbleの補間仕様](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)とも整合。

回避探索の評価は`200 × max(0, target_clearance − 距離)² + 0.003 × Σ(関節角 − 初期角)²`。
初期姿勢からの移動抑制はあるが、前回指令との差・速度変化・切替頻度の罰則はなし。
各区間の補間が滑らかでも、更新時に実測速度・直前指令速度を引き継ぐ構成ではない。
区間ごとの停止・再加速や関節選択の切替が揺れへ寄与する可能性はあるが、実測による原因確定は未実施。
変更候補は速度・加速度を引き継ぐ指令生成と、探索の時間方向の連続性。採用・実装は未決定。

通常デモは固定8姿勢と指の開閉、回避デモは前腕カプセルからの退避であり、自動把持の完結経路ではない。
`grasp_joint_candidates.launch.py`は候補経路・最終関節姿勢の出力専用で、Gazeboコントローラへの指令なし。
`gazebo_pick_and_place.launch.py`は物体・センサを含む環境起動で、渡すロボット設定が`ToPoDualArm.yaml`固定。
max/long対応には同launchの`params_file`引数化・受け渡しと、把持段階から制御への接続が別途必要。
モデル切替の正本は`config/topo_dual_arm_max.yaml`・`config/topo_dual_arm_max_long.yaml`。
両設定の回転式グリッパー専用把持体積は未定義。旧モデルの把持体積の無条件流用は不可。
HTMLの汎用IKには前回解を優先する項がなく、回転関節の探索範囲はURDF値によらず±π。
HTML側を修正する場合は、補間だけでなく関節制限・前回解からの連続性も確認対象。

調査範囲は作業ツリーのソース・設定・既存プロセス一覧・導入パッケージ版の読取り。
調査中に回避スクリプトの並行更新を確認。記述は最終読取り時点であり、稼働中プロセスとの一致は未確認。
本調査で制御コード・設定の変更、シミュレーション起動、既存プロセスの停止は未実施。
