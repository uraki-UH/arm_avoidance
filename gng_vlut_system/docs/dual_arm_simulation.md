# 双腕シミュレーションの制御・物理設定

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
| controller状態 / action監視 | 50 Hz / 20 Hz。JointStateは物理周期 | 同上 |
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

| キー | 操作 |
| --- | --- |
| A | 回避デモON/OFF |
| L | リーダー追従ON/OFF |
| Space / S | ソフト停止 |
| R | 停止解除、保持の継続 |
| H | 実機出力ON要求。現状は拒否 |
| Q / Ctrl-C | 停止要求後のlaunch全体終了 |

初期状態: 保持。回避と追従は排他選択、追従中の回避なし。切替時は保持・実測静止確認後に旧指令を破棄。
解除後の自動再開なし。実測・操作端末・追従入力の更新失効時は停止要求。要求受付と実測停止は別表示。

リーダー入力: `leader_topic:=/leader/joint_states`、関節名・rad単位の`JointState`、進行する実測時刻が必要。
追従指令速度上限: 0.3 rad/s。実機からの既存入力を使用し、このlaunchからUSBドライバの起動なし。
必要時のみ`leader_mapping_file:=<校正済みYAML>`でDynamixel状態→JointStateの変換を追加。
入力元: `leader_input_topic:=/leader/dynamixel/state/present`。旧機種の対応表の無条件流用は不可。

実機出力: 無効固定。機種別校正・実測鮮度・実機専用停止処理の未確認による制限。
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
