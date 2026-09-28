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
