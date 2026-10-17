# 双腕シミュレーション

既定はLong。機体設定は[topo_dual_arm_max_long.yaml](../config/topo_dual_arm_max_long.yaml)を参照。
実機MID-360は[専用手順](../../hardware_drivers/mid360/README.md)、ブラウザ操作は[Simulator](../../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/README.md)を参照。

## センシング → ROS回避 → 実行先

bag／実センサの点群・ロボットTF・センサTF → ROI生成・観測側自己除去 → 共通ROS回避 → MuJoCo／実機。
回避計算の現在姿勢は制御対象の実測JointState。self_voxelとブラウザ表示も実測に追従。
以下は`docker compose exec gng_cpu bash`内、ROS環境をsource済みの端末。

### 1. 入力の起動

bag:

```bash
bash /ros2_ws/src/scripts/play_arm_avoidance_bag.sh
```

設定は[arm_avoidance_bag.yaml](../../scripts/arm_avoidance_bag.yaml)を参照。
再生対象は`/livox/lidar`・`/tf`・`/tf_static`だけ。記録済みの関節値・ROI・自己マスク・指令は不要。
自己マスクはURDFの形状キャッシュと観測時刻TFから再生成。点群はVLUTと同じ格子へ1回登録。

実センサは専用ドライバと、観測ロボットの実測に基づくTF・校正済み取付TFを使用。
既存のTF配信は重複起動しないこと。ROI・自己マスクの別生成は不要。

### 2. 共通回避と実行先

bag → MuJoCo:

```bash
ros2 launch gng_vlut_system sensing_avoidance.launch.py \
  execution_backend:=mujoco \
  source_tf_topic:=/recorded/tf source_static_tf_topic:=/recorded/tf_static
```

実センサではTF引数2個を省略。ブラウザ内物理は別途開始しないこと。

| 実行先 | 用途 |
| --- | --- |
| `mujoco` | MuJoCoも起動。実測は`/sim/joint_states` |
| `external` | ROS回避指令まで。外部ドライバは別起動 |
| `dynamixel` | 既存ID変換を使用。実機出力は明示許可時のみ |

実機用の共通実測変換を使う場合は`state_topic:=/follower/joint_states`。
指令は`/avoidance/joint_command`のJointTrajectory、即時1点・絶対関節角の経由点[rad]。
区間照合を通る名目目標へは直接追従。遮断時は保存済みGNGによる経路探索。
速度から仮の角度刻みへの変換なし。実機出力は既定OFF、実機駆動は未検証。
名目目標の部分関節指定:

```bash
ros2 topic pub --once /avoidance/goal sensor_msgs/msg/JointState \
  '{name: [L_joint1, L_joint2, L_joint4], position: [-1.0, -1.0, -0.9]}'
```

設定は[機体YAML](../config/topo_dual_arm_max_long.yaml)の`avoidance`を参照。
制御基台は`avoidance.base_pos`・`base_rot_deg`、観測基台はbagのworld接続TFを使用。
world未接続時だけ`environment_voxelization`の明示固定配置を使用。観測側TFの現在時刻化・全体への再配信なし。
制御対象の移動基台は未対応。記録ボクセルを使う旧経路は`input_kind:=voxel`＋bag再生の`--roi-voxels`。

### 3. 表示・判定

表示: `http://127.0.0.1:8877/?model=long&joint_topic=/sim/joint_states&state_source=simulation`。
入力・回避・ROSブリッジのdomainを統一。終了は各起動元でCtrl+C。

TF不足・観測停止時は姿勢保持。自己除去はURDFボクセル近似、境界候補は制御側で要確認として保持。
生点群・TFだけの実bagでROI生成・往復動作・予測方式の局所退避指令・入力停止時の保持を確認済み。
視野外・遮蔽による保持と点群の重なりが残存。無接触での往復継続は未達。bagは仮想ロボット姿勢による観測更新なし。

### 机・床などの固定禁止領域

設定は[機体YAML](../config/topo_dual_arm_max_long.yaml)の`workspace_mask`（既定: 無効）。座標は制御ロボットの`base_link`、単位はm。
机の範囲だけの箱形マスクにより、後方の張り出した空間は固定マスク上は利用可能。全面の高さ制限は任意で併用可能。
寸法は実測値へ置換。変更の反映は回避ノードの再起動、GNG学習・VLUT再構築は不要。

固定禁止状態はセンサの`unknown`と独立、観測の消去による解除なし。
代表姿勢は既存VLUTで判定、移動区間は外接球列・直線補間・各軸の速度上限による近似照合。実測速度による減速余地の予測を併用。
実機の制動性能・追従誤差を含む連続衝突回避は未検証。

### 非停止のbag往復実験（MuJoCo専用）

ホストの元workspaceで:

```bash
bash scripts/run_dual_arm_simulation.sh --nonstop-simulation --gng-strategy vrm_predictive
```

bag・左腕往復・回避計算・表示を一括起動。タスクは[task.json](../../benchmarks/shuttle_bag/task.json)、bagは[arm_avoidance_bag.yaml](../../scripts/arm_avoidance_bag.yaml)を参照。
表示URLは起動ログ。終了はCtrl+C、時間指定は`--duration`。
通常は事前生成した関節空間の基準軌道へ追従、表示線はその手先軌跡。先読み区間の障害物で回避へ切替え、離隔の安定後に基準軌道へ復帰。
回避候補はGNG接続・VLUT状態からの有界A*種経路と局所候補。目標切替え・基準経路逸脱を抑える選好を併用。
視野外・遮蔽・欠測・接触・経路なしによる保持なし。欠測時は名目往復、観測時は退避先の選好コストへ反映。
有効な固定禁止領域は例外。禁止領域を避ける候補がない場合は、その領域への指令を棄却。
URDF可動域は維持。無接触保証なし、実機使用禁止。指令は`/sim/nonstop/joint_command`、通常の実機指令経路とは分離。
基準軌道はGNG種経路／RRT-Connectと独自の離散CHOMP系による生成。RRT*・MoveIt CHOMPは未使用、実電力量・大域最適性の保証なし。
bag途中からの再生は、先頭付近の固定TF欠落に注意。観測不足時の名目往復は回避動作の検証対象外。

## Gazebo Harmonicの標準effort制御

ROS 2 Jazzy＋Harmonic＋標準gz_ros2_control。JointTrajectory→PID→制限付きeffort。
設定は[dual_arm_effort.yaml](../config/dual_arm_effort.yaml)を参照。

ホストのworkspaceルートで:

```bash
docker build -f docker/Dockerfile.gazebo_harmonic -t uraki-gazebo-harmonic:local .
docker run --rm -it -e ROS_DOMAIN_ID=96 -e ROS_LOCALHOST_ONLY=1 \
  -e GZ_PARTITION=uraki_harmonic uraki-gazebo-harmonic:local \
  ros2 launch gng_vlut_system dual_arm_gz.launch.py
```

max／longの駆動・トルク上限・停止を検証済み。実機パラメータは未同定。
この入口に旧Classicの点群回避・ソフト停止ラッチ・実機出力は未接続。
再現入口は[check_sim_motor.py](../test/check_sim_motor.py)、結果は`log/artifacts/gz_motor_20261003/`。

## タスクの組替えと方式選択

```bash
ros2 launch gng_vlut_system dual_arm_tasks.launch.py
```

Gazeboとタスク実行器を起動。設定は[task_program.yaml](../config/simulation/task_program.yaml)を参照。
タスクenumと進行管理は[task_program.py](../scripts/task_program.py)、方式の組合せは[task_components.py](../scripts/task_components.py)。
TAMPは上位でタスク列を生成する接続が必要。追従中QP・把持・実機出力は未接続。

### 追加の経路計画（任意依存）

選択肢: BIT*、RRT系、CHOMP、STOMP。設定は[task_program.yaml](../config/simulation/task_program.yaml)を参照。

```bash
ros2 launch gng_vlut_system dual_arm_planners.launch.py planners:=bit_star
```

MoveIt計画サーバーと選択プラグインが必要。BIT*等の登録拡張は任意ビルド。
依存未配置でも通常ビルドは継続、未導入方式の実行はエラー。
軌道化は既存quinticを使用し、各経由点で停止。MoveItシーンへのGazebo・点群の自動同期は未接続。
STOMPは端点接続の再検査あり。検証環境のMoveIt 2.12.4では終了時SIGSEGVを観測。
追加ビルド設定は[Dockerfile](../../docker/Dockerfile.gazebo_harmonic)を参照。

### ロボット姿勢グラフの接続とオンライン更新

経路部品は`robot_graph`。環境点群の`/topological_map`とは別のロボット関節姿勢グラフ。
設定は[task_program.yaml](../config/simulation/task_program.yaml)、入力契約は[robot_graph_route.py](../scripts/robot_graph_route.py)を参照。
既存GNGの接続は[robot_graph_bridge.py](../scripts/robot_graph_bridge.py)。

入力はsnapshot／delta／heartbeat。ロボットID・関節順序・世代・鮮度を照合。
使用中ノード・辺の変更や期限切れで停止し、明示resume時に実測から再計画。
label=1の端点だけでは辺途中の連続衝突回避を保証しない。Gazebo実運動によるオンライン更新追従は未検証。

### 起動と操作

Humbleでは専用起動サービス、Jazzyでは直接実行。既定はheadless・開始待ち。
GUIは`gui:=true`、自動開始は`enable_autostart:=true`。

```bash
ros2 service call /sim_topo_dual_arm_max_long/task_executor/start std_srvs/srv/Trigger '{}'
```

同じ接頭辞にpause／resume／cancel、状態はstatus。障害物試験入力はobstacle（SetBool）。
障害物解除だけで自動再開なし。入力失効等のfailed状態は原因解消後のノード再起動が必要。
同じコントローラへの複数指令元は併用不可。

### 起動サービスの初回準備・管理

ホストの元workspaceで:

```bash
install -d -m 0700 docker/runtime/harmonic_launcher
docker build -f docker/Dockerfile.gazebo_harmonic -t uraki-gazebo-harmonic:local .
docker compose -f docker/compose.harmonic_launcher.yaml up -d
```

設定は[Compose](../../docker/compose.harmonic_launcher.yaml)を参照。コード・同梱設定の変更後は再ビルド。
同時1セッション。クライアント切断・ハートビート失効で所有プロセスを終了。
Humble/Jazzy間の基本操作は検証済み、ノード一覧・全メッセージの互換性保証なし。
別シミュレータとの同一domain・clock併用は禁止。GUIはX11のみ検証済み。
停止は同じComposeへの`down`。

### 差替え境界と検証範囲

共通入力・方式登録・Action接続は[planning_components.md](planning_components.md)を参照。
関節制限・始終点・時刻等の検査と、連続区間の衝突保証は別。
Harmonicの移動・中断・再開・取消は確認済み。取消直後の速度過渡は未解決で、指令制限を実測上限として扱わないこと。
再現入口は[check_task_program.py](../test/check_task_program.py)。

## 環境シナリオの管理

設定は[objects.yaml](../config/simulation/objects.yaml)と[scenarios](../config/simulation/scenarios/)を参照。
HarmonicとIsaacで共用。座標world、長さm、質量kg、RPYはrad。

| シナリオ | 環境 |
| --- | --- |
| empty | 物体なし |
| tabletop | 天板・箱・円柱 |
| dual_obstacles | 左右の円柱 |
| narrow_workspace | 左右の壁 |
| rolling_ball／rolling_balls | 斜面と球 |

### 転がる球

```bash
ros2 launch gng_vlut_system dual_arm_gz.launch.py scenario:=rolling_ball
```

重力・接触による運動。球の寸法・斜面の変更は各YAMLを参照。

### 選択と追加

scenariosへYAMLを追加し、`scenario:=名前`で選択。Isaacは`--scenario 名前`。
生成SDF／USDを直接編集しないこと。保存済みscenarioは初期環境であり、実行途中のチェックポイントではない。

### 検証範囲

Harmonicの6環境・接触・落下・球の運動を確認。IsaacはUSD構造検証まで。
シナリオ層からのセンサ点群・GNG/VLUT・Viewer環境表示は未接続。
検証は[test_dual_arm_launch.py](../test/test_dual_arm_launch.py)を参照。

## 標準ROS 2トピックとViewerの接続

名前空間は`/sim_topo_dual_arm_max_long`。標準JointState・JointTrajectory・URDF・TF・clockを使用。
関節角rad、角速度rad/s。HarmonicのTF名はURDFリンク名のままのため複数機体はframe分離が必要。

Viewerは外部実測として接続:

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:= robot_name:=sim_topo_dual_arm_max_long \
  urdf_path:=/ros2_ws/src/urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf \
  joint_control_backend:=external state_topic:=/sim_topo_dual_arm_max_long/joint_states \
  enable_robot_state_publisher:=false use_sim_time:=true robot_base_frame:=world
```

ROS domain・DDSネットワークを統一。初期姿勢・TFの二重配信は避けること。
接続試験は[check_sim_motor.py](../test/check_sim_motor.py)。表示接続に回避・停止機能は含まない。

## Isaac Simの接続

Isaac Sim 6.1.0・ROS 2 Jazzy。Harmonicと同じ[effort設定](../config/dual_arm_effort.yaml)を使用。

### 起動

利用規約への同意とGPUコンテナ環境が前提。設定は[Compose](../../docker/compose.dual_arm_isaac.yaml)を参照。

```bash
export ACCEPT_EULA=Y
export ROS_DOMAIN_ID=96
docker compose -f docker/compose.dual_arm_isaac.yaml up --build
```

終了は同じComposeへの`down`。Harmonicとの同一domain・clock併用は禁止。

### 検証と現在の制限

共通設定・USD drive構造は検証済み。GPUコンテナ未準備のため、Isaac本体のインポート完走・controller activation・物理追従・停止・Viewerは未検証。
再現入口は[check_sim_motor.py](../test/check_sim_motor.py)の`--backend isaac`。

## ToPoDualArmの統合回避デモ

以下は旧Gazebo Classic構成。Harmonicの入口とは別。

```bash
ros2 launch gng_vlut_system dual_arm_control.launch.py robot:=topodualarm
```

姿勢GNGではなく模擬前腕に対する幾何回避。UDP実機出力は非対応。
設定は[topodualarm_avoidance_demo.yaml](../config/topodualarm_avoidance_demo.yaml)、制限・検証は[既存記録](releases/2026-10-01_topodualarm_demo.md)を参照。
点群による旧Classic回避は[pointcloud_avoidance.md](pointcloud_avoidance.md)を参照。

## 2026-10-01: max実機追従・Viewer・回避の作業計画

旧UDP構成の未完了計画は[TASK_LIST](TASK_LIST.md#max-operation-20261001)を参照。
実機実測表示・リーダー／フォロワーの現行入口は[dynamixel_sim_control.md](dynamixel_sim_control.md)。
実測値と指令値は別系統、旧14関節UDP設定の19関節機体への流用は禁止。実機回避の通し検証は未完了。

## 胸部LiDARの取り付け形状

Longの既定は45°。取付情報は[chest_lidar.json](../../urdf/topo_dual_arm_max_long/meshes/chest_lidar.json)を参照。
60°版は位置・ブラケット形状も異なるため、Pitch変更だけでは代用不可。

```bash
python3 scripts/chest_lidar_urdf.py --angle 60 --output /tmp/topo_dual_arm_max_long_60.urdf
```

実行先のROSホスト／Docker内で生成。計測原点・追加部品の質量慣性は未校正。
保存済みGNG/VLUT・自己形状キャッシュは取付変更後の整合確認が必要。

## 頭部深度カメラ

旧Classicのmax／max_long用:

```bash
ros2 launch gng_vlut_system dual_arm_control.launch.py robot:=max_long point_cloud_source:=head_depth
```

設定は[dual_arm_depth_camera.yaml](../config/dual_arm_depth_camera.yaml)を参照。
headlessでもOpenGL環境が必要。点群は名前空間内camera/depth/points、光学座標系。
取得時刻のTFが必須。実機RealSenseの校正・ノイズ・欠損再現、回避完遂は未検証。

## Classicの制御・物理設定

設定は[world](../worlds/dual_arm_demo.world)・[機体URDF](../../urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf)・[Gazebo launch](../launch/dual_arm_gazebo_demo.launch.py)を参照。
ODE有限トルク駆動。effort実測には接触・ストッパ反力も含むため、駆動トルク上限とは区別。
元URDF質量はmax／longとも6.042 kg。実測質量・Longの慣性妥当性は未検証。

## Gazeboのソフト停止

旧Classicのbounded_gazebo_system専用。実機非常停止の代替ではない。

```bash
ros2 service call /sim_topo_dual_arm_max/safety/stop std_srvs/srv/Trigger '{}'
```

要求受付と実測停止は別。safety/statusのis_stop_applied・is_stoppedを確認。
解除はcontroller停止→実測停止確認→reset→再activation。解除だけで動作再開なし。
maxの保持速度逸脱・入力失効、Longの停止未確認が残存。[検証結果](releases/2026-10-01_gazebo_software_stop.md)を参照。

### 1つのlaunchでの統合操作

```bash
ros2 launch gng_vlut_system dual_arm_control.launch.py robot:=max_long
```

A=回避、L=リーダー追従、Space=停止、H=UDP許可、Ctrl+C=終了。
回避と追従はホールド経由で切替。停止後のLは解除→ホールドで、追従再開には再度Lが必要。
USBドライバは別起動。UDPは設定と明示許可が揃うまでOFF。

### Gazebo目標のUDP出力

設定は[dual_arm_udp_loopback.yaml](../config/dual_arm_udp_loopback.yaml)を参照。
入力はcontrollerの補間目標で、実測角ではない。19関節の対応と受信側停止・watchdog確認が必要。
設定例のENABLE／STOPは模擬用。実機への流用禁止。到達・順序・実機停止の保証なし、実機送信は未検証。

### 切替条件の単体試験

[test_dual_arm_control.py](../test/test_dual_arm_control.py)・[test_dual_arm_control_udp.py](../test/test_dual_arm_control_udp.py)を参照。
モード切替・出力許可の試験で、実機停止の検証ではない。

### キーボード停止

```bash
ros2 run gng_vlut_system gazebo_stop_keyboard.py --namespace sim_topo_dual_arm_max
```

起動端末を前面にしてSpace／S。Q／Ctrl+Cは停止要求後に監視終了。解除機能なし。

### ソフト停止の再現試験

設定は[gazebo_software_stop_direct_cases.json](../test/gazebo_software_stop_direct_cases.json)・[dual_arm_control_cases.json](../test/dual_arm_control_cases.json)を参照。
実機と異なるdomain・Gazeboポートを使用。補間・把持経路の調査は[motion_smoothing.md](motion_smoothing.md)を参照。
max／longの回転式グリッパー用把持体積は未定義、自動把持の完結経路は未接続。
