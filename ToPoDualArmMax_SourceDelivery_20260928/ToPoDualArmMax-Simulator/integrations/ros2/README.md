# ROS 2連携

別PCの準備は[RUN_GUIDE](../../../../RUN_GUIDE.md)。通常のブラウザ単体起動にROSは不要。

## サーバーとROSブリッジの一括起動

ホストのworkspaceルート、起動済み`gng_cpu`を使用:

```bash
bash scripts/open_dual_arm_nvidia.sh long
```

Docker内のSimulatorルートでは`bash start_ros.sh`。更新時は`bash start_ros.sh --restart`とブラウザ再読み込み。
UIソース変更後は`npm ci && npm run build`。
表示は`http://127.0.0.1:8877/?model=long`、HTTPブリッジは8879、WebSocketは8880。
ブラウザ終了だけでは背景サービスは停止しないため、起動時の停止案内を参照。

## センシングとROS回避の独立起動

bag／実センサ → 共通ROS回避 → MuJoCo／実機。
[共通手順](../../../../gng_vlut_system/docs/dual_arm_simulation.md#センシング--ros回避--実行先)を参照。
ブラウザ内物理と外部シミュレータによる同一ロボットの同時駆動は禁止。

## ROS処理結果の表示専用接続

ROS環境をsource後:

```bash
ros2 run topo_fuzzy_viewer viewer_ws_gateway_node
```

接続先は`ws://127.0.0.1:9001/observe`。別画面のToPoFuzzy-Viewer起動は不要。
候補詳細は`viewer_edit_node`、モデル照合は`viewer_vehicle_registration_node.py`も必要。

対応形式は[Viewer](../../../../ToPoFuzzy-Viewer/README.md)と共通。Simulator本体とROS由来ロボットは別個体。
表示基準はworld、未解決TFのデータは非表示。動的TFの観測時刻補間は未対応。
「mapを基準に表示」は表示原点の指定で、worldとの校正の代用ではない。
ROS表示レイヤーは仮想センサの撮影対象外。

`/observe`は表示・詳細計算用。ROSノード操作・bag再生・編集確定は非対応。localhost待受、遠隔公開・認証は未対応。
接続仕様は[ViewerのROSブリッジ](../../../../ToPoFuzzy-Viewer/doc/ROS2_BRIDGE.md)を参照。

## 独立した点群送信

取得周期はセンサ側、送信側は配信対象の選択のみ。送信側で点群を再生成しない構成。

| 内容 | トピック |
| --- | --- |
| LiDAR | `/sim/lidar/points` |
| RGB-D全体 | `/sim/rgbd/points` |
| 対象物体の完全表面 | `/sim/object/full_points` |
| 対象物体の遮蔽付きRGB-D | `/sim/object/visible_points` |

型はPointCloud2、XYZはfloat32・m。機体基準はLongがbase_link、標準がbase_footprint。
RGB-D点群は`rgb`・`color_valid`付き。無効な色は灰色・`color_valid=0`。
完全表面には裏面・内部面も含むため、外皮抽出とは別。全面遮蔽では0点を送信。
取得時刻と関節姿勢は`/sim/points/info`。headerはROS受信時刻で、実センサとの時計同期なし。
未送信トピックの配信口は生成せず、無送信3秒で解除。

単体ブリッジの起動は`python3 integrations/ros2/pointcloud_bridge.py`。
依存・メッセージ構築は[pointcloud_bridge.py](pointcloud_bridge.py)・[native_points.py](native_points.py)を参照。

### GNGへの接続

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=sim_rgbd.yaml
```

設定は[sim_rgbd.yaml](../../../../ais_gng_cpu/src/ais_gng/config/gng_cpu/sim_rgbd.yaml)を参照。
実機用`graspnet.yaml`へのトピック名だけの変更では座標系が一致しないため使用不可。

### MID-360の送信

点群は機体基準に変換済み。取得時のセンサTFは同じstampで配信し、取付変換の二重適用は禁止。
Livox CustomMsg・IMU・点別時刻は出力なし。[センサ仕様](../../docs/SENSOR_FIDELITY.md)を参照。

### 深度画像と画素対応点群（2026-10-01追加）

RGB-Dの画素対応出力:

| トピック | 形式 |
| --- | --- |
| `/sim/camera/depth/image_rect_raw` | Image、32FC1・m、無効値0 |
| `/sim/camera/depth/camera_info` | CameraInfo |
| `/sim/camera/depth/points` | PointCloud2、画像と同じwidth・height、無効XYZはNaN |

3出力は共通stamp・光学frame（X右・Y下・Z前）。RGB画像への位置合わせは非対応。
校正は[RGB-D手順](../../docs/RGBD_README.md)を参照。実機からの校正値推定は非対応。

### MuJoCoと点群

物理OFFならポーズ・テーブル注視・VM動作はブラウザ単独で使用可能。

物理は同梱URDFの慣性・関節・effort上限を使用。自己接触は既定OFF、ON時は凸包近似。
ヒンジ・スライドはworld基準の拘束、物体同士の結合は非対応。
直接配置は実行中も反映。動的物体の速度、ヒンジ・スライドの拘束基準はリセット。
駆動設定は[physics_robot.py](physics_robot.py)、シーン生成は[physics_scene.py](physics_scene.py)を参照。

### MID-360 bagによるGNG-VLUT回避

Longの学習済み左右独立GNG/VLUTが必要。
GNG-VLUT回避器はMuJoCoの位置制御に接続。
開始姿勢を復帰目標とし、動作中の姿勢操作は名目目標の更新。描画・self_voxelはMuJoCo実測に追従。

bag設定は[arm_avoidance_bag.yaml](../../../../scripts/arm_avoidance_bag.yaml)を参照。
ブラウザ内GNGモードでは`remap: {}`で標準TFへ配信。共通ROS回避は既定の記録TF分離を使用。
入力は点群・観測ロボットTF・センサTFだけ。ROIと観測側自己マスクは再生成、制御対象のself_voxelは実測姿勢から更新。
記録姿勢・指令の仮想ロボットへの転用なし。起動は[共通ROS回避](../../../../gng_vlut_system/docs/dual_arm_simulation.md#センシング--ros回避--実行先)を参照。

観測時刻のTF不足・点群停止・視野外・遮蔽等で保持になる場合あり。
実測とノード間の接続は近似で、動的障害物に対する連続区間の保証なし。
対象bagでの全身重なり解消・往復タスク完遂は未達。

#### 回避優先方策（実験）

選択は`avoidance_first`。退避→保持→復帰の局所探索。
条件・再現設定は[比較結果](../../../../benchmarks/avoidance_first/results.json)を参照。
行き止まり時は保持。全身重なり解消・実機動作は未達。

#### 予測・不一致適応方策（実験）

選択は`vrm_predictive`。セッション内のVLUT状態履歴によるリスク予測。衝突確率ではない値。
条件は[再現設定](../../../../benchmarks/vrm_predictive/results.json)を参照。
既存方式に対する回避性能改善・未知運動への保証は未達。

### CLIでのMuJoCo・GNG-VLUT実行

workspaceルートで:

```bash
bash scripts/run_dual_arm_simulation.sh --duration 30
```

左側↔前側の往復動作だけの対照試験（回避なし）:

```bash
bash scripts/run_dual_arm_simulation.sh --avoidance none \
  --task-file /ros2_ws/src/benchmarks/shuttle_bag/task.json --duration 8 --min-round-trips 2
```

引数は`bash scripts/run_dual_arm_simulation.sh --help`、初期姿勢は[robot-poses.json](../../app/robot-poses.json)、往復設定は[task.json](../../../../benchmarks/shuttle_bag/task.json)を参照。
往復回数は手先の位置・姿勢・速度による実測到達。名目指令の切替や関節変位量は回数に算入しないこと。
`--goal-file`は部分関節目標のJSON［rad］。出力はJSONL、既存ファイルへの上書きなし。
終了コード: 0=正常、2=実行エラー、3=入力未成立／要求動作量・往復数未達。
`--duration 0`はCtrl+Cまで。終了時に所有bag・launch・ブリッジを停止、外部プロセスは継続。

表示は開始JSONの`viewer_url`。ブラウザ内物理・ROS状態送信はOFF、既定のデモ机・物体は非表示。
外部実測の表示URLは`?model=long&joint_topic=/sim/joint_states&state_source=simulation`。
URLの`layer=/機体名/control_voxels`は環境レイヤーの初期表示指定。`observe`で同じROSドメインの表示サーバーを指定。
`/sim/joint_states`の受信は書込み不可の専用接続。基台配置は[共通ROS回避](../../../../gng_vlut_system/docs/dual_arm_simulation.md#センシング--ros回避--実行先)を参照。

### 追跡障害物の実験

MuJoCo上の追跡障害物による回避比較。実センサ・実機出力なし。
CLI入口:

```bash
bash scripts/run_dual_arm_simulation.sh --scenario chasing_sphere \
  --scenario-file benchmarks/vlut_pursuit/vlut.json
```

設定は[追跡実験JSON](../../../../benchmarks/vlut_pursuit/vlut.json)を参照。
表示線は事前生成した往復基準の手先軌跡。CHOMP系は独自の離散共変勾配版、MoveItプラグインとは別実装。
事前計画の対象は自己形状・床・台座、動的障害物は既存の局所回避。基準軌道の生成器は[bag非停止実験](../../../../gng_vlut_system/docs/dual_arm_simulation.md#非停止のbag往復実験mujoco専用)と共用。
区間検証は関節補間の標本照合。連続衝突回避・実電力量最小・大域最適性の保証なし。
`vlut_graph`は保存済みGNG/VLUTを使用。VLUT対応の`gng_dynamic_kernel`が必要。
GNG候補方式・CBF系はrad/sの直接速度駆動専用。固定角度刻み・仮角度目標への換算なし。
従来の代表姿勢幾何照合・CBF系も比較用に保持。接触・追跡停滞・QP不成立の一般解消は未保証。
条件と結果は[比較検証](../../../../benchmarks/vlut_pursuit/repair_verification.json)を参照。

連続実行は記録なし、時間指定は記録あり。リロードで自動再生なし。
リンク型の原点はworld座標［m］、実験開始時に固定。実行中の原点変更は非対応。
接触／異常で終了、ブラウザを閉じても実行は継続。記録削除は元データも削除し取消不可。
実験の可動域は実機未照合。高速の往復回避・実機安全性の検証は未達。

### ロボットの用途・指令・制御

用途・指令元・制御方式の変更で物理を停止。明示的な開始操作で適用。
ROS実測の表示は描画専用、シミュレーションはMuJoCo実測を表示。

| 制御方式 | 即時指令 |
| --- | --- |
| `position` | position |
| `position_velocity` | position・velocity、任意のeffort |
| `velocity` | velocity |
| `torque` | effort |

指令は`/sim/joint_command`のJointTrajectory、即時1点。非ゼロtime_from_start・複数点・加速度は拒否。
単位はSI、URDF上限を適用。重力・コリオリ補償はトルク方式で既定OFF、他方式でON。
入力失効時は位置保持／速度ゼロ／トルクゼロ。リーダー入力と既存回避は位置制御専用。

操作APIのCLI（Simulatorルート）:

```bash
node tools/robot_control.mjs status
node tools/robot_control.mjs configure --usage simulation --source api --mode position
node tools/robot_control.mjs start
node tools/robot_control.mjs command --json '{"mode":"position","position":{"neck_pan_joint":0.2}}'
node tools/robot_control.mjs stop
```

複数画面は`--session ID`、回避器の名目目標は`goal`。
APIは`GET/POST /api/robot`。契約は[robot_control.mjs](../../tools/robot_control.mjs)を参照。実機出力の有効化は非対応。

### リーダー・フォロワーの接続選択

```bash
ros2 launch gng_vlut_system robot_follow.launch.py
```

接続設定と実行開始は別操作。設定だけでは追従・物理・実機出力を開始しない構成。
設定・起動条件は[共通追従構成](../../../../gng_vlut_system/docs/dynamixel_sim_control.md#srfの共通追従構成)を参照。
リーダー入力の失効・切断・範囲超過で保持。通信復旧だけでは再開せず物理の明示再開始が必要。

### ロボット配置・TF・ROS軌道の往復（2026-10-01追加）

基台配置はworld座標、XYZ［m］・RPY［deg］。
TF／Poseによる基台配置の受信追従は未対応。

| トピック | 内容 |
| --- | --- |
| `/sim/joint_states` | JointState |
| `/sim/base_pose` | PoseStamped |
| `/tf`・`/sim/tf` | 基台・リンク・取得時センサTF |
| `/sim/robot_description` | URDF、transient local |
| `/sim/command/{standard,long}/joint_trajectory` | 力学なしの軌道表示プレビュー |

同じTFの重複配信・複数ブラウザからの状態送信は禁止。
軌道表示はpositionsのみの線形補間。速度・加速度・effort付き軌道は拒否、接触保証・実機指令なし。
受信OFF・Esc・ページ非表示・モデル切替で表示再生を停止。
仕様は[robot_exchange.py](robot_exchange.py)を参照。

## 外部AI連携（別途導入）

AiS-GNG-FVG本体・observer・独自ROSメッセージが必要。送付版でのVM再接続は未検証。
外部AIのROS環境をsource後:

```bash
python3 integrations/ros2/robot_bridge.py
```

接続先は`http://localhost:8878`。
