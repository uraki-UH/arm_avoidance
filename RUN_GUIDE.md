# 別PCでの双腕ブラウザ・ROS 2の起動（Ubuntu・NVIDIA不要）

ソース: [uraki-UH/arm_avoidance](https://github.com/uraki-UH/arm_avoidance)、開発ブランチ: `grasp_new4`。
非公開の場合は受領者のGitHubアカウントへのアクセス権が必要です。共有元でcommit・pushしていない変更はリンク先に含まれません。共有時に対象commitを伝え、双方で`git rev-parse HEAD`の一致を確認してください。

対象: Ubuntu系Linux、Docker EngineとDocker Compose plugin、WebGL 2対応のChrome / Edge / Firefox。ROS環境とNode.jsはコンテナ内へ導入するため、ホスト側のROS・Node.js・NVIDIAドライバ・CUDA・NVIDIA Container Toolkitは不要です。まずUbuntu 22.04相当のROS 2 Humble環境を使用します。別CPUアーキテクチャでのビルドは未検証です。

| 機能 | 実行条件 |
| --- | --- |
| ROS 2、MuJoCoの関節・接触計算、ブラウザなしの物理実行 | CPUのみ。基本構築で利用可能 |
| ブラウザのロボット・点群表示、RGB-D画像生成 | ホストブラウザのWebGL 2。NVIDIA専用機能なし。内蔵GPUも対象 |
| GNG／VLUT回避・ROS結果の表示 | 追加のROSビルドと機体設定に対応する学習済みデータ |
| AIS-GNGのライブ学習、OSCBF、人物検出、実機ドライバ、Isaac Sim | この基本構成の対象外。機能別の追加依存が必要 |

「NVIDIA不要」と「画面表示までCPUだけで高速に処理可能」は別条件です。画面なしのCPU物理実行が、GPU非依存の最小確認になります。初回の依存取得・ROSビルドと、準備後の起動も区別してください。初回はネットワーク通信と大きなディスク空き容量が必要で、所要時間は回線・CPUに依存します。

## ソース取得・基本構築

```bash
git clone --depth 1 --branch grasp_new4 https://github.com/uraki-UH/arm_avoidance.git
cd arm_avoidance
bash scripts/dual_arm_cpu.sh build
bash scripts/dual_arm_cpu.sh check
```

リポジトリ全体の配置が必要です。Simulatorフォルダー単独では、UIビルドが参照する`ToPoFuzzy-Viewer/frontend/src/`・`libs/ros_visualization_web/`や、ROSの独自メッセージが不足します。

`build`はCPUイメージ、基本ROSメッセージ4パッケージ、ブラウザUIの準備です。初回のnpm依存取得とUIビルドは、不足ファイルがある場合に実行します。UIソース変更後の明示再ビルドは、`shell`でアプリのルートから`npm ci && npm run build`。CPUイメージではPyTorch・YOLO・PyG・Emscripten・USDの追加インストールを省略します。既存の通常Docker構成は従来どおり開発用イメージを使用します。

`check`はROSメッセージ・Python依存の読込み、C++点群処理の初回コンパイル、1秒間の画面なしMuJoCo実行です。成功時はJSONの`summary`が出力され、有限時間で終了します。ここではGNG回避の成功を判定しません。

## ブラウザとROS 2の起動・終了

```bash
bash scripts/dual_arm_cpu.sh start
```

端末を開いたまま、ホストの通常ブラウザで **http://127.0.0.1:8877/?model=long** を開きます。他PC向けのこの手順では、NVIDIA／Intel用のブラウザ起動フラグは指定しません。ROS送信先は`http://127.0.0.1:8879`、関節・物理接続はその次のポートです。物理計算は「物理を開始」で開始、回避モジュールを使用しない基本確認から進めてください。ブラウザを閉じただけではROSサービスは停止しません。

終了は起動端末でCtrl+C。その後のコンテナとネットワークの片付けは次のコマンドです。専用のbuild/install領域は保持します。

```bash
bash scripts/dual_arm_cpu.sh stop
```

この構成はUSBデバイス、X11認証、ホストGPU、privileged、実機ドライバを要求しません。ROS 2通信のためhost networkとIPCを使用します。既存の開発用`gng_cpu`とは別のComposeプロジェクト・build/install volumeです。両者を同時に起動すると既定ポートが競合するため、同じPCでは利用する構成を選んでください。ROSドメインを分離する場合は、build後の各コマンドと連携ノードへ同じ`ROS_DOMAIN_ID`を指定します。

```bash
ROS_DOMAIN_ID=38 bash scripts/dual_arm_cpu.sh start
```

## 画面なしのCPU実行・ROS確認

```bash
bash scripts/dual_arm_cpu.sh simulate --avoidance none --duration 3
bash scripts/dual_arm_cpu.sh shell
```

`shell`内はROS 2 Humbleと構築済み独自パッケージを読込み済みです。別端末で`start`を起動した状態なら、`ros2 node list`・`ros2 topic list`でブリッジを確認できます。シミュレータの点群・関節送信はブラウザで明示的に有効化します。未送信のセンサがトピック一覧にない状態は正常です。`exit`で一時シェルを終了します。

## GNG／VLUT回避まで使う場合

```bash
bash scripts/dual_arm_cpu.sh build-gng
```

`build-gng`はGNG・VLUTとROS表示ゲートウェイを追加構築します。既定はパッケージを順番に、C++の同時コンパイル数1で処理し、メモリ消費を抑えます。余裕があれば`CMAKE_BUILD_PARALLEL_LEVEL=2`などを指定できます。通常起動時の学習や再コンパイルはありません。ここで追加するのは腕のGNG／VLUT計画・判定と表示です。PyTorch依存のAIS-GNGライブ学習コンポーネントは含みません。

**GitだけではLongの学習済みデータは揃いません。** `gng_results/topo_dual_arm_max_long_independent/`のmanifest・model.jsonは追跡されていますが、左右の`gng.bin`・`vlut.bin`は別途必要です。共有元と同じURDF・設定・固定関節条件に対応するデータを同じ相対配置へ置くか、[独立腕モデルの学習手順](gng_vlut_system/README.md#双腕学習viewerの既定モデル)に沿って生成します。`check-gng`は実ファイルの存在とモデル情報の整合性を確認し、不足時は終了コードを返します。データの代わりに空ファイルを置いて先へ進めないでください。

追加データ配置後の確認と画面なし起動:

```bash
bash scripts/dual_arm_cpu.sh check-gng
bash scripts/dual_arm_cpu.sh simulate --avoidance gng --duration 10
```

実点群の回避検証には入力のPointCloud2・TFなども必要です。入力なしの待機状態を回避成功として扱いません。終了コード3は、入力待ちなどの準備未完了、または指定した動作量の未達です。bag入力・任意目標・ROS結果の表示・トピック接続は[SimulatorのROS連携手順](ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md)を参照してください。ROS表示ゲートウェイは別途起動が必要で、コンテナ内では`source /ros2_ws/install/setup.bash`の後に`ros2 run topo_fuzzy_viewer viewer_ws_gateway_node`を実行します。

## 起動できない場合

- Dockerの接続権限エラー: ホスト側のDocker Engine起動と利用権限を確認。コンテナ内へのNVIDIA導入は不要。
- WebGL 2の初期化失敗: ブラウザのハードウェアアクセラレーションと、そのPCの標準GPUドライバを確認。先に画面なしの`check`でROS・CPU計算を切り分け。
- 8877／8879／8880番の競合: 利用中の別構成を確認し、この構成と重複起動しないこと。既存サービスの自動停止はなし。
- 学習済みデータ不足: `check-gng`の対象ファイルと上記の追加条件を確認。
- ソースと画面が不一致: `shell`内で`npm ci && npm run build`後、ブラウザを再読み込み。ROS・C++変更後は対応するbuild操作と、この構成の再起動。

---

# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド
cd ~/uraki_ws
docker compose down && docker compose up --build  && docker compose up -d
docker compose exec gng_cpu bash
## frontendの起動
docker compose exec gng_cpu bash -lc 'cd /ros2_ws/src/ToPoFuzzy-Viewer/frontend && npm run build'

docker compose --profile manual up  frontend

chrome://restart

## 実機リーダー・フォロワー／ブラウザ物理フォロワー

コンテナ内の対話端末で、更新済み`dynamixel_handler`を別途起動したうえで:

```bash
ros2 launch gng_vlut_system dynamixel_leader_follower.launch.py
```

既定は実機出力OFF。設定は[gng_vlut_system/config/dynamixel_leader_follower.yaml](gng_vlut_system/config/dynamixel_leader_follower.yaml)。実機へ出力する場合は、校正・支持・機種とモードの確認後に`allow_hardware_output`と`max_current_ma`を明示設定。Hで保持準備、Fで追従、Spaceで保持停止、Eで選択IDトルクOFF、Ctrl+Cで所有出力のOFF要求と終了。

既定対象: 右腕1〜7→31〜37、左腕11〜17→41〜47。グリッパー・首は校正前の対象外。実機駆動は未検証。[準備・制約・操作](gng_vlut_system/docs/dynamixel_sim_control.md#実機リーダーフォロワー制御)。

ブラウザ側は「ロボット → 姿勢の入力元 → リーダー → 物理フォロワー」を選択し、「物理 → 関節動力学 → 物理を開始」。入力`/leader/joint_states`をモータ目標へ反映し、物理の実姿勢を描画。実機出力OFFでも利用可能。既存ROS／物理ブリッジの更新はSimulatorフォルダで`bash start_ros.sh --restart`とブラウザ再読み込み。実機トルクONは不要。

## 両腕の重力補償付き手動操作

3 Mbps handlerを起動したまま、コンテナ内で:

```bash
ros2 launch gng_vlut_system dynamixel_hand_guiding.launch.py
```

既定は出力なしの計算プレビュー。対象: 31〜37・41〜47、首・グリッパー除外。
設定: [dynamixel_hand_guiding.yaml](gng_vlut_system/config/dynamixel_hand_guiding.yaml)。
実機有効化には関節別の許容電流・トルク換算、URDF質量・重心・符号・原点の校正が必要。未校正のまま出力許可だけ変更しないこと。
実機の重力支持は未検証。終了・異常時のOFFによる落下に備えた腕の支持が必要。
[制御条件・起動終了・制限](gng_vlut_system/docs/dynamixel_sim_control.md#両腕の重力補償付き手動操作)。

### AMD GPUでWebGLコンテキスト喪失が発生する場合

ホスト側のNVIDIA GPUを使う専用Chromeの起動:ビューアー安定化版

bash scripts/open_viewer_nvidia.sh


##  backendの起動
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py

## ロボットおよび対応する学習済みGNGの起動
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max_long.yaml

元マップは`/ToPoDualArm/Tmap_static`、集約マップは`/ToPoDualArm/Tmap_vis_L0`。ViewerのTopicsで表示を選択。追加launchは不要。[集約データの再生成](gng_vlut_system/docs/releases/2026-09-15_spatial_tmap_aggregation.md)。

新しい双腕モデルは`params_file`を`topo_dual_arm_max.yaml`または`topo_dual_arm_max_long.yaml`へ変更。ロボット本体は学習前でも表示可能。GNG・VLUTの表示には機種ごとの学習後にlaunchを再起動。[設定・学習手順](gng_vlut_system/docs/releases/2026-09-28_dual_arm_models.md)。

### FuzzBotの表示専用起動

初回のみDocker内でモデル・メッシュのパッケージをビルド。親ディレクトリの`COLCON_IGNORE`は維持、実機ドライバは対象外。

```bash
cd /ros2_ws
colcon build --paths /ros2_ws/src/fuzzbot_gng/fuzzbot/fuzzbot_description --packages-select fuzzbot_description --symlink-install
source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/fuzzbot.yaml
```

既存Viewerへの`fuzzbot`の原点表示。初回の車輪ゼロ姿勢だけ配信、走行指令・実機接続・Gazebo・学習・自己認識ボクセルは未起動。
設定は[fuzzbot.yaml](gng_vlut_system/config/fuzzbot.yaml)。`joint_control_backend: external`で関節指令経路を省略。明示launch引数が最優先、YAML未設定時は従来の`viewer`。
学習データ不足の通知は未学習時の正常動作。ロボット表示にGNG・VLUTは不要。停止は起動ターミナルで`Ctrl+C`。
起動・TF・配信の隔離検証: Docker内で`python3 -B /ros2_ws/src/gng_vlut_system/test/check_fuzzbot_viewer.py`。ROSドメイン228の未使用が前提、所有launchは試験終了時に停止。

### FuzzBotの移動状態グラフ

Docker内で、上記のロボット表示とは別ターミナルから起動。

```bash
source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system mobile_state_graph.launch.py
```

既定設定は`fuzzbot.yaml`。初回生成後に保存、次回は整合性検査付きで読込。ViewerのTopicsで`/fuzzbot/state_graph/markers`をON。点は状態の位置、短線は向き、青線は遷移軌跡。点の色は並進速度（赤寄りが後退、緑寄りが前進）。同じ位置で向き・速度が異なる状態は表示上重複。

- 状態: `(x, y, yaw, v, omega)`。有向エッジ: 一定の並進加速度・角加速度による遷移。
- 生データ: `/fuzzbot/state_graph/data`（JSON）。全状態・有向接続・加速度・遷移時間・単位を保持。
- 保存先: `/ros2_ws/install/gng_vlut_system/share/gng_vlut_system/gng_results/fuzzbot/motion_graph.json`。`output_path:=/任意の場所/motion_graph.json`で変更可能。
- 生成だけ: 同じlaunchに`build_only:=true`を追加。生成後は自動終了。
- 設定変更後: `enable_rebuild:=true`で保存グラフを再生成。指定なしの設定不一致はエラー（古いグラフの黙用防止）。
- 停止: `Ctrl+C`。表示中の反復生成なし。ノード停止後の新規購読への配信なし。

設定は[fuzzbot.yaml](gng_vlut_system/config/fuzzbot.yaml)の`motion_graph`。既定は原点の静止状態から探索、半径2 m以内・最大3,000状態。半径内の全域や全速度の被覆保証なし。車輪半径・輪間隔・車輪速度上限はURDFから取得、加減速上限はシミュレーション用の仮値。

現段階は平坦床・横滑りなしの運動モデルによる事前生成であり、GNG学習・障害物照合・実機走行指令は対象外。実ロボットの到達保証なし。現在位置への再配置や現在速度に応じた検索、エフェクティビティ評価は未接続。任意URDFへの対応ではなく、`libgazebo_ros_diff_drive.so`設定と左右車輪の速度制限を持つ差動二輪モデルが対象。

隔離起動試験: `python3 -B /ros2_ws/src/gng_vlut_system/test/check_mobile_state_graph.py`。未使用のROSドメイン227で生成・後発購読・保存再利用を検証、試験launchは終了時に停止。

## 双腕Gazeboデモ

```bash
ros2 launch gng_vlut_system dual_arm_gazebo_demo.launch.py enable_auto_start:=true
```

左腕・右腕・両腕・グリッパーを1巡。`gui:=false`で画面なし。停止は`ros2 service call /sim_topo_dual_arm_max/demo/stop std_srvs/srv/Trigger '{}'`。[設定・longへの切替・検証結果](gng_vlut_system/docs/releases/2026-09-28_dual_arm_gazebo_demo.md)。

## 人の前腕接近に対する退避デモ

```bash
ros2 launch gng_vlut_system dual_arm_avoidance_demo.launch.py
```

左・右へ前腕カプセルが接近し、距離に応じて退避・復帰。Viewerで`/sim_topo_dual_arm_max/avoidance/markers`をONにすると、前腕・手首軌跡・距離を表示。
停止は`ros2 service call /sim_topo_dual_arm_max/avoidance/stop std_srvs/srv/Trigger '{}'`。
[回避設定・検証](gng_vlut_system/docs/releases/2026-09-28_dual_arm_avoidance_demo.md) / [位置制御・物理パラメータ](gng_vlut_system/docs/dual_arm_simulation.md)。

## 点群・GNG・VLUTによる双腕退避

```bash
ros2 launch gng_vlut_system dual_arm_gng_lidar_demo.launch.py
```

maxの10,000ノード学習済みデータを利用。LiDAR点群の自己除去、VLUTによる安全状態更新、GNG経路と局所退避を接続。
Viewerは`/sim_topo_dual_arm_max/lidar_points`、`Tmap_static`、`plan_Tmap`、`avoidance/markers`を表示ON。
[設定・検証範囲](gng_vlut_system/docs/releases/2026-09-28_dual_arm_gng_lidar.md)。

## ロボットを座標変換
python3 test_tf_publisher.py --world-frame world --frame-id ToPoDualArm/base_link --x 0.35 --y 0.15 --z -0.3 --yaw 3.2

graspnet用
python3 test_tf_publisher.py --world-frame world --frame-id ToPoDualArm/base_link --x 0.15  --y 0.0 --z -0.2 --yaw 3.2

rosbag用？
python3 test_tf_publisher.py --world-frame world --frame-id ToPoDualArm/base_link --x 0.3 --y 0.0 --z -0.2   --yaw 3.14


## GNG平面クラスタから上方向把持候補を生成
ros2 launch grasping_system top_grasp_pose_candidates.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

`ais_gng.launch.py`でCPU GNGと平面クラスタを起動した状態で、上面把持候補を生成する。

- ID・姿勢・到達性状態: `/grasp_pose_cands` (`gng_control_msgs/msg/GraspCandidateArray`)
- 候補スコア: `/grasp_pose_cand_scores`
- 判定概要: `/grasp_pose_cands/summary`

## 把持候補の関節角度・候補軌道の出力
ros2 launch gng_vlut_system grasp_joint_candidates.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

`topological_map_path_planner_node`で候補経路・評価・`final_joint_state`を出力。回避ノードの起動、追従・退避、関節指令の配信なし。Viewerの候補ロボット表示だけを止める場合は`publish_candidate_robot_preview:=false`を追加。

## 実点群による把持幅・姿勢の追加補正
ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py

上記の候補・関節候補と実点群が必要。`/grasp_pose_refined/markers`に把持幅の線枠と進入方向の矢印を表示。文字なし、幅未計算は灰色矢印のみ。`candidate_goal_preview`は置換しない。設定は`gng_vlut_system/config/grasp_candidate_refinement.yaml`。点群名が異なる場合は`point_cloud_topic:=<入力名>`を追加。[描き分け・検証結果](gng_vlut_system/docs/releases/2026-09-15_grasp_geometry_markers.md)。
 

## HTML起動
python3 -m http.server 8000
http://localhost:8000/ToPo-FUZZY_Manipulation_v1.html


### 点群をToPoDualArmのVLUTへ反映
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml


## AISGNG実行
ros2 launch ais_gng ais_gng.launch.py   backend:=cpu   lidar:=graspnet.yaml


上方把持候補は同じ平面クラスタIDが既定5更新連続で有効になってから公開し、既定2更新の短期欠測は保持する。
`candidate_frame: ""` は入力座標系のままでTF変換なし。ロボット基準にする場合は `candidate_frame: "ToPoDualArm/base_link"` とし、外部センサTFをURDFまたは実機bringupから配信。

`grasp_joint_candidates.launch.py`の既定入力へ接続。上方方式とボクセル方式は同じ出力先のため、候補生成はどちらか一方だけ起動。比較時は出力トピックを分離し、名前付きYAMLの出力設定とlaunch引数を同じ値へ変更。

## RVizでロボットを表示
ros2 launch gng_vlut_system visualize_robot_rviz.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm


==============================================================

# URDF準拠のダミー関節状態
ros2 launch gng_vlut_system dummy_joint_pub.launch.py \
  urdf_path:=/ros2_ws/src/<robot_package>/<robot>.urdf

ros2 launch gng_vlut_system dummy_joint_pub.launch.py \
  urdf_path:=/ros2_ws/src/urdf/dual_arm_urdf/dual_arm_robot.urdf


## GNGの学習の実行
  ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true \gng_profile_names:=left_arm 
  (initial_collision_only:=true):初期姿勢での衝突リンクの組み合わせを検証

## 衝突urdfの球化
ros2 launch gng_vlut_system voxel_spherized_robot_viewer.launch.py  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml


ros2 launch gng_vlut_system topological_map_avoidance.launch.py   params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml   trial_mode:=true   trial_safe_only:=true ( trial_return_home:=true
)

ros2 launch gng_vlut_system target_joint_state_executor.launch.py   robot_name:=ToPoDualArm   target_topic:=target_joint_states   state_topic:=joint_states   command_topic:=joint_commands   max_joint_velocity:=0.6   publish_hz:=20.0


python3 -m pip install --user torch==2.8.0 torchvision --index-url https://download.pytorch.org/whl/cpu


## GNGノードを把持候補用ボクセルへ変換
`/topological_map`と`/scan/transformed`を照合し、点群支持のある物体候補をボクセル化。
`SAFE_TERRAIN`、`HUMAN`、`CAR`は候補から除外。

ros2 launch ais_gng topological_grid.launch.py \
  input_topic:=/topological_map \
  pointcloud_topic:=/scan/transformed \
  output_topic:=/topo_voxel_ids \
  grid_size:=0.02

## 把持ボクセル照合（左グリッパ、POC）
ボクセルとグリッパ体積、平面クラスタなどを使って  tcp候補を得る

ros2 launch grasping_system grasp_voxel_matcher.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml


## Gazeboに召喚
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true

## Gazeboでベースをワールドに固定したい場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  spawn_z:=0.0 \
  fixed_base_link:=base_footprint

## Gazeboのピックアンドプレース用worldを使う場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  spawn_z:=0.0 \
  fixed_base_link:=base_footprint \
  world:=/ros2_ws/src/gng_vlut_system/worlds/pick_and_place.world

## Gazeboの点群・GNG検証用起動
`gng_vlut_system/config/gazebo_pick_and_place.yaml`で設定管理　　点群トピック　/lidar_points

ros2 launch gng_vlut_system gazebo_pick_and_place.launch.py

設定ファイルだけを差し替える場合は次のとおり。

```bash
ros2 launch gng_vlut_system gazebo_pick_and_place.launch.py \
  gazebo_params_file:=/ros2_ws/src/gng_vlut_system/config/gazebo_pick_and_place.yaml
```

現在の`dual_arm_robot.urdf`には`gazebo_ros2_control`がないため、Gazebo内の関節と
グリッパを指令して実際に把持するには、別途Gazebo用controller接続が必要。

## GazeboでTF追従させたい場合
ros2 launch gng_vlut_system robot_gazebo_sim.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  robot_name:=ToPoDualArm \
  gui:=true \
  follow_tf_frame:=ToPoDualArm/base_footprint






必要に応じて
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

alias sh='source /opt/ros/humble/setup.bash'
alias sw='source /ros2_ws/install/setup.bash'


## 左腕をtopological_map_avoidanceで動かす
ros2 launch gng_vlut_system topological_map_avoidance.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  trial_mode:=true \
  trial_safe_only:=true


すべてのプロセスをキル　frontendを除く
./scripts/stop_ros2_stack.sh

docker compose --profile manual up -d --build frontend

## GNGエッジから差分方式で平面クラスタを作る
ros2 launch ais_gng plane_cluster_incremental.launch.py \
  input_topic:=/topological_map

今はais_gng_実行で生成できるようにしている

## 点群から占有ボクセルに変換
ros2 launch gng_vlut_system point_to_voxel.launch.py \
  input_topic:=/semantic_points \
  output_topic:=/topo_voxel_ids

ros2 launch gng_vlut_system point_to_voxel.launch.py \
  input_topic:=/camera/camera/depth/color/points \output_topic:=/topo_voxel_ids



### depth画素handle付きpersistent world indexの比較

固定カメラのraw depthからpersistent world indexを構築し、全再構築方式と比較する。
実機では`camera_world_*`へ外部パラメータを設定する。


# ターミナル2: raw depthとcamera_infoを同時に配信
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41 \
  --topics \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info


## ダミー把持候補を状態付き配列で流す
ros2 launch gng_vlut_system grasp_pose_dummy_publisher.launch.py \
  frame_id:=world \
  candidate_count:=1


### 大容量PointCloud2再生用UDPバッファ
```bash
sudo install -m 0644 docker/ros2_fastdds_udp_buffers.conf \
  /etc/sysctl.d/99-ros2-fastdds-udp-buffers.conf
sudo sysctl -p /etc/sysctl.d/99-ros2-fastdds-udp-buffers.conf
```

ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41_transformed  --loop

ros2 launch graspnet_ros2 play_scene.launch.py scene_id:=3 camera:=realsense start:=10 end:=20 hz:=20.0
