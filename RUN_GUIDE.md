# ToPoFuzzy-Viewer 実行ガイド

##　実行ガイド
cd ~/uraki_ws
docker compose down && docker compose up --build  && docker compose up -d
docker compose exec gng_cpu bash
## frontendの起動
docker compose exec gng_cpu bash -lc 'cd /ros2_ws/src/ToPoFuzzy-Viewer/frontend && npm run build'

docker compose --profile manual up  frontend

chrome://restart

### AMD GPUでWebGLコンテキスト喪失が発生する場合

ホスト側のNVIDIA GPUを使う専用Chromeの起動:ビューアー安定化版

bash scripts/open_viewer_nvidia.sh


##  backendの起動
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py

## ロボットおよび対応する学習済みGNGの起動
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

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
