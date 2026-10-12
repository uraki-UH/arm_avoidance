# データセット取得・再生ガイド

`dataset_pointcloud_player`によるデータセットの取得、ROS 2 PointCloud2再生、およびToPoFuzzyViewerでの表示手順。

## ROS 2 bagの必要トピック限定再生

入口: `scripts/play_bag.sh`。ROS 2環境、Python 3、PyYAMLが必要。Docker内・source済みホスト環境の両方に対応。対象は`metadata.yaml`を含むbagディレクトリで、元bagの変更・別bagへの書出しなし。

ROSコンテナ内の例（`/rosbag/取得したbag`を実際のディレクトリへ変更）:

```bash
# 再生せず、選択対象とコマンドだけ確認
bash /ros2_ws/src/scripts/play_bag.sh /rosbag/取得したbag --dry-run

# 選択したトピックだけループ再生
bash /ros2_ws/src/scripts/play_bag.sh /rosbag/取得したbag --loop
```

既定の許可リスト: [`scripts/bag_topics.txt`](scripts/bag_topics.txt)。RGB-D点群`/camera/camera/depth/color/points`、LiDAR点群`/livox/lidar`、`/tf`、`/tf_static`、自己除去前の`/topo_dual_arm_max_long/roi_voxels`。自己マスク`self_voxel`・除去後の`self_filter_roi_voxels`・画像・関節状態・その他のGNG/VLUT計算結果・制御指令は既定で再生対象外。用途は生点群・記録ROIからの処理再計算で、すべての用途に必要な入力を自動判定するものではない。同名ROIを稼働ノードで再計算する場合は、別の許可リストからROIを外して重複配信を回避。

同形式の新しいbagはパスのみ変更。異なる収録構成は許可リストを別ファイルで保存し、`--topics-file /path/to/topics.txt`で指定。1行1トピック、`#`以降はコメント。`*`と`?`のパターンにも対応。bag内に存在し、記録件数があるトピックだけ選択し、不在・0件は警告、選択結果0件は再生せず終了。

```bash
# 記録されたロボット姿勢を追加
bash /ros2_ws/src/scripts/play_bag.sh /rosbag/取得したbag \
  --topic /topo_dual_arm_max_long/joint_states --loop --rate 0.5
```

`--loop`、`--rate`、`--clock`、`--remap`、QoS上書きなどは直接指定し、`ros2 bag play`へ転送。bagパスは再生オプションより前に指定。従来の`-- --loop`も利用可能。トピックの選択は`--topics-file`／`--topic`のみ。先読みキューは既定50メッセージで、`--read-ahead-queue-size`による変更可能。

回避用の`play_arm_avoidance_bag.sh`は同じ共通処理へ接続し、既定の2026-10-09 22:01:23 bagを再生。ブラウザ回避に必要な記録ROI・記録自己マスクを既定で追加し、カメラ取付TFも同時起動。`--joint-states`は記録姿勢の追加用、従来の`--roi-voxels`も受付可能（重複選択なし）:

```bash
bash /ros2_ws/src/scripts/play_arm_avoidance_bag.sh --loop
```

既存の全件再生を止めずに追加起動すると、同一トピックへの重複配信が発生。切替前に既存再生を停止。記録TFと稼働中ロボットの同名TFの競合にも注意。`/tf`内のフレーム単位の抽出・座標変換は対象外。`/clock`は明示指定時に生成、ROS内部の`/rosout`等はbagトピックの選択対象とは別。

### 未収録のカメラ取付TF

`/tf_static`の選択だけでは、bagに記録されていない親子接続の復元は不可能。2026-10-12確認の`arm_avoidance`内4収録（2026-10-09の22:01:23／22:05:49／22:06:34／22:07:41）は、`topo_dual_arm_max_long/camera_link → camera_link`が未収録。回避用bashは、以下の既存launchを同時起動するため、別ターミナルでの起動は不要:

```bash
ros2 launch gng_vlut_system realsense_mount_tf.launch.py \
  parent_frame:=topo_dual_arm_max_long/camera_link child_frame:=camera_link
```

変換値は既存の`realsense_mount.yaml`（ゼロ補正、測量校正は未実施）。Ctrl+C・再生完了・片方のプロセス終了時に、自分で起動した再生とTFを両方停止。`--dry-run`は両方のコマンド表示のみで配信なし。同じ子フレームを別ノードまたはbagが配信済みの場合は`--no-camera-tf`で同時起動を無効化。TFの既存配信の自動判定はなし。汎用の`play_bag.sh`単体には機体固有TFの追加なし。

22:01:23のbagにはRealSense内部TF（`camera_link → camera_depth_frame`等）も存在。後の3収録にはそれらも未収録で、上記の一本だけではカメラの完全なTFツリーは復元不可。以後の収録では取付TFとRealSense内部TFの配信を確認し、`/tf_static`のTransient Localデータも受信・保存できていることを検証。

## dataset_playerコンテナの作成

```bash
cd /home/uraki/uraki_ws
docker compose --profile dataset up -d --build
```

ソース変更後に`dataset_player`だけを再ビルドする場合、image再作成は不要。対象コンテナを再作成すると起動時に`colcon build`が実行される。

```bash
docker compose --profile dataset up -d --force-recreate dataset_player
```

実機ドライバも同時に使う場合は、両profileを指定する。

```bash
docker compose --profile dataset --profile hardware up -d --build
```

## RGB-Dデータセット

### OCID

```bash
docker compose exec dataset_player bash -c '
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash &&
ros2 launch dataset_pointcloud_player dataset_pointcloud_player.launch.py \
  dataset_path:=/datasets/OCID'
```

## 実機LiDARデータセット

### Hilti 2023 Robot

```bash
docker compose exec dataset_player bash -c '
cd /ros2_ws/src/dataset_pointcloud_player &&
bash scripts/download_hardware_dataset.sh --root /datasets --datasets hilti2023_robot &&
bash scripts/convert_ros1_bag.sh \
  /datasets/hardware/hilti_slam_2023/rosbags/site2_robot_2.bag \
  /datasets/hardware/hilti_slam_2023/rosbags/site2_robot_2_pointcloud_ros2 \
  --topics /rslidar_points &&
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash &&
ros2 launch dataset_pointcloud_player hardware_bag_player.launch.py \
  bag_path:=/datasets/hardware/hilti_slam_2023/rosbags/site2_robot_2_pointcloud_ros2 \
  loop:=true'
```

ToPoFuzzyViewerでは`/rslidar_points`を選択して有効化する。

### The Great Outdoors

```bash
docker compose exec dataset_player bash -c '
cd /ros2_ws/src/dataset_pointcloud_player &&
bash scripts/download_field_dataset.sh --root /datasets'

docker compose exec dataset_player bash -c '
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash &&
ros2 launch dataset_pointcloud_player lidar_dataset_player.launch.py \
  dataset_path:=/datasets/the_great_outdoors \
  output_topic:=/dataset/outdoor/points \
  loop:=true'
```

### Forest RTK GNSS

```bash
docker compose exec dataset_player bash -c '
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash &&
ros2 launch dataset_pointcloud_player hardware_bag_player.launch.py \
  bag_path:=/datasets/hardware/forest_rtk_gnss/rosbag2_2023_04_11-16_40_36 \
  loop:=true'
```

## 人物を含むデータセット

### Bonn dynamics_0

```bash
docker compose exec dataset_player bash -c '
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash &&
ros2 launch dataset_pointcloud_player laser_scan_bag_player.launch.py \
  bag_path:=/datasets/human/bonn_dynamics_0/dynamics_0_ros2 \
  scan_topic:=/laser_scan_front/scan \
  output_topic:=/dataset/human/bonn_front/points \
  loop:=true'
```

### REveL

初回は、取得と補正用ROS 2 bagの準備を行う。

```bash
docker compose exec dataset_player bash -c '
cd /ros2_ws/src/dataset_pointcloud_player &&
bash scripts/download_human_dataset.sh --root /datasets'
```

Vicon姿勢と`calibration.yaml`のLiDAR外部キャリブレーションを用いた補正再生。

```bash
docker compose exec dataset_player bash -c '
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash &&
ros2 launch dataset_pointcloud_player revel_bag_player.launch.py \
  loop:=true'
```

`/dataset/points`は、各点群時刻の`world ← Vicon marker ← camera ← LiDAR`変換を直接適用した座標である。REveL session_1のVicon worldはROSのz-up座標と一致しないため、静的点群平面と人物軌跡から検証した上向き軸をROSの`+Z`へ静的補正する。さらに、検出した床面候補が`z=0`となる`world_z_offset:=2.584`を適用する。初回LiDAR姿勢への相対化や、フレームごとの床面推定は行わない。Vicon座標を無変換で使う場合は`enable_vicon_up_alignment:=false world_z_offset:=0.0`を指定する。外部向けの点群出力はこのtopicだけ。再生中の生LiDAR点群は内部topic `/_dataset_player/revel/raw_points` だけで使用する。
