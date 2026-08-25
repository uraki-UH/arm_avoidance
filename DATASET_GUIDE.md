# データセット取得・再生ガイド

`dataset_pointcloud_player`によるデータセットの取得、ROS 2 PointCloud2再生、およびToPoFuzzyViewerでの表示手順。

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
