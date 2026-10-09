# MID-360実機 → ROS 2

対象: Linuxホスト、Docker Compose、ROS 2 Humble。公式Livox SDK2・livox_ros_driver2を固定リビジョンでReleaseビルド。GNGコンテナとは独立したセンサ用サービス。

## 接続・起動

1. MID-360を適合電源・Ethernetで接続。PCの有線LANにセンサと通信できる固定IPv4を設定。Wi-Fiと有線LANの同一サブネット競合に注意。
2. このフォルダの `mid360.yaml` の `host_ip` と `lidar_ip` を実値に変更。例: PC `192.168.1.5`、センサ `192.168.1.198`。例示IPのまま決め打ちしない。
3. ワークスペースルートから起動:

```bash
docker compose -f integrations/mid360/compose.yaml up --build
```

終了: 同じ端末でCtrl+C。次回からは `--build` 不要。ホストネットワーク使用、PCのNIC設定の自動変更なし。IP未設定・ホスト未割当なら起動時エラー。既存のLivoxドライバとの同時起動不可。

| 出力 | 型 | frame |
| --- | --- | --- |
| `/sensors/mid360/points` | sensor_msgs/msg/PointCloud2 | mid360_link |
| `/sensors/mid360/imu` | sensor_msgs/msg/Imu | mid360_link |

点群: 既定10 Hz、XYZ[m]・intensity等。色なし。IMU周期は点群のpublish_freqとは別。センサは1台を対象。

疎通確認はGNGコンテナ内でROS環境を読み込み:

```bash
docker compose exec gng_cpu bash
source /ros2_ws/install/setup.bash
ros2 topic info /sensors/mid360/points -v
ros2 topic hz /sensors/mid360/points
ros2 topic echo /sensors/mid360/points --once --field header
```

型の検出だけでなく、継続受信・frame・時刻を確認。受信側とドライバの `ROS_DOMAIN_ID` を統一（既定0）。別PCでDDS受信する場合は双方で同一Domain、`ROS_LOCALHOST_ONLY=0`、DDS通信を許可。LiDAR直結側ではUDP 56101/56201/56301/56401/56501の受信とセンサ向け通信を許可。ファイアウォール全体の無効化は不要。

## 取付TFとボクセル化

`mid360.yaml` の `pos` [m]・`rot_deg` [roll,pitch,yaw、deg] は親frameから測定原点への変換。単独利用時のみ `enable_mount_tf: true` で配信。Viewer連携時はfalseを維持。SDK側のextrinsicはゼロ固定のため二重変換なし。

Viewer連携時のTF接続: `topo_dual_arm_max_long/base_link → … → topo_dual_arm_max_long/chest_lidar_link → mid360_link`。Viewer側の機体YAMLで取付TFを配信するため、専用ドライバの `enable_mount_tf` はfalse。longのURDFが位置とpitch=45°を保持し、腰関節の実測角に追従。最後の変換はゼロの公称値で、45°の二重適用なし。腰角0の公称位置はbase_link基準で約[0.069326, 0, 0.352347] m、センサの+Xは前方斜め下。計測原点の差分は未校正であり、機体YAMLのmid360.pos・rot_degで補正。
IMUを融合する用途ではIMU軸・原点と点群frameの対応も別途確認。

GNG/VLUT用のViewer連携は `gng_vlut_system/config/topo_dual_arm_max_long.yaml` で選択:

```yaml
mid360:
  enable_input: true
  points_topic: "/sensors/mid360/points"
  enable_mount_tf: true
  parent_frame_id: "chest_lidar_link"
  frame_id: "mid360_link"
  pos: [0.0, 0.0, 0.0]
  rot_deg: [0.0, 0.0, 0.0]
```

`enable_input` の既定はfalse。trueでMID-360入力と環境ボクセル化を有効化し、PointCloud2のframeを使用。取付親frameにはrobot_nameを自動付与。ドライバの起動・IP設定は専用Compose側。通信設定とViewerの入力選択を分離。

GNGコンテナ内で起動:

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=topo_dual_arm_max_long.yaml
```

明示したlaunch引数 `environment_input_topic`・`enable_environment_voxelization` はYAMLより優先。実機では `use_sim_time:=false`（既定）を使用。

前提: longのURDF・左右GNG/VLUTデータ、実姿勢の関節情報（waist_jointを含む）、`topo_dual_arm_max_long/base_link → … → mid360_link` のTF。既にrobot_state_publisherが稼働中なら `enable_robot_state_publisher:=false` を追加して二重配信を回避。固定センサの場合は校正先の親frameを変更。

```bash
ros2 run tf2_ros tf2_echo topo_dual_arm_max_long/base_link mid360_link
ros2 topic hz /topo_dual_arm_max_long/self_filter_roi_voxels
```

点群とロボットの重なり、自己除去後の手・物体の残存、移動後のボクセル消去を確認。実機点群では壁時計を使用。シミュレーション時刻への混在や移動中のスキャン歪み補正は本構成の対象外。ここまでが認識入力の準備であり、実機アームへの回避指令出力は含まない。

## 検証

```bash
docker compose -f integrations/mid360/compose.yaml config --quiet
docker run --rm --network none \
  -v "$PWD/integrations/mid360:/check:ro" \
  uraki-mid360:local bash -c \
  'source /opt/livox_ws/install/setup.bash && PYTHONDONTWRITEBYTECODE=1 python3 /check/test_config.py'
```

取付TFの検証（GNGコンテナ、他の処理と異なるROS Domainで実施）:

```bash
docker compose exec gng_cpu bash -lc 'source /opt/ros/humble/setup.bash; ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 python3 /ros2_ws/src/integrations/mid360/test_mount_tf.py --urdf /ros2_ws/src/urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf'
```

確認済み: 腰角0°・90°でbase_linkからの位置・45°下向きの測定軸。検証用TFノードは終了時に停止。ROS Domain 97は検証専用とし、他用途で使用中なら変更。

実機未接続: 受信Hz・時刻同期・取付校正・実点群での自己除去は未検証。

公式仕様: [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)、[Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)。ROS 2では `xfer_format=0` のPointCloud2を使用。
