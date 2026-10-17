# MID-360実機 → ROS 2

Linux・Docker Compose・ROS 2 Humble。Livox SDK2／livox_ros_driver2の専用サービス。
設定は[mid360.yaml](mid360.yaml)を参照。

## ドライバの導入（実機接続不要）

ホストのworkspaceルートで:

```bash
docker compose build mid360
```

GNGコンテナのビルドとは独立。通常のCompose起動では対象外。

## 固定IP設定込みの自動起動

電源・Ethernetを接続し、設定YAMLを確認後、ホストで:

```bash
python3 hardware/mid360/start.py --check
python3 hardware/mid360/start.py
```

有線LAN選択・PC固定IP設定・ドライバ起動。IP設定時のみsudo認証。
終了はCtrl+C。IP設定は維持。解除は`sudo nmcli connection down mid360-direct`。
複数LAN・既存IPとの競合は自動変更せず停止。

## 接続先の確認・手動設定

```bash
nmcli device status
ip -brief address
ip -4 route
```

LiDARのIPは本体設定・Livox Viewer 2で確認。工場出荷時は`192.168.1.1XX`（SN末尾2桁）。
相手IP不明時の受信確認は`sudo timeout 10s tcpdump -nn -e -i <NIC> 'arp or udp'`。
PCのIP設定だけではセンサ側IPは変化しない。

既にPCの固定IPを設定済みなら:

```bash
docker compose run --rm --no-deps mid360 python3 /opt/mid360/mid360.launch.py --check-config
docker compose up mid360
```

終了はCtrl+Cまたは`docker compose stop mid360`。別のLivoxドライバとの同時起動不可。
設定変更は再起動、コード・Dockerfile変更は再ビルド後に反映。
`Cannot assign requested address`はPCへのIP未割当。`--check-config`成功はセンサ疎通の保証ではない。
旧`hardware/mid360/compose.yaml`とルートComposeは別プロジェクトのため重複起動不可。

| トピック | 型 | frame |
| --- | --- | --- |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | `mid360_link` |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_frame` |

受信側と`ROS_DOMAIN_ID`を統一。別PC受信は`ROS_LOCALHOST_ONLY=0`とDDS通信許可が必要。
LiDAR側UDPポートは56101・56201・56301・56401・56501。

```bash
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --field header
```

## 取付TFとボクセル化

GNGコンテナ内で:

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=topo_dual_arm_max_long.yaml
```

設定は[機体YAML](../../gng_vlut_system/config/topo_dual_arm_max_long.yaml)を参照。
独立したROS回避への入力は[共通手順](../../gng_vlut_system/docs/dual_arm_simulation.md#センシング--ros回避--実行先)を参照。

前提: 観測側ロボットの実測JointStateと、`world → ロボット基台 → … → chest_lidar_link → mid360_link`のTF。
外部TFの配信済み部分は重複起動しないこと。

取付pitch=45°はURDF、計測軸yaw=90°は機体YAML。専用ドライバ側の取付TFはOFF、SDK extrinsicはゼロ。
取付TFと補正の保存は機体launchの校正管理ノード。設定は[校正ストア](../../gng_vlut_system/config/sensor_calibration.yaml)。IMU内部TFや利用側の校正対応が未確認のまま、IMU利用中の補正更新は不可。
45°の二重適用は禁止。計測原点の位置校正は未完了。固定センサは設置先へTFを校正。
IMU融合では`livox_frame`の別原点・加速度g単位からSIへの変換が必要。

環境点群の学習:

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=mid360.yaml
```

学習設定は[ais_gngのmid360.yaml](../../ais_gng_cpu/src/ais_gng/config/gng_cpu/mid360.yaml)を参照。
同名の通信設定とは別ファイル。範囲変更後はGNGを再起動。world固定地図には自己位置推定を含むTFが別途必要。

## 検証

Releaseビルド・設定検査・腰角変更時の取付TFを確認済み。
実機読取りは10秒で点群100件・IMU 1,994件。yaw補正後の重力軸誤差0.67°。
位置校正・絶対方位・時刻同期・実点群での自己除去・自動IP設定からの通し起動は未検証。
検証入口は[test_start.py](test_start.py)・[test_config.py](test_config.py)・[test_mount_tf.py](test_mount_tf.py)。

仕様根拠: [Livoxマニュアル](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)・[公式ドライバ](https://github.com/Livox-SDK/livox_ros_driver2)・[SDK2](https://github.com/Livox-SDK/Livox-SDK2)。
