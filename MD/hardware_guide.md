# 実機入力

MID-360は[専用手順](../hardware/mid360/README.md)、制御元の切替は[共通関節制御](../gng_vlut_system/docs/joint_control.md)。

## RealSense

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true
```

画素対応が必要な場合はorganized点群を使用。RGBへの整列と未着色点の保持は用途に応じてdriver設定を選択。
取付TFは[realsense_mount.yaml](../gng_vlut_system/config/realsense_mount.yaml)。

```bash
ros2 launch gng_vlut_system realsense_mount_tf.launch.py
```

GNGは入力TFで座標変換。`input.local_coordinates: false` と `input.base_frame_id` を設定。
録画用の固定校正は[realsense_calibration.yaml](../ais_gng_cpu/src/ais_gng/config/realsense_calibration.yaml)。既存TFのあるセンサへの重複配信は禁止。
Viewerコピーは`pos`・`rot_deg`をそのまま使用（`scl`は1）。`camera_mount_pose`はROS標準RPY [rad]で、併用不可。
取付TF・補正保存は共通校正管理ノード。設定は[校正ストア](../gng_vlut_system/config/sensor_calibration.yaml)。MID-360との同時使用は機体launchの`enable_realsense_mount_tf`へ集約し、単独取付launchとの併用は不可。
変換済み点群の配信が必要な場合だけ `gng_vlut_system/pointcloud_tf_transformer_node` を使用。
bag再生は[play_bag.sh](../scripts/play_bag.sh)。

## Dynamixel

ドライバ起動は[handler](../hardware/dynamixel/dynamixel_handler/ReadMe.md)。
実測姿勢の表示は[読取り専用接続](../gng_vlut_system/docs/releases/2026-10-01_dynamixel_ids_31_52.md#実測現在姿勢の読取り専用表示)。
対応表は[bridge設定](../hardware/dynamixel/dynamixel_joint_state_bridge/config/)。実測の表示と目標指令を混同せず、同じ変換ノードの重複起動を避けること。
