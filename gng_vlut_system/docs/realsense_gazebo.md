# RealSense実点群を使うGazebo回避

RealSenseの実時間PointCloud2を受信し、指定した仮想カメラ配置でGazeboの基準座標へ変換。GNG/VLUTと点群距離で仮想ロボットを継続回避。実機への指令出力なし。

```text
RealSense実点群 → カメラ内部TF → 仮想カメラ配置 → 新鮮さ検査・Gazebo時刻付与
  → ROIボクセル → 実機姿勢による自己除去 → VLUT/GNG → 継続回避 → Gazeboの関節指令
```

## 起動

RealSenseの既存launchはそのまま利用。両方の端末で同じROS_DOMAIN_ID・ROS_LOCALHOST_ONLYが必要。このPCの既存Viewer・RealSenseは25 / 0。

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true pointcloud.enable:=true
```

Gazebo側（同じGazeboポート・名前空間の旧デモは終了後に起動）:

```bash
docker exec -it gng_cpu_container bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 ROS2CLI_NO_DAEMON=1
ros2 launch gng_vlut_system pointcloud_avoidance.launch.py \
  input_config:=/ros2_ws/src/gng_vlut_system/config/realsense_gazebo_input.yaml \
  camera_pose:='[0.08, 0.0, 0.55, 0.0, 0.0, 0.0]' \
  gui:=true enable_viewer:=true
```

上記camera_poseは表示確認用の仮想配置例。実機の校正値ではない。配列はGazeboロボットのルート基準の`camera_link`位置xyz [m]・姿勢roll/pitch/yaw [rad]。光学frameの姿勢を直接指定するパラメータではない。実機の頭部姿勢と一致させたい場合は、実測関節角と取付外部パラメータから求めた値が必要。

自己除去には`/ToPoDualArm/joint_states`の全独立関節の有効な姿勢が必須。入力YAMLの`real_joint_topic`で変更可能。腰は固定0°というユーザー確認に基づき、新ID関節対応表の`fixed_joint_names`・`fixed_joint_positions`で明示。腕・首・グリッパーの新鮮な実測と一緒に配信。Viewerの初期値・Gazeboの関節値を未確認関節の実測代用にしない。各軸の符号・ゼロ点・URDF範囲の校正は別途必要。

起動後は保持。Viewerで点群配置を確認してから、操作端末のAで回避開始。Aで保持、Spaceで停止、Ctrl+Cで終了。停止後のLはラッチ解除後保持。実点群モードでは決まった時間でのデモ完了なし。障害物が離れれば開始姿勢へ復帰し、観測を継続。

別機体は`robot_config:=...`を追加。[共通機体設定](pointcloud_avoidance.md)と同じ条件。

## 表示と位置合わせ

### 実機頭部へのRealSense取付TF

URDFのカメラと実RealSenseの位置・向きが同じというユーザー確認に基づき、取付補正ゼロの設定を`config/realsense_mount.yaml`へ追加。`ToPoDualArm.yaml`は`enable_realsense_mount_tf: true`のため、通常のViewer起動だけで取付TFも同時起動。

```bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

既存のViewerには再起動時から反映。単独の`realsense_mount_tf.launch.py`を起動中の場合は先に終了。同時起動の無効化は`enable_realsense_mount_tf:=false`、設定ファイルの変更は`realsense_mount_config:=...`。機体YAMLで有効化していない他機体では既定OFF。

ToPoDualArmでは`enable_dynamixel_current_pose: true`により、Dynamixel生角度から実測関節値・Viewer関節値への変換も同時起動。既存の`/dynamixel/state/present`を購読し、Viewer起動によるUSBオープン・実機指令送信なし。RealSenseドライバとDynamixel通信ノードは別起動。独立した関節変換ノードを重複起動しない構成。GUI指令による従来の仮想姿勢表示へ戻す場合は`enable_dynamixel_current_pose:=false`を指定。

接続は`ToPoDualArm/camera_link → camera_link → camera_depth_optical_frame`。新規配信は最初の固定TFのみ。ロボットの首・腰による動的変換とRealSense内部の光学変換を利用し、光学回転の二重適用を回避。

取付位置の微調整は設定YAMLの`camera_mount_pose`を変更。xyzは親カメラリンク基準のメートル、rpyはラジアン。取付TF単独のlaunchでは`camera_mount_pose:='[x, y, z, roll, pitch, yaw]'`・`parent_frame`・`child_frame`・`mount_config`でも上書き可能。同じ子frameに対する取付TFの重複起動は避け、変更時は取付TFノードを終了後に再起動。

この接続はViewerの実機点群用。実測直接表示では首ピッチをURDF上限へ丸めず反映、腰は確認済みの固定0°。各軸のゼロ点・方向の校正は別途必要で、TF接続のみで実位置の正確さを保証するものではない。Gazeboの仮想配置用`camera_pose`とは別の設定。取付補正を変更した場合は`realsense_gazebo_input.yaml`の`camera_mount_pose`も同じ値へ更新。

### 通常ViewerのROI表示

`ToPoDualArm.yaml`の`enable_environment_voxelization: true`により、通常Viewer起動時にROI生成とVLUT入力変換も起動。入力は`environment_voxelization.input_topic`のRealSense点群、出力は`/ToPoDualArm/roi_voxels`。自己除去後の`self_filter_roi_voxels`を`voxel_to_vlut_node`で`occupied_voxels`・`danger_voxels`へ変換し、`topofuzzy_bridge_node`のGNG状態更新へ接続。

`Tmap_static`のstaticは学習済みのノード位置を維持する意味。点群に応じて安全・危険・衝突ラベルを更新。`danger_source: environment_inflation`では設定された余白の危険ボクセルを生成し、`vlut_distance`では環境側の膨張を0として既存VLUT距離判定を利用。

点群はTFで`ToPoDualArm/base_link`へ変換し、`Tmap_static`の範囲と機体YAMLの余白でROI抽出。セルサイズは読込み済みVLUT・自己マスクと共通。worldへの仮の接続・未接続点群の座標読み替え・world bucket索引は利用しない。TF欠落時は配信抑止、ROI外の点だけなら空の結果。自己認識・自己除去を無効にしたROI起動は拒否。

ROI生成だけを別ノードへ任せる場合は`enable_environment_voxelization:=false`。この表示経路の起動自体はGazeboの回避開始を含まない。

### Gazebo側の表示

- Viewer機体: `sim_ToPoDualArm`。
- 回避に使う配置済み点群: `/sim_ToPoDualArm/external_points`。Viewerのトピック一覧から表示。元の`/camera/camera/depth/color/points`とは座標・時刻が異なる別トピック。
- 自己除去前ROI: `/sim_ToPoDualArm/roi_voxels`。実測関節がなくても点群とclockが有効なら配信。
- 実機自己形状: `/sim_ToPoDualArm/self_voxel`。`sim_ToPoDualArm/real`名前空間の自己認識ノードによる配信。
- 自己除去後・回避用ボクセル: `/sim_ToPoDualArm/self_filter_roi_voxels`。VLUT・回避は常にこのトピックを使用。自己除去なしの迂回経路なし。
- 入力診断: `/sim_ToPoDualArm/external_cloud/status`。受理数・拒否数・最終出力からの時間・拒否理由・仮想カメラ配置、`has_fresh_real_state`・`real_state_detail`による実機関節の不足・失効表示。
- 経路・状態: `/sim_ToPoDualArm/plan_Tmap`、`avoidance/status`、`avoidance/gng_status`。
- Gazebo GUI: 仮想ロボットの物理追従表示。実点群をGazeboの衝突物体へ変換する処理なし。障害物との距離は回避ノードでのボクセル・外接球判定。点群の重ね合わせ表示はViewer。

点群と仮想ロボットの距離・向きをViewerで確認し、配置変更時はAで保持後にデモを終了してcamera_poseを変更。実カメラの移動・首腰の移動があった場合も再配置が必要。既存の実機TFや元点群のframeを書き換える操作なし。

## 入力・停止条件

- source: `/camera/camera/depth/color/points`、frame: `camera_depth_optical_frame`。変更は入力YAMLの`pipeline.external_cloud`。
- カメラ内部TF: RealSenseドライバの`camera_link ← camera_depth_optical_frame`を使用。worldへの取付TFを捏造せず、出力XYZだけを仮想空間へ変換。
- 時刻: 実時間stampの新規点群を受信したときだけGazebo時刻を付与。既定入力期限1秒。重複stamp・古いstamp・未来時刻・clock停止・frame違い・空点群は出力なし。最新点群の繰り返し再配信なし。
- 出力: 有限XYZのfloat32、最大10 Hz。RGB・法線などの付加フィールドは出力対象外。
- 自己除去: 常設。実測全関節からのFKと`robot_camera_link`・`camera_mount_pose`から実機ルートを仮想空間へ配置し、実点群と同じ配置で自己マスクを生成。Gazeboで動く腕とは別の自己形状。
- 取付補正: `camera_mount_pose`はURDFの`robot_camera_link`からRealSense本体`camera_frame`へのxyz [m]・rpy [rad]。既定ゼロは両座標系が一致する取付けの場合だけ妥当。
- 実測失効: 欠損関節・古い／重複stamp・URDF範囲外の値は採用なし。実測姿勢の0.5秒失効でマスク更新を抑止。マスク失効後は自己除去後更新を抑止し、回避入力期限で停止。TF取得失敗時の単位変換への代替なし。
- 初期重なり・距離不足: 開始拒否または停止。点群を消したり停止余裕を緩めたりして回避開始する処理なし。
- 適用範囲: ToPoDualArm左腕7関節の学習済みデータ。模擬LiDAR・模擬前腕は生成なし。未知・遮蔽領域の完全な衝突保証なし。

## 試験コマンド

実時間stamp付きの試験点群による回避・復帰・欠測停止（コンテナ内）。実カメラを操作する試験とは別。

```bash
ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 \
  python3 /ros2_ws/src/gng_vlut_system/test/check_external_pointcloud.py \
  --output /ros2_ws/src/artifacts/realsense_gazebo_trial
```

入力変換の実カメラ試験とGazebo試験の結果は[リリースノート](releases/2026-10-01_realsense_gazebo.md)を参照。
