# URDF → GNG / VLUT 構築ガイド

URDFのロボットモデルからGNGとVLUTを構築するまでの手順。

## コンテナの起動

```bash
cd ~/arm_avoidance
docker compose down && docker compose up --build -d
docker compose exec gng_cpu bash
```

## ビルド

```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install \
  --packages-select voxel_msgs voxel_idx gng_control_msgs gng_vlut_system
source install/setup.bash
```

## GNGとVLUTのオフライン構築

URDFから両腕分のGNGを学習。`gng_profile_names`で対象アームを指定。

```bash
ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  use_voxel_collision:=true \
  gng_profile_names:=left_arm
```

- `initial_collision_only:=true`: 初期姿勢での衝突リンクの組み合わせだけを検証。

片腕・単一プロファイルの場合。

```bash
ros2 launch gng_vlut_system offline_urdf_trainer.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

`ros2 run`で直接実行する場合は
[gng_vlut_system/README.md](gng_vlut_system/README.md) を参照。

## GNGの後処理

```bash
# 非activeノードを削除
ros2 launch gng_vlut_system offline_gng_main5_clean.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

# 分離した島を除去し最大連結成分のみ保持
ros2 launch gng_vlut_system offline_gng_main6_island_pruning.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

# 可操作性などの特徴量を再計算
ros2 launch gng_vlut_system offline_gng_status_updater.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

## 実行時のURDF→VLUT（自己認識ボクセル）

```bash
ros2 launch gng_vlut_system self_recognition_viz.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  root_link:=right_link2 \
  leaf_link:=right_link8 \
  mask_topic:=/ToPoDualArm/right_arm_voxel
```

### 自己認識ボクセルを occupied_voxels / danger_voxels へ橋渡し

```bash
ros2 launch gng_vlut_system voxel_to_vlut.launch.py \
  robot_name:=ToPoDualArm \
  input_topic:=/ToPoDualArm/right_arm_voxel \
  danger_inflation:=0.05
  # (output_voxel_size:=0.02)
```

## URDF準拠のダミー関節状態

```bash
ros2 launch gng_vlut_system dummy_joint_pub.launch.py \
  urdf_path:=/ros2_ws/src/<robot_package>/<robot>.urdf
```

## ロボットを座標変換

```bash
python3 test_tf_publisher.py --world-frame world \
  --frame-id ToPoDualArm/base_link \
  --x 0.35 --y 0.15 --z -0.3 --yaw 3.2
```

## 常駐プロセスの停止

```bash
./scripts/stop_ros2_stack.sh
```
