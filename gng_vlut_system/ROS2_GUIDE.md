# ROS2 Guide

このドキュメントは、`gng_safety` を使った ROS 2 実行系のまとめです。

## 1. 何ができるか

- `joint_state_mux_node`: RViz 用と実機用の `JointState` を `/joint_states` に集約
- `robot_bridge_node`: 実機 UDP から `/joint_states_real` を生成
- `topoarm_joint_state_player`: RViz 用 UDP から `/joint_states_rviz` を生成
- `robot_description_player`: `/robot_description` を publish
- `self_recognition_viz_node`: 自己認識マスクを `/self_mask_viz` に表示
- `safety_monitor_node`: GNG/VLUT の安全状態を `/gng_viz` に表示
- `voxel_status_test_publisher`: 仮想ボクセルを流して色変化を確認

## 2. ビルド

```bash
cd ~/uraki_ws/gng_vlut_system
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gng_safety
source install/setup.bash
```

## 3. 基本起動

RViz 用の入力源を使う場合:

```bash
ros2 launch gng_safety visualize_topoarm_ros2.launch.py
```

実機の現在姿勢を使う場合:

```bash
ros2 launch gng_safety visualize_topoarm_ros2.launch.py \
  joint_state_source:=real
```

安全監視まで見る場合:

```bash
ros2 launch gng_safety visualize_topoarm_ros2.launch.py \
  joint_state_source:=rviz \
  enable_safety_monitor:=true \
  gng_model_path:=/path/to/topoarm_real_v1_phase2.bin \
  vlut_path:=/path/to/gng_spatial_correlation.bin
```

## 4. ボクセル色変化テスト

球状の占有・危険ボクセルを流す例:

```bash
ros2 run gng_safety voxel_status_test_publisher --ros-args \
  -p scenario:=sphere \
  -p sphere_center_world_cm:="[20.0, 20.0, 30.0]" \
  -p sphere_radius_cm:=10.0 \
  -p sphere_danger_margin_cm:=3.0 \
  -p voxel_size:=0.02
```

## 5. トピック

- `/joint_states`
- `/joint_states_rviz`
- `/joint_states_real`
- `/robot_description`
- `/self_mask_viz`
- `/gng_viz`
- `/occupied_voxels`
- `/danger_voxels`

## 6. UDP のメモ

- `topoarm_joint_state_player` は RViz 用の簡易 UDP 入力で `/joint_states_rviz` を出す
- `robot_bridge_node` は実機向け UDP のブリッジで `/joint_states_real` を出す
- `joint_state_mux_node` が選択中のソースを `/joint_states` にまとめる
- `joint_state_source:=rviz` なら RViz 用入力、`joint_state_source:=real` なら実機姿勢を使う
