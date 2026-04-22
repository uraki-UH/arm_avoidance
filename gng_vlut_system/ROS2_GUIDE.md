# ROS2 Guide

このドキュメントは、`gng_safety` を使った ROS 2 実行系のまとめです。

## 1. 何ができるか

- `joint_state_mux_node`: RViz 用と実機用の `JointState` を `/joint_states` に集約
- `robot_bridge_node`: 実機 UDP から `/joint_states_real` を生成
- `topoarm_joint_state_player`: RViz 用 UDP から `/joint_states_rviz` を生成
- `robot_description_player`: `/robot_description` を publish
- `self_recognition_viz_node`: 自己認識マスクを `/self_mask_viz` に表示
- `safety_monitor_node`: GNG/VLUT の安全状態を `/gng_viz` に表示
- `topofuzzy_bridge_node`: `/topological_map` を Viewer 互換の形式で publish
- `robot_arm_bridge_node`: `/joint_states` を使ってアーム姿勢を Viewer 向けに publish
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
ros2 launch gng_safety visualize_topoarm_rviz.launch.py
```

トピック送信だけを起動する場合:

```bash
ros2 launch gng_safety topoarm_runtime.launch.py
```

Gazebo でも同じ URDF を召喚する場合:

```bash
ros2 launch gng_safety spawn_topoarm_gazebo.launch.py
```

Gazebo の GUI を出す場合:

```bash
ros2 launch gng_safety spawn_topoarm_gazebo.launch.py gazebo_gui:=true
```

実機の現在姿勢を使う場合:

```bash
ros2 launch gng_safety visualize_topoarm_rviz.launch.py \
  joint_state_source:=real
```

安全監視まで見る場合:

```bash
ros2 launch gng_safety visualize_topoarm_rviz.launch.py \
  enable_safety_monitor:=true \
  gng_results_config_path:=gng_results/config.txt
```

ToPoFuzzy-Viewer へ流す場合:

```bash
ros2 launch gng_safety topofuzzy_bridge.launch.py
```

アーム姿勢を Viewer へ流す場合:

```bash
ros2 launch gng_safety topoarm_viewer_bridge.launch.py
```

この launch は `topoarm_runtime.launch.py` を内部で含むので、`/joint_states` の準備も一緒に行う。

## 4. ボクセル色変化テスト

球状の占有・危険ボクセルを流す例:

```bash
ros2 run gng_safety voxel_status_test_publisher --ros-args \
  -p scenario:=sphere \
  -p sphere_center_world_cm:="[0.0, 0.0, 30.0]" \
  -p sphere_orbit_radius_cm:=25.0 \
  -p sphere_orbit_period_s:=20.0 \
  -p sphere_radius_cm:=10.0 \
  -p sphere_danger_margin_cm:=3.0 \
  -p scenario_period_s:=0.1 \
  -p voxel_size:=0.02
```

色の意味:

- `safe` = 緑
- `danger` = 黄
- `collision` = 赤

## 5. トピック

- `/joint_states`
- `/joint_states_rviz`
- `/joint_states_real`
- `/robot_description`
- `/self_mask_viz`
- `/gng_viz`
- `/topological_map`
- `/viewer/internal/stream/robot_arm`
- `/occupied_voxels`
- `/danger_voxels`

## 6. UDP のメモ

- `topoarm_joint_state_player` は RViz 用の簡易 UDP 入力で `/joint_states_rviz` を出す
- `robot_bridge_node` は実機向け UDP のブリッジで `/joint_states_real` を出す
- `joint_state_mux_node` が選択中のソースを `/joint_states` にまとめる
- `joint_state_source:=rviz` なら RViz 用入力、`joint_state_source:=real` なら実機姿勢を使う

## 7. 起動分離のメモ

- `topoarm_runtime.launch.py` は `robot_state_publisher`、`robot_description_player`、`joint_state_mux_node`、`self_recognition_viz_node`、`safety_monitor_node` をまとめる
- `visualize_topoarm_rviz.launch.py` は `topoarm_runtime.launch.py` を呼び出したうえで RViz だけを起動する
- GNG の色付き marker は `safety_monitor_node` が `/gng_viz` に出す。通常は `gng_results/config.txt` が優先され、必要なときだけ `gng_model_path` / `vlut_path` を上書きする
- `topofuzzy_bridge.launch.py` は `/occupied_voxels` と `/danger_voxels` を受けて `/topological_map` を publish する。Viewer 側は `viewer_stack.launch.py` を起動しておけばそのまま受け取れる
- `topoarm_viewer_bridge.launch.py` は `/joint_states` を受けてアーム姿勢を `/viewer/internal/stream/robot_arm` に publish する。Viewer 側はそのまま 3D アームとして表示する
- `voxel_status_test_publisher` の `sphere` シナリオは `scenario_period_s` で更新周期を決める。`sphere_orbit_radius_cm` と `sphere_orbit_period_s` を入れると、中心が XY 平面で原点まわりを回る
- `spawn_topoarm_gazebo.launch.py` は `gzserver` と必要なら `gzclient` を起動し、`temp_robot.urdf` を直接 spawn する
- 既定では GUI は起動しない。`gazebo_gui:=true` で ROS plugin なしの素の `gzclient` を出し、Qt は `xcb` を使う
- `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` で初期配置を調整できる
- 既定では `spawn_z:=0.02` なので、床との干渉が気になる場合は少し上げるとよい
