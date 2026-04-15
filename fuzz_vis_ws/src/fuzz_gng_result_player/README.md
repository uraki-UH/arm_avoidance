# fuzz_gng_result_player

`offline_urdf_trainer` で保存した GNG の結果を読み直して、`TopologicalMap` として ROS 2 に流すための小さい補助パッケージです。

## 何をするか

- `gng_results/<experiment_id>/<experiment_id>_phase2.bin` を定期監視します
- 互換のため `gng_results/<experiment_id>_phase2.bin` の直下配置も自動で探します
- ファイルが更新されたら再読み込みします
- ノードの `status` 相当を `TopologicalMap` の `label` に反映します
- `ais_gng` の `visualizar` と `rviz2` で表示できます
- `visualize_result_with_arm.launch.py` では UDP の関節角を `robot_state_publisher` に流して、TopoArm も動かします
- `/marker` は transient local で配信するので、RViz を後から立ち上げても受け取りやすくしています
- `republish_ms` で同じ `MarkerArray` を定期再送するので、RViz 側で一度オフにしても戻しやすいです
- `normal` と `edges` は RViz の初期表示でオフにしています

## ビルド

```bash
cd ~/uraki_ws/fuzz_vis_ws
source /opt/ros/humble/setup.bash
source ~/uraki_ws/ais_gng_cpu/install/setup.bash
colcon build --symlink-install --packages-select fuzz_gng_result_player
source install/setup.bash
```

## 起動

```bash
ros2 launch fuzz_gng_result_player visualize_result.launch.py \
  experiment_id:=topoarm_real_v1 \
  data_directory:=/home/fuzzrobo/uraki_ws/fuzz_arm/gng_results \
  phase2_suffix:=_phase2
```

`offline_urdf_trainer` の結果が `~/uraki_ws/fuzz_arm/gng_results/` にあるなら、以下でそのまま読めます。

```bash
ros2 launch fuzz_gng_result_player visualize_result.launch.py \
  experiment_id:=topoarm_real_v1 \
  data_directory:=/home/fuzzrobo/uraki_ws/fuzz_arm/gng_results \
  phase2_suffix:=_phase2
```

## アームも一緒に見る

```bash
ros2 launch fuzz_gng_result_player visualize_result_with_arm.launch.py \
  experiment_id:=topoarm_real_v1 \
  data_directory:=/home/fuzzrobo/uraki_ws/fuzz_arm/gng_results \
  frame_id:=world
```

この launch は次も使います。

- `/robot_description` に URDF を流す `robot_description_player`
- UDP `12345` を受ける `udp_joint_state_player`
- `robot_state_publisher`

`temp_robot.urdf` や `topoarm_description` を使うので、必要なら `fuzz_arm` 側の環境も source してください。

この launch は `ais_gng` の `visualizar` と `rviz2` も使います。  
`ais_gng` が見つかれば自動で一緒に起動し、見つからない場合は `gng_result_player` だけ起動します。  
`ais_gng` 側のビルド済み環境が別にあると、そのまま流用できます。

## 更新間隔を短くする

色の更新をより細かく追いたい場合は `poll_ms` を短くします。

```bash
ros2 launch fuzz_gng_result_player visualize_result.launch.py poll_ms:=100
```

## 設定ファイルを使う

`config_file` に `key=value` 形式のファイルを渡すと、以下のキーを読めます。

- `experiment_id`
- `data_directory`
- `phase2_output_suffix`
- `online_input_suffix`

例:

```bash
ros2 launch fuzz_gng_result_player visualize_result.launch.py \
  config_file:=/path/to/config.txt
```

## 補足

- `visualize_result.launch.py` は `ais_gng/rviz.launch.py` を内部で呼びます
- `TopologicalMap` の色分けは `visualizar` 側の既存ロジックをそのまま使います
- このパッケージは `offline_urdf_trainer` とは別機能なので、分離しています
