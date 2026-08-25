# グリッパ体積グラフキャッシュ

`gripper_volume_graph_node` は、メッシュから生成した静的 `TopologicalMap` を CDR 形式で保存できる。通常起動では STL ボクセル化を行わず、キャッシュを復元して publish する方式。

## ToPoDualArm の事前構築

Docker 内で次を一度実行する。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch grasping_system gripper_volume_graph.launch.py \
  grippers_file:=/ros2_ws/src/grasping_system/config/ToPoDualArm_gripper_volumes.yaml \
  tf_prefix:=ToPoDualArm \
  cache_directory:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gripper_volume_cache \
  cache_mode:=refresh \
  publish_graph:=false \
  exit_after_publish:=true
```

この実行は cache のみを生成し、`TopologicalMap` topic を publish しない。完了後は通常どおり次を起動する。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
```

`ToPoDualArm.yaml` は cache directory と `cache_mode: use` を既定値として持つ。

## 再構築条件

`cache_mode: use` では、グリッパ形状、解像度、メッシュ姿勢、TF frame、topic、ラベル、STL 内容を署名に含める。不一致・破損・未作成時は自動的に再構築して保存する。

形状更新を明示的に反映したい場合は、上記の事前構築コマンドを `cache_mode:=refresh` のまま再実行する。
