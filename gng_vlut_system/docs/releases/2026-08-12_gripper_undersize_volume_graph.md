# 2026-08-12 - Gripper undersize volume graph

## Summary

ToPoDualArmの左右グリッパーについて、全閉時にも占有されない領域を
`TopologicalMap`として配信する処理を追加した。

## Added

- 左右TCPに固定された`undersize`体積graph。
- rosbag bundle exporterの任意記録topic。

## Behavior Impact

`gng_viewer_bridge.launch.py`を`ToPoDualArm.yaml`で起動すると、既存の最大把持領域2topicに加えて
`undersize`領域2topicも同じ汎用`gripper_volume_graph_node`から1回ずつpublishされる。
各publisherは従来どおりtransient-local QoSを使う。

`undersize`領域は最大把持領域全体を母領域とし、全閉時の左右指とグリッパ基部のSTL占有格子を
除外した差分graphである。固定の最小幅boxや点数閾値ではなく、閉じ切ってもグリッパが占有しない
空間を形状として保持する。

## Topics / Params / Messages

- `/ToPoDualArm/L_gripper_volume_undersize_topological_map` (`ToPoDualArm/L_tcp`)
- `/ToPoDualArm/R_gripper_volume_undersize_topological_map` (`ToPoDualArm/R_tcp`)
- message: `ais_gng_msgs/msg/TopologicalMap`
- source dimensions: `[0.061, 0.074, 0.0883] m`
- center: `[0.0, 0.0, 0.04415] m`
- resolution: `0.01 m`
- exclusion clearance: `0.005 m`
- exclusion meshes: 全閉時の左右指STLとグリッパ基部STL

## Verification

- 3設定ファイルを`yaml.safe_load`で読み込み、YAML構文を確認した。
- Docker内で`colcon build --packages-select grasping_system --symlink-install`に成功した。
- `ctest --test-dir build/grasping_system --output-on-failure`: 1/1成功。
- 隔離ROS domainで指定の`gng_viewer_bridge.launch.py params_file:=.../ToPoDualArm.yaml`を起動し、
  最大把持領域2topicと`undersize`領域2topicの計4topicを確認した。
- 最大把持領域が各504 nodes、1321 edgesのまま維持され、左右`undersize` publisherが
  全閉時メッシュ3個を除外して各300 nodes、639 edgesを生成した。frameが
  `ToPoDualArm/L_tcp` / `ToPoDualArm/R_tcp`であることを確認した。
- 検証後、隔離launchとROS daemonが残っておらず、既存domain 25のlaunch、Viewer、rosbridge、
  コンテナ状態がテスト前から変わっていないことを確認した。

## Risk / Notes

- 現在は設定に記録した全閉時TCP相対変換を使用する。URDFの指原点やTCP固定変換を変更した場合は、
  `exclude_closed_meshes.position`も同時に更新する必要がある。
