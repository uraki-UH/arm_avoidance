# 2026-08-12 - Gripper undersize volume graph

## Summary

ToPoDualArmの左右グリッパーについて、全閉時の左右指に挟まれた内部の非占有領域を
`TopologicalMap`として配信する処理を追加した。

## Added

- 左右TCPに固定された`undersize`体積graph。
- rosbag bundle exporterの任意記録topic。

## Behavior Impact

`gng_viewer_bridge.launch.py`を`ToPoDualArm.yaml`で起動すると、既存の最大把持領域2topicに加えて
`undersize`領域2topicも同じ汎用`gripper_volume_graph_node`から1回ずつpublishされる。
各publisherは従来どおりtransient-local QoSを使う。

`undersize`領域は最大把持領域全体を探索格子とし、閉方向の正負rayが全閉時の対応する指STLへ
両方とも到達する格子だけを残す。さらに左右指とグリッパ基部のSTL占有格子を除外するため、
最大boxの外側自由空間ではなく、閉じた左右指に挟まれた内部の非占有空間だけを形状として保持する。
Viewerで元の最大boxが重ならないよう、このgraphの`TopologicalMap.clusters`は空とする。

## Topics / Params / Messages

- `/ToPoDualArm/L_grip_V_undersize_topological_map` (`ToPoDualArm/L_tcp`)
- `/ToPoDualArm/R_grip_V_undersize_topological_map` (`ToPoDualArm/R_tcp`)
- message: `ais_gng_msgs/msg/TopologicalMap`
- source dimensions: `[0.061, 0.074, 0.0883] m`
- center: `[0.0, 0.0, 0.04415] m`
- resolution: `0.01 m`
- exclusion clearance: `-0.005 m`（全閉メッシュ占有を内側へ縮める逆向きマージン）
- exclusion meshes: 全閉時の左右指STLとグリッパ基部STL
- closing axis: TCP座標の`+Y / -Y`
- internal side: 正側指 / 負側指を設定で明示
- cluster: 未生成（内部graphのnodes / edgesだけを配信）

## Verification

- 3設定ファイルを`yaml.safe_load`で読み込み、YAML構文を確認した。
- Docker内で`colcon build --packages-select grasping_system --symlink-install`に成功した。
- `ctest --test-dir build/grasping_system --output-on-failure`: 2/2成功。
- 隔離ROS domainで`gripper_volume_graph.launch.py grippers_file:=.../ToPoDualArm_gripper_volumes.yaml`
  を起動し、最大把持領域2topicと`undersize`領域2topicの計4topicを確認した。
- 指定の`gng_viewer_bridge.launch.py params_file:=.../ToPoDualArm.yaml`経由でも同じ生成数を確認した。
- 最大把持領域が各504 nodes、1321 edgesのまま維持され、左右`undersize` publisherが
  逆向き`0.005 m`マージンで各130 nodes、279 edges、0 clustersを生成した。変更前の正方向
  `0.005 m`では各22 nodes、39 edgesだった。拡張後も正負rayによる内部限定は有効である。
  frameは単体launchのため`L_tcp` / `R_tcp`となる。
- 検証後、隔離domain 129〜132のlaunch、node、ROS daemonが残っていないことを確認した。検証中に
  domain 25で別途起動された`gng_viewer_bridge.launch.py`と4個のvolume publisherは停止せず維持した。

## Risk / Notes

- 現在は設定に記録した全閉時TCP相対変換を使用する。URDFの指原点やTCP固定変換を変更した場合は、
  `exclude_closed_meshes.position`も同時に更新する必要がある。
- `exclusion_clearance`は符号付きで、正値は非占有領域を縮小し、負値は非占有領域を拡張する。
