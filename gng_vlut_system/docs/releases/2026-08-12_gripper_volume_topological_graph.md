# 2026-08-12 - Gripper volume topological graph

## Summary

グリッパーの把持可能体積を、既存の
`ais_gng_msgs/msg/TopologicalMap`として生成・配信する経路を
`grasping_system`へ追加した。専用messageは追加しない。

## Added

- `box`、`ellipsoid`、`cylinder`の体積を3次元格子graphへ変換するbuilder。
- graphを`TopologicalMap`へ変換するadapter。
- transient-local publisher `gripper_volume_graph_node`。
- 任意数のグリッパー定義と対応TFを受け取る汎用launch。
- `gng_viewer_bridge.launch.py`から同じ`params_file`設定に従って汎用launchをincludeする統合経路。
- ToPoDualArmのURDF形状・prismatic上限から求めた最大把持体積設定。

## Removed

- ロボット名を固定した`ToPoDualArm_gripper_volume_graph.launch.py`。同じ設定は
  `gripper_volume_graph.launch.py grippers_file:=...`で起動する。

## Behavior Impact

- `nodes`は体積内部のサンプル中心、`edges`は6近傍接続を表す。
- `clusters[0]`は体積中心、姿勢、XYZ外形寸法、所属node IDを保持する。
- graphはTCP frameに固定され、ロボット姿勢との合成は既存TFを使う。
- `gng_viewer_bridge.launch.py`経由では`robot_name`をTF prefixとして付与するため、
  設定ファイルにはURDF由来の非prefix frame名を記述する。
- Viewer gatewayは`/tf_static`をtransient-local QoSで購読・保持し、frontendは
  親子TFをルートまで合成してgraphを配置する。
- 起動時に1回だけpublishする。late subscriberへの配信はtransient-local QoSに任せ、
  application-levelの再送や周期タイマーは持たない。
- 非box形状でも、現在のViewer上のcluster表示は外接boxになる。形状差は
  `nodes`と`edges`に保持される。
- ToPoDualArm設定は最大開口時の把持可能領域であり、現在のgripper開度には追従しない。

## Topics / Params / Messages

Message typeは既存の`ais_gng_msgs/msg/TopologicalMap`を変更せず使用する。

| 項目 | 内容 |
|---|---|
| `output_topic` | 出力するgraph topic |
| `frame_id` | TCPまたはtool frame |
| `shape` | `box`、`ellipsoid`、`sphere`、`cylinder` |
| `dimensions` | 体積のXYZ外形寸法[m] |
| `center` | `frame_id`基準の体積中心[m] |
| `orientation_xyzw` | `frame_id`基準の体積姿勢 |
| `resolution` | graph格子間隔[m] |

`gripper_volume_graph.launch.py`は次のlaunch引数を持つ。

| launch引数 | 内容 |
|---|---|
| `grippers` | `name`、`tf_frame`、topic、形状をまとめたYAMLリスト |
| `grippers_file` | 同じリストを持つYAMLファイル。指定時は`grippers`より優先 |
| `tf_prefix` | 各`tf_frame`へ付与する任意のrobot prefix。同じprefixは重複付与しない |

`gng_viewer_bridge.launch.py`側の設定と上書き引数:

| 項目 | 内容 |
|---|---|
| `gripper_volume_graph.enabled` | `params_file`内の自動起動フラグ |
| `gripper_volume_graph.definitions_file` | `params_file`内のグリッパー定義ファイルURI |
| `enable_gripper_volume_graph` | launch引数で有効・無効を上書き |
| `gripper_volume_config_file` | launch引数で定義ファイルを上書き |
| `gripper_volume_grippers` | launch引数でinline YAMLリストを指定 |

ロボット固有launchは作らず、グリッパー構成だけを設定ファイルまたは
`grippers`引数として渡す。

ToPoDualArmの既定topic:

- `/ToPoDualArm/L_grip_V_topological_map` (`ToPoDualArm/L_tcp`)
- `/ToPoDualArm/R_grip_V_topological_map` (`ToPoDualArm/R_tcp`)

## Verification

- `colcon build --packages-select grasping_system --symlink-install`: 成功。
- `ctest --test-dir build/grasping_system --output-on-failure`: 1/1成功。
- 汎用launchへToPoDualArm設定を渡し、各graphで504 nodes、1321 edgesを生成。
- `grippers`直接引数から`test_gripper` / `custom_tcp`のellipsoid graphを生成。
- 先行起動した別ROS 2 subscriberで左topicを受信し、TCP frameを確認。
- publisher起動後に左右subscriberを追加し、transient-localに保持されたTCP frameを確認。
- Docker内で`gng_viewer_bridge.launch.py params_file:=.../ToPoDualArm.yaml`の子として
  左右publisherが起動し、両topicから`ToPoDualArm/L_tcp` / `ToPoDualArm/R_tcp`を受信。
- 同じ隔離ROS domainで`base_footprint`から左右TCPまでのTFを取得し、graph frameとの
  完全一致を確認。
- `/tf_static`のTCP固定変換をgatewayが購読し、新規WebSocket接続の`stream.tf`にも
  左右TCPを再送することを確認。
- `grasping_system`と`topo_fuzzy_viewer`のDocker build、frontend production buildに成功。
- 隔離ROS domainで左右publisherのpublishログが起動時各1回だけであり、late subscriber
  接続後にもapplication-levelの追加publishが発生しないことを確認。
- 検証後、launch、左右node、topic echo、ROS daemonが残っていないことを確認。

## Risk / Notes

- `TopologicalMap`の`label`は文字列のハンド種別ではないため、グリッパー識別には
  topic名と`header.frame_id`を使う。
- `TopologicalMap`のnode/edge indexは`uint16`なので、細かすぎる`resolution`で
  65,535 nodesに達する場合は生成を拒否する。
- publisherはtransient-localなので、Viewerやrosbag recorderを後から開始しても
  DDSに保持された静的graphを取得できる。application-levelの再送は行わない。
