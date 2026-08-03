# 2026-08-03 - visualization-only 3D GNG

## Summary

姿勢GNGのcoord layerを3次元サンプルとして再学習する、可視化専用GNGを追加した。
可視化ノードは関節角を持たず、対応する元姿勢GNGノードID群を保持する。

## Added

- `visualization_gng_trainer`: 元`gng.bin`からlayer別の可視化GNG binを事前生成する。
- `VisualizationGngModel`: 3次元ノード、エッジ、元ノードID群、照合signatureを保存する。
- `topofuzzy_bridge_node`: 可視化GNGを`TopologicalMap`としてlayer別に配信する。
- 元ノード状態のbest-wins集約と、障害物更新時の再配信。

## Behavior Impact

既存の姿勢GNG、VLUT、`topological_map_static`、node feature配信は変更しない。
`ToPoDualArm.yaml`では可視化GNG配信を有効にした。binが存在しない場合や元GNGと
signatureが一致しない場合は、その可視化layerだけを警告付きでスキップする。

可視化binには`weight_angle`を複製しない。元ノードIDを介して元GNGの姿勢と状態を参照する。

## Topics / Params / Files

- Topic: `/ToPoDualArm/topological_map_visualization_layer_0`
- Message: `ais_gng_msgs/msg/TopologicalMap`
- Params: `visualization_gng.enabled`, `visualization_gng.path_prefix`, `visualization_gng.topic_prefix`
- Generated file: `gng_results/ToPoDualArm3/visualization_gng_layer_0.bin`

## Measured Result

- Source active nodes: 941
- Visualization nodes: 500
- Visualization edges: 840
- Empty visualization memberships: 0
- Generated bin: 18,516 bytes
- Default training time: 約0.22秒(CPU、現環境)

## Verification

- `visualization_gng_trainer`で保存後に再読込し、layer、signature、元ノード総数を照合。
- 同一入力とseedで再生成したbinが`cmp`で完全一致。
- `visualization_gng_trainer`と`topofuzzy_bridge_node`の個別CMake build成功。
- 新規の一時build/installで`gng_control_msgs`生成後、`gng_vlut_system`全体build成功。
- 稼働中launchを維持したままHumbleの`gng_cpu_container_uraki`内で全体build成功。
- ROS_DOMAIN_IDを分離した実配信で500ノード、840エッジを受信。
- テスト後に既存コンテナ、ROS launch、ROS daemonが開始前と同じ状態であることを確認。
- `git diff --check`。

## Risk / Notes

- bin version 1はnative binary形式で、現在のx86_64 little-endian環境を前提とする。
- build前から起動中の`topofuzzy_bridge_node`は旧プロセスのまま維持したため、通常topicへの
  反映には`gng_viewer_bridge.launch.py`の次回起動が必要。
