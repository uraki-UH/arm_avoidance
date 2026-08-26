# 2026-08-26 - TopoFuzzy incremental voxel stream

## Summary

TopoFuzzy Viewerへのvoxel配信を、初回snapshot後は追加・削除セルだけを送る方式へ変更した。
同時に、GNG bridgeは購読されていない全量graphの周期再構築を省略する。

## Changed

- Viewer Gatewayは`voxel_msgs/Voxel`の前回値をtopicごとに保持し、WebSocketへ
  `stream.voxel.delta`を配信する。
- Frontendは差分の`sequence`連続性を検証し、欠落時は`request.state`で全量再同期する。
- `topofuzzy_bridge_node`は起動時にtransient-local用の全graphを1回publishし、以後は
  subscriberが存在するgraphだけを再構築・publishする。

## Added

- WebSocket `stream.voxel.delta`
  - `added`: 追加またはlabel変更したvoxel ID文字列
  - `labels`: `added`と整列するlabel。labelなしstreamでは空配列
  - `removed`: 削除したvoxel ID文字列
  - `sequence`: Gatewayがtopicごとに割り当てる連続番号

## Behavior Impact

- ROS 2の`voxel_msgs/Voxel` topicは従来どおり完全snapshotであり、安全判定側の契約は変更なし。
- voxel layout、frame、label有無が変わった場合は、差分ではなく新しい全量snapshotへ戻る。
- `/ToPoDualArm/topological_map_vis_L0`だけをViewerで購読した場合、
  `/ToPoDualArm/topological_map_static`の約1 MB全量messageを更新ごとに構築しない。
- 新規graph subscriberは起動時のtransient-local snapshotを受信し、次の状態更新から最新値を受信する。

## Topics / Params / Messages

- ROS topic、ROS parameter、ROS messageの追加・変更なし。
- Viewer内部WebSocket messageとして`stream.voxel.delta`を追加。

## Verification

- `npm run build`: TypeScript buildおよびVite production build成功。
- `colcon build --symlink-install --packages-select topo_fuzzy_viewer gng_vlut_system`: 成功。
- `/topo_voxel_ids`を6秒間配信した実測:
  初回88,736 bytes、差分54件の平均8,071 bytes、1更新あたり約91%削減。

## Risk / Notes

- 差分受信後もFrontendは描画用配列を再構築する。ボクセル数自体が非常に多い場合は、
  次段階として`InstancedMesh`の追加・削除をGPU bufferへ直接反映する余地あり。
- 10,801ノードの`topological_map_static`表示より、150ノードの
  `topological_map_vis_L0`表示を推奨。
