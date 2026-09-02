# 2026-09-02 - 非平面成分IDのTopologicalNode統合

## Summary

平面クラスタ未所属nodeのGNG連結成分を、既存`TopologicalMap`のnode属性とコンパクトな
`/nonplane_components` topicで配信する。

## Changed

- CPU版`ais_gng`が平面クラスタ生成直後に非平面成分を抽出
- 各`TopologicalNode`へ`nonplane_component_id`を設定
- `std_msgs/UInt32MultiArray`の`/nonplane_components`へ成分所属を出力
- ToPo-FUZZY Viewerの既存GNG node詳細へ成分IDを表示
- ToPo-FUZZY Viewerが`/nonplane_components`を成分表示layerとして認識

## Removed

- `NonplaneComponent.msg`
- `NonplaneComponentArray.msg`
- `nonplane_component_node`
- `/nonplane_components/markers`

## Behavior Impact

同一IDのnode集合と`TopologicalMap.edges`から成分内edgeを、`PlaneClusterArray`との接続から平面anchor edgeを復元可能。

## Topics / Params / Messages

- 追加field: 既存fieldの末尾に`TopologicalNode.NONPLANE_COMPONENT_NONE`、`TopologicalNode.nonplane_component_id`
- `NONPLANE_COMPONENT_NONE` (`4294967295`): 平面所属nodeまたは最小node数未満の成分
- `/nonplane_components`: `std_msgs/UInt32MultiArray`、`[frame_number, component_num, component_id, node_num, node_index..., ...]`
- YAML parameter: `nonplane_component.direct_enabled`、`nonplane_component.min_component_nodes`、`nonplane_component.output_topic`

## Verification

- `colcon build --symlink-install`による`ais_gng_msgs`、`ais_gng`、`topo_fuzzy_viewer`のbuild完了
- `ctest --test-dir /ros2_ws/build/ais_gng --output-on-failure -R '^test_nonplane_component_extractor$'`の成功
- `ros2 interface show ais_gng_msgs/msg/TopologicalNode`で追加fieldを確認
- `ros2 topic echo --once /nonplane_components`で成分所属出力を確認
- `ros2 interface list`と`ros2 pkg executables ais_gng`で廃止message・nodeが非公開であることを確認

## Risk / Notes

`TopologicalNode`のmessage定義変更とViewerのtopic認識追加のため、依存packageと実行中nodeの再build・再起動が必要。
