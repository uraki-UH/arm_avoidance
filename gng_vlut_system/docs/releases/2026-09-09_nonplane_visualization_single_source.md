# 2026-09-09 - Single Nonplane Visualization Source

## Summary

CPU版ais_gngの非平面成分表示を `/nonplane_components` に統一。
Viewerは所属情報とTopologicalMapから直接描画するため、重複するMarkerArray生成を省略。

## Changed

`ais_gng.launch.py` が `enable_nonplane_markers` をCPU直結成分抽出の有効状態から設定。
無効なマーカートピックについての起動ログも省略。

## Added

なし。

## Fixed

同じ非平面連結成分について、所属情報と再抽出MarkerArrayの2経路が同時に公開される問題。

## Removed

CPU直結成分抽出が有効な標準起動での `/nonplane_components/markers` publisher。
マーカー生成機能自体は維持。

## Behavior Impact

- CPU直結が有効: `/nonplane_components` をViewerで選択。
- CPU直結が無効、またはGPU版: 従来の非平面MarkerArray経路を使用。
- 単独の `plane_cluster_incremental.launch.py` の既定動作は変更なし。
- 平面可視化は変更なし。上位曲面モデルの表示形式は別変更 `2026-09-09_curved_surface_graph.md` を参照。
- 実行中ノードの動的切替は未実装。次回launch起動時に適用。

## Topics / Params / Messages

所属情報はstd_msgs/UInt32MultiArray、マーカーはvisualization_msgs/MarkerArrayのまま。
既存の `nonplane_component.direct_enabled` と `enable_nonplane_markers` を利用。新規パラメータなし。

## Verification

Releaseビルドに成功。ROS_DOMAIN_ID=197/198で下記launchをそれぞれ起動し、初期化完了とpublisher数を検証。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml \
  plane_params_file:=<テスト用YAML>
```

直結有効時の `[所属情報, 非平面マーカー, 平面ノードマーカー]` publisher数は `[1,0,1]`。
直結無効時は `[0,1,1]`。検証用launchと全子ノードはSIGINT後の正常終了を確認。
既存GNG・Viewer・bag再生の停止操作なし。

## Risk / Notes

`/nonplane_components` は非平面連結成分であり、平面パッチも含む上位SurfaceModelとは別の情報。
両者の表示統合は今回の変更には含まない。
