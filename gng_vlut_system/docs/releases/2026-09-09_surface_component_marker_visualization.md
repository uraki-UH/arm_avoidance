# 2026-09-09 - 曲面候補ノード・エッジの可視化

## Summary

平面クラスタ未所属のGNGノード連結成分を曲面候補として、TopoFuzzy Viewerで確認できるMarkerArray出力を追加した。

## Added

- `/nonplane_components/markers` (`visualization_msgs/msg/MarkerArray`)
- 曲面候補ノードのPOINTSマーカー
- 曲面候補内部のGNGエッジ
- 平面クラスタへ接続するアンカーエッジ
- 前フレームから消えた成分の明示的なDELETE

## Topics / Params / Messages

- 入力: `/topological_map`, `/plane_clusters`
- 出力: `/nonplane_components/markers`
- `surface_marker_topic`（既定: `/nonplane_components/markers`）
- `surface_component.min_nodes`（既定: `2`）
- 既存の平面クラスタtopic、GNG本体、メッセージ定義は変更しない

## Verification

- Docker内で`ais_gng`をReleaseビルド
- 実行中の`/topological_map`と`/plane_clusters`へ一時可視化ノードを接続
- `/nonplane_components/markers`が`visualization_msgs/msg/MarkerArray`として広告され、`surface_component_nodes`、`surface_component_edges`、`surface_component_anchors`を含むことを確認
- 検証後、一時ノードを停止。既存のGNG・Viewerプロセスは維持

## Risk / Notes

現段階では曲面のメッシュやQuadric面そのものではなく、GNG構造から得た曲面候補のノード・エッジを可視化する。Quadric係数のオンライン推定は点群入力との同期が必要なため、別途統合する。
