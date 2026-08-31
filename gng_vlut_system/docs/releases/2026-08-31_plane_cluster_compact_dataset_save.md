# 2026-08-31 - PlaneCluster保存と名称統一

## 概要

物体GNGテンプレート保存に局所平面クラスタを追加し、ROS APIの正規名称を
`PlaneCluster`と`PlaneClusterArray`へ統一。

## 変更

- ROSメッセージ型を`PlaneCluster`、`PlaneClusterArray`、`PlaneResidualBlob`、`PlaneResidualBlobArray`へ変更
- 平面クラスタtopic既定値を`/topological_plane_clusters_incremental`へ変更
- launch引数とexporter parameterを`plane_clusters_topic`へ変更
- `object_gng_dataset_exporter_node`による一致frameの平面クラスタ保存
- 保存JSONの`gng.plane_clusters`への圧縮表現

## 保存形式

各`plane_clusters`要素は次の値を持つ。

- `id`、`idx`、`label`
- `centroid`、`normal`、`tangent_u`
- `covariance_ut`、`extent`、`spacing`、`planarity`、`residual_ratio`

`support_edges`、`boundary`、常に0の`area`、復元可能な`tangent_v`は保存しない。
support edgeはテンプレートのGNG edgeと`idx`集合から復元する。

## 互換性

旧`Planar*`メッセージ型と`/topological_planar_clusters_incremental`は廃止。
利用ノード、rosbag、launch引数は新しい名称へ更新が必要。
