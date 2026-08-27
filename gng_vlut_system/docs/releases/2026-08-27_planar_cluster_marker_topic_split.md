# 2026-08-27 - Planar cluster marker topic split

## Summary

増分平面クラスタの凸包と法線矢印を、個別に表示選択できるMarkerArray topicへ分離した。

## Changed

- 凸包と任意のクラスタラベルを`/topological_planar_clusters_incremental/markers/hull`へ配信する。
- 法線矢印を`/topological_planar_clusters_incremental/markers/normal`へ配信する。
- 所属ノードとクラスタ内edgeの`/topological_planar_clusters_incremental/markers/nodes`は変更しない。
- 各topicが独立して消滅クラスタのDELETEマーカーを管理する。

## Behavior Impact

- TopoFuzzy ViewerとRVizで、凸包を残したまま法線矢印だけ非表示にできる。
- 従来の`/topological_planar_clusters_incremental/markers/obb`は配信しない。

## Topics / Params / Messages

- 追加: `/topological_planar_clusters_incremental/markers/hull`
- 追加: `/topological_planar_clusters_incremental/markers/normal`
- 廃止: `/topological_planar_clusters_incremental/markers/obb`
- ROS parameterおよびmessage型の変更なし。

## Verification

- `colcon build --packages-select ais_gng --symlink-install`: Release build成功。
- `test_plane_cluster_incremental`の8件を含む機能テストはすべて成功。
- ROS domain 225でノードを起動し、`hull`、`normal`、`nodes`の3topicが
  `visualization_msgs/msg/MarkerArray`として独立して広告されることを確認した。

## Risk / Notes

- ViewerやRVizで旧`markers/obb`を選択している場合は、`markers/hull`と
  `markers/normal`を個別に選び直す必要がある。
- パッケージ全体のament lintには、今回と無関係な既存のcopyright・format違反が残る。
