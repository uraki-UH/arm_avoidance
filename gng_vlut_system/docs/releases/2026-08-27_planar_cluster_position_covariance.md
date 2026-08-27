# 2026-08-27 - planar cluster position covariance

## Summary

平面クラスタ所属GNGノードの空間共分散を`PlanarCluster`へ追加した。既存の平面フィット統計を再利用するため、ノード再走査や固有値分解は増やさない。

## Changed

- `PlaneAccumulator::solve()`で算出済みの母共分散を平面フィット結果とクラスタ状態へ保持
- 平面クラスタ出力時に共分散を行優先の`float32[9]`へ変換
- 2760ノード・5344エッジの決定的ベンチマークを追加

## Topics / Params / Messages

- `ais_gng_msgs/PlanarCluster.position_covariance`: `[xx,xy,xz,yx,yy,yz,zx,zy,zz]`、単位`m^2`
- `/topological_planar_clusters_incremental`: 1クラスタ当たり36 byte増加

## Behavior Impact

- 平面クラスタごとに、任意方向`n`の空間分散を`n^T covariance n`で取得できる。
- GNGノードの`winner_point_covariance`は入力残差、今回の値はクラスタ所属位置の広がりであり、意味が異なる。
- 13クラスタでは468 byte/frame、17クラスタでは612 byte/frame増える。

## Verification

- Docker内Releaseビルド成功
- `test_plane_cluster_incremental` 8テスト成功。6x6格子平面の理論分散`0.007291667 m^2`、対称性、法線方向分散`0`を確認
- `grasping_system` 4テスト成功
- 2760ノード・5344エッジ・2000更新を5回測定した中央値:
  - 変更前: mean `0.307 ms`、p50 `0.289 ms`、p95 `0.421 ms`、p99 `0.472 ms`
  - 変更後: mean `0.290 ms`、p50 `0.281 ms`、p95 `0.317 ms`、p99 `0.412 ms`
- 変更後がmeanで`0.017 ms`短く出た差は実行時の揺らぎであり、高速化とはみなさない。測定可能な性能回帰は確認されなかった。
- GraspNetストリームで対称な9要素のpublishを確認。13クラスタ時はupdate `0.70-0.92 ms`、publish `0.42-0.52 ms`

## Risk / Notes

- 共分散はクラスタ形状を楕円体近似する用途向けであり、非凸形状の厳密な最小距離ではない。
- `PlanarCluster.msg`の型定義が変わるため、同メッセージを使うノードは再ビルドと再起動が必要。
- `ais_gng`の対象GTestは成功したが、パッケージ全体の既存copyright・style lintは引き続き失敗する。
