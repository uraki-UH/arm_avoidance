# 2026-08-14 - TopoFuzzy point cloud capacity

## Summary

Viewerの点群上限を、詳細な把持対象も確認できる500000点へ引き上げた。

## Changed

- `pointcloud_max_points` の既定値を `100000` から `500000` へ変更した。
- gateway内部でlaunchを経由せず起動した場合も、同じ既定値になるよう統一した。

## Behavior Impact

- 500000点以下の入力点群はViewer転送時に間引かれない。
- 500000点を超える入力は、点群全域から決定論的に均等抽出する。
- `0` を指定すると点数制限なしで転送する。高解像度点群を高周期で送る場合はGPU負荷が増える。

## Topics / Params / Messages

- topic名とROS message型の変更はない。
- `viewer_stack.launch.py` の `pointcloud_max_points` は起動時に任意の非負整数へ変更できる。

## Verification

- `colcon build --packages-select topo_fuzzy_viewer --symlink-install`: Dockerのgng_cpuコンテナ内で成功。

## Risk / Notes

- AMD GPUドライバでWebGLリセットが起きる環境では、500000点・10 Hzは負荷が高い可能性がある。
