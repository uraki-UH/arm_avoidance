# 2026-08-14 - TopoFuzzy point cloud safe default

## Summary

AMD GPUのWebGLリセットを再発させないため、ライブPointCloud2の既定送信点数を、WebGL安定化を確認した100000点へ戻した。

## Changed

- `viewer_stack.launch.py` の `pointcloud_max_points` 既定値を300000から100000へ変更した。
- Viewerゲートウェイのパラメータ既定値も100000へ揃えた。

## Behavior Impact

- 既定の最大負荷は10 Hzで毎秒100万点となる。
- 300000点や500000点が必要な場合は、`pointcloud_max_points` launch引数で明示的に指定できる。

## Verification

- `journalctl -k` に、Chromeプロセスに対応するAMD GPUの`ring gfx` timeoutとGPU resetを確認した。
- `b1adc4e web Glクラッシュの解消`時点の既定上限100000点と一致することを確認した。

## Risk / Notes

- 高密度ライブ更新を再び既定にすると、AMD GPUドライバのWebGLリセットが再発する可能性が高い。
