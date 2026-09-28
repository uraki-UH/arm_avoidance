# 2026-09-28 - 平面クラスタ計算の切替統一

## 要約

平面計算の設定を `plane_clustering` に一本化。CPU直結の計算本体と依存処理は維持。
旧 `plane_cluster.direct_enabled` の宣言・launch変換を撤去し、関連launch・試験を移行。

- センサー別YAMLの指定を共通YAMLより優先。
- 共通YAMLの既定値は従来同様 `false`。CPUノード単体の未指定時は従来同様 `true`。
- CPUノードへの直接指定も `-p plane_clustering:=true` または `false`。
- 曲面側の `curve_clustering` と `surface_model.enable` の関係は変更なし。
- 独自YAML・コマンドの旧名は新名への置換が必要。旧名の互換エイリアスなし。
- 起動時の切替であり、稼働中の動的ON/OFFの追加ではない。

## 条件・検証

- 設定解決テスト15件成功。CPU/GPU、ON/OFF、共通値とセンサー値の優先、外部平面入力を確認。
- 共通設定から旧パラメータが生成されないことを回帰テストで確認。
- ais_gngのCPU/GPUビルド・install成功。検証・ビルドプロセスは全て終了。
- 実bag再生・処理時間比較は未実施。計算アルゴリズムの変更なし。
- 既存ROSノードの停止・再起動なし。

実行コマンド（既存 `gng_cpu_container` 内、`/ros2_ws`、ROSとinstallをsource後）：

```bash
PYTHONDONTWRITEBYTECODE=1 timeout 90 python3 src/ais_gng_cpu/src/ais_gng/test/test_clustering_yaml_launch.py
CMAKE_BUILD_PARALLEL_LEVEL=2 timeout 240 colcon build --packages-select ais_gng --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
```
