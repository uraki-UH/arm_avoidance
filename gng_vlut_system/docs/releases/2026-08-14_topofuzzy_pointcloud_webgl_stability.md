# 2026-08-14 - TopoFuzzy point cloud WebGL stability

## Summary

GraspNet点群とAiS-GNGを連続表示した際に、ChromeのWebGLコンテキストが失われる問題への負荷制限を追加した。

## Changed

- PointCloud2はSensorDataQoSのdepth 1で購読し、表示待ちがある場合は古いフレームを保持しない。
- Viewerへ送る点群は入力全域から均等に抽出し、既定で1フレーム最大100000点、最大10 Hzとした。
- Frontendはトピックごとに最新の受信フレームだけを次の描画まで保持する。
- 点群GPUバッファをstream用途として更新し、実データ範囲だけをGPUへ転送する。
- WebGLはhigh-performance GPUを要求し、MSAAを無効化した。

## Fixed

- ブラウザ描画が遅れた際に、古い点群のデシリアライズとReact更新が積み上がる問題を修正した。
- WebSocket送信待ちがある接続へ点群フレームを追加し続ける問題を修正した。
- 変化していない色・強度バッファと点群境界球を毎フレーム再計算する処理を削減した。

## Behavior Impact

- GNGへの入力点群、学習ノード、学習周期は変更しない。制限対象はViewerへの転送と描画だけである。
- 100000点を超える点群は先頭100000点ではなく、点群全域から決定論的に均等抽出される。
- ブラウザまたはネットワークが遅れた場合は、表示の遅延を蓄積せず古い点群フレームを破棄する。
- Canvasのアンチエイリアスは無効になる。点群形状と色情報は維持する。

## Topics / Params / Messages

- topic名とROS message型の変更はない。
- `viewer_stack.launch.py` に `pointcloud_max_points` を追加した。既定値は `100000`、`0`で点数制限なし。
- `viewer_stack.launch.py` に `pointcloud_max_hz` を追加した。既定値は `10.0`、`0`で周期制限なし。
- `viewer_stack.launch.py` に `websocket_max_backpressure_bytes` を追加した。既定値は `8388608`。

## Verification

- `npm run lint`: Dockerのfrontendコンテナ内で成功。
- `npm run build`: Dockerのfrontendコンテナ内で成功。
- `colcon build --packages-select topo_fuzzy_viewer --symlink-install`: Dockerのgng_cpuコンテナ内で成功。
- `viewer_ws_gateway_node` を別ポート19001で5秒起動し、新規parameterの読込、WebSocket listen、SIGINT終了を確認した。

## Risk / Notes

- `powerPreference: high-performance` はGPU選択の要求であり、最終的なGPU選択はChromeとOSが決定する。
- AMD GPUドライバのpage faultとgfx ring resetを再発させる危険を避けるため、実ブラウザでの数分間の旧条件再現試験は行っていない。
- 稼働中のgatewayはビルド前バイナリを保持するため、適用には `viewer_stack.launch.py` の再起動が必要である。
- backendビルドには今回と無関係な既存warningが3件残っている。
