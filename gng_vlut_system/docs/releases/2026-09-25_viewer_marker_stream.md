# 2026-09-25 - 大容量MarkerによるViewer更新待ちの抑制

## 1. 要約

`/plane_clusters/markers/nodes`をOFFにすると改善する症状を調査。取得した1フレームは134 Marker、ノード12,311点とエッジ端点53,512点、JSON約3.85 MB。Graphと異なりMarkerは反映待ちなしで連続送信していた。描画個数の固定上限到達は未確認。

Markerにもクライアント・トピックごとの反映通知と最新1件の保持を追加。Frontendは1描画周期に最新1件だけ反映。JSON生成を2回から1回へ削減し、送信拒否を未達の反映待ちとして扱わない処理と、送信待ち解消時の再送判定を追加。ノード・エッジの個数、GNG計算、ROSメッセージは維持。

[WS通知と互換性](../../../ToPoFuzzy-Viewer/common/ws_protocol_v2.md#markerの反映完了に合わせた配信)。Release配置済み。既存Viewerを停止していないため、適用はViewerバックエンド再起動とブラウザ再読み込み。

## 2. 条件・検証

| 項目 | 結果 |
| --- | --- |
| 配信の調査 | 平面ノード表示OFF時、描画なしのWS受信でnonplane約10.06 Hz。実画面FPSとは別 |
| 大容量Marker | 取得した座標・個数を使った8更新。反映通知を保留した新クライアントは1件・3,843,065 bytes、同時接続の従来方式は8件・30,744,520 bytes |
| 更新再開 | 1→8へ直接更新。全点数保持とMarker待機中のGraph更新を確認 |
| 互換性 | 旧クライアントの連続受信、購読解除・再開、request.state、再接続、旧サーバーの辞書省略を確認 |
| 回帰・ビルド | Frontend回帰5件、非平面Graphの全6到着順・空Graph・再接続、lint、Frontend build、Backend Release build成功 |
| 未検証 | 実ユーザー画面のGPU時間・FPS改善。固定の描画個数上限が原因という確定なし |
| 後片付け | 診断購読・試験gateway・WS接続は終了。既存ROS・bag・Viewerの停止操作なし |

再現コマンド（ROS環境を読み込んだコンテナ内）:

```bash
ROS_DOMAIN_ID=94 python3 -B /ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/test_marker_stream.py
ROS_DOMAIN_ID=218 python3 -B /ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/test_nonplane_stream.py
```

前者は空きポートで`viewer_ws_gateway_node --ros-args -p port:=<空きポート>`を起動し、finallyで停止。今回の最終試験は57507。診断購読と1フレーム取得は`docker exec gng_cpu_container bash -lc`内の`python3 -u -`、WS調査は`node --input-type=module -e`で有限時間実行し終了済み。
Frontend回帰: `node --test tests/marker_stream.test.mjs tests/stream_restart.test.mjs tests/marker_array_renderer.test.mjs tests/human_car_display.test.mjs`。
