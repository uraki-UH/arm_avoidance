# 2026-08-27 - GNG topological map delta API

## Summary

バイナリ化後も後段処理がGNG内部へ依存せず増分更新できるよう、node・edge変更一覧を取得するread-only C ABIを`gng_cpu`へ追加した。

## Added

- 差分記録の有効化・無効化: `gng_setMapDeltaCapture(uint8_t enable)`
- 直近の`gng_exec()`で確定した差分取得: `gng_getTopologicalMapDelta()`
- node差分: ADD / UPDATE / REMOVE
- edge差分: ADD / REMOVE。端点は辞書順に正規化
- node ID再利用を区別する`(id, frame)`キー

## Behavior Impact

- GNGの学習、node・edge生成、削除、labeling、クラスタリング結果は変更しない。
- 現時点の`plane_cluster_incremental`はこのAPIを使用しておらず、従来どおり毎frame全node・edgeを処理する。平面クラスタ処理時間は今回の変更では短縮されない。
- 差分記録は既定で無効。初回有効化までは変更一覧用バッファを確保せず、無効時の記録処理は早期returnする。
- node UPDATEは同一frame内でIDごとに1件へまとめる。node/edgeのADD/REMOVEは発生順に保持する。
- 差分配列はライブラリ所有で、次の`gng_setPointCloud()`または`gng_exec()`まで有効。

## Topics / Params / Messages

- ROS topic、parameter、message型の変更なし。
- `GngMapDelta.version`は`1`。

## Verification

- `gng_cpu`: Release、`GNG_BUILD_BENCHMARKS=ON`、frame log無効でビルド成功。
- `ais_gng`: Releaseビルド成功。
- `gng_map_delta_api_test`: `node_deltas=26 edge_deltas=22 api_test=passed`。
- 既存`gng_training_event_api_test`: `captured_event_num=4000 api_test=passed`。
- `nm -D`: `gng_setMapDeltaCapture`、`gng_getTopologicalMapDelta`のexportを確認。
- Release一時ベンチマーク、2500点、warm-up 20 frame、計測300 frameを各3回実施。中央値はcapture OFF `1.821 ms/frame`、ON `1.833 ms/frame`で、差分記録コストは`0.012 ms/frame`、約`0.7%`。
- 現行の平面クラスタReleaseベンチマークは2760 nodes / 5344 edgesで、平均`0.305 ms`、p95 `0.398 ms`。差分API未接続の全量処理値。

## Risk / Notes

- capture有効化時と取りこぼし時の全量再同期はconsumer側で行う。
- node IDだけでは再利用を判別できないため、consumerは必ず`id`と`frame`を組で扱う。
- node/edge差分は変更一覧だけを持ち、現在値は既存の`gng_getTopologicalMap()`から取得する。
