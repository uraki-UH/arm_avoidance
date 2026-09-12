# ToPoFuzzyViewer Backend API (WebSocket Protocol v2)

This document describes the **v2** WebSocket API exposed by `viewer_ws_gateway_node`.

## Endpoint
- URL: `ws://<host>:9001`
- Transport:
  - Binary: 点群（`common/protocol.md`）およびグラフ（`TMG1`）
  - Text: JSON request/response + async events

## Request/Response Format

### Request
```json
{ "id": "uuid", "method": "xxx.yyy", "params": { } }
```

### Success Response
```json
{ "id": "uuid", "ok": true, "result": { } }
```

### Error Response
```json
{
  "id": "uuid",
  "ok": false,
  "error": {
    "code": "ERR_CODE",
    "message": "description",
    "details": {}
  }
}
```

## Stream/Event Messages

### `stream.pointcloud.meta`
Published before each binary cloud frame.
```json
{ "type": "stream.pointcloud.meta", "topic": "/points", "tag": "/points", "frameId": "lidar_link" }
```

`frameId`は入力`PointCloud2.header.frame_id`。後続のbinary cloud frameの座標系。

点数制限はバックエンドで有効XYZ点へ適用。上限超過時のみ領域分けなしの全有効点シャッフルと先頭抽出、受信ごとの選択更新。上限以下・無制限設定では元順序のまま全有効点を配信。RGB・intensityの対応維持。配信形式の変更なし。

### `stream.graph`

現行配信は `TMG1` バイナリ。以下は互換JSON表現。

```json
{ "type": "stream.graph", "graph": { "timestamp": 0, "nodes": [{ "id": 1, "x": 0.0, "y": 0.0, "z": 0.0, "isGoal": false, "is_boundary_candidate": true }], "edges": [], "clusters": [] } }
```

`is_boundary_candidate` はGNGからの境界候補フラグ。専用トピック・Viewer側での次数再集計は不要。
`boundary_evidence` はGNG実行側からの観測証拠ビット（遮蔽1・自由空間2・視野端4、0は不明）。併存可能。
実測レイと局所面延長の比較結果であり、真の物体境界の確定情報ではない。Viewerは受信属性の表示のみ。
バイナリでは候補フラグがノードレコード内オフセット5、証拠がオフセット6の各1バイト。詳細は `common/ws_protocol_v2.md` を参照。

### `job.progress`
```json
{ "type": "job.progress", "jobId": "job-...", "sessionId": "edit-...", "progress": 42, "stage": "processing" }
```

### `job.completed`
```json
{
  "type": "job.completed",
  "jobId": "job-...",
  "sessionId": "edit-...",
  "publishedTopic": "/points/edited",
  "pointCount": 12345,
  "durationMs": 1200,
  "message": "Edit commit finished"
}
```

### `job.failed`
```json
{
  "type": "job.failed",
  "jobId": "job-...",
  "sessionId": "edit-...",
  "error": { "code": "TF_UNAVAILABLE", "message": "..." }
}
```

## Methods

## Sources / Stream Control
- `sources.list`
- `sources.setActive` (`{ sourceId, active, removeLayer? }`)
  - `active=false, removeLayer=true` stops the subscription and emits stream deletion events for the corresponding scene layer.

`PoseArray`と`gng_control_msgs/msg/GraspCandidateArray`も表示対象。`sources.list`の型は`marker`で、元のROSトピック名をsource IDとして使用。
例: `/grasp_pose_cands`を有効化すると`source_type: "pose_array"`付きの`stream.marker_array`を配信。
外部のMarker変換ノード・把持計画launchへの依存なし。空候補、再接続キャッシュ、購読解除も既存ストリーム契約に準拠。
他のMarkerとの自動照合・色統合なし。詳細は`common/ws_protocol_v2.md`を参照。
候補配列はID・位置・完全な姿勢・状態を配信。別のreachability購読なし。
色・寸法・基準位置・補助軸はブラウザの共通矢印設定。姿勢入力に描画設定を毎回付加する処理はなし。
標準Markerの矢印は`arrow_style_id`で共有`arrow_styles`辞書を参照。辞書は初回・変更・再接続時に配信し、省略時は前回値を保持。
詳細は[WS v2](../common/ws_protocol_v2.md)と[矢印共通仕様](../common/arrow_visual_spec.md)を参照。

## File
- `files.list`
- `files.load` (`{ path }`)

## Rosbag
- `rosbag.list`
- `rosbag.play` (`{ path, remaps, loop }`)
- `rosbag.stop`
- `rosbag.status`

## GNG
- `gng.listConfigs`
- `gng.start` (`{ inputTopic, configFile?, maxNodes?, learningNum?, voxelGridUnit? }`)
- `gng.stop`
- `gng.status`

## Parameters
- `params.get`
- `params.set` (`{ paramName, value }`)

## Edit Session (AABB / map frame)
- `edit.openSession` (`{ sourceTopic, targetFrame="map" }`)
- `edit.addRegion` (`{ sessionId, min:[x,y,z], max:[x,y,z], frameId="map" }`)
- `edit.removeRegion` (`{ sessionId, regionId }`)
- `edit.clearRegions` (`{ sessionId }`)
- `edit.getSession` (`{ sessionId }`)
- `edit.commit` (`{ sessionId, transform:{position,rotation,scale}, outputTopic? }`)
  - returns immediately with `{ jobId }`
  - execution result arrives through `job.*` events
- `edit.cancelSession` (`{ sessionId }`)

## Notes
- Legacy RPC method names are intentionally unsupported in v2.
- Edit regions must be sent in `map` frame.
- If TF to `map` is unavailable, commit fails with `job.failed`.
- Default output topic is `<sourceTopic>/edited`.
