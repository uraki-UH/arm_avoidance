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
色・寸法・基準位置・補助軸はブラウザの共通矢印設定。把持候補は共有スタイルで矢先位置基準・補助軸OFFを指定。
標準Markerの矢印は`arrow_style_id`で共有`arrow_styles`辞書を参照。辞書は初回・変更・再接続時に配信し、省略時は前回値を保持。
候補状態色も同じ辞書の`candidate_state.state_colors`で配信し、ROSとViewerで共通のsRGB定義を使用。
Marker・PoseArray・候補の購読QoSは送信元に追従し、遅着・再起動時も必要に応じて再購読。
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

## 候補の独立表示

`edit.inspect_graph`は`viewer_edit_node`による読取専用RPC。編集セッション、追加のROS購読・publishは不要。

- 入力: `{ source_id, selection, graph?, marker_array?, enable_bounds_only? }`。`graph`または`marker_array`は取得時の受信フレーム全体で、ブラウザ側のノード切り出しなし。
- `selection`: `{ kind: "node" | "cluster" | "component" | "marker", id, ns? }`。`marker`のみ`ns`必須。
- 通常出力: `{ source_id, selection, title, graph, frame_id, min_position, max_position, node_color, node_diameter }`。`selection`は解決後の所属。座標系は入力元、寸法はノード中心群のAABB、単位m。
- `enable_bounds_only=true`では`{ source_id, selection, frame_id, min_position, max_position, node_diameter }`だけを返却。ノード列・エッジ列の返送とエッジ再対応なし。既定falseで通常取得の挙動を維持。
- ノードクリックは`clusters[].nodeIds`の明示所属を優先し、次に有効な`nonplaneComponentId`を利用。所属なしは単一ノード、複数クラスタへの所属はエラー。形状・接続からの物体推測なし。
- 所属はノードID、`edges`は配列添字。切り出し先の添字へ再対応し、選択クラスタの属性は保持。
- `SPHERE_LIST`は同一source・namespace・ID単位。既存把持候補の`grasp_plane`と`grasp_nonplane`だけは同じIDの部品を合算。別namespaceの同一IDは混入なし。
- Markerの`pos`・`quat`はバックエンドで適用。候補部品のframe不一致はエラー。両端が選択点に一致する既存`LINE_LIST`だけをエッジ化し、補間・メッシュ生成なし。
- Markerの表示色・直径は選択した部品から取得。一般グラフでは`node_color=null`、`node_diameter=0`。
- 入力グラフ上限は20万ノード・100万エッジ。空候補、消失したID、曖昧な所属、不正座標・エッジ等は`INSPECTION_FAILED`。

独立ビューは取得時点の固定表示。「最新を取得」で同じsource・所属IDの最新受信フレームを再要求。
取得失敗時は直前の表示を保持しエラーを表示。主画面のカメラ、TF、ROSデータへの変更なし。
`map`必須の編集RPCとは別用途であり、独立表示にはTFへの変換不要。

ホバー枠は同じ所属解決によるAABBを利用し、主画面と同じTF・手動表示変換を適用。
選択可能なノード・クラスタだけを最大10 Hzで当たり判定し、計測時間が長い場合は間隔を拡大。
範囲取得は同時1件・最大4 Hz、対象と受信フレームが同じなら再取得なし。
カーソル外れ・ドラッグ・編集モード・購読解除時は非表示。古い要求の遅延応答による枠の再表示なし。
取得失敗時は枠を表示せず、推定値で代替なし。点群全体・ロボットメッシュはホバー対象外。

## Notes
- Legacy RPC method names are intentionally unsupported in v2.
- Edit regions must be sent in `map` frame.
- If TF to `map` is unavailable, commit fails with `job.failed`.
- Default output topic is `<sourceTopic>/edited`.
