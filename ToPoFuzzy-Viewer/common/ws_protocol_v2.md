# WebSocket Protocol v2

## Transport
- Endpoint: `ws://<host>:9001`
- Binary frames: 点群（`common/protocol.md`）およびグラフ（`TMG1`）
- Text frames: JSON request/response and asynchronous events

## Request
```json
{ "id": "uuid", "method": "xxx.yyy", "params": { } }
```

## Response
### Success
```json
{ "id": "uuid", "ok": true, "result": { } }
```

### Error
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

## Events

### Point Cloud Metadata
```json
{ "type": "stream.pointcloud.meta", "topic": "/points", "tag": "/points", "frameId": "lidar_link" }
```

`frameId`は入力`PointCloud2.header.frame_id`。フロントエンドで固定座標系へのTF適用と状態表示に使用。

### Graph Stream

現行のグラフ配信は `TMG1` バイナリ。以下は互換JSON表現。

```json
{ "type": "stream.graph", "graph": { "timestamp": 0, "nodes": [{ "id": 1, "x": 0.0, "y": 0.0, "z": 0.0, "isGoal": false, "is_boundary_candidate": true }], "edges": [], "clusters": [] } }
```

`is_boundary_candidate` は `TopologicalNode` のGNG側境界候補属性。Viewer内での次数判定なし。
`boundary_evidence` は候補付近の観測証拠ビット。1=遮蔽、2=自由空間、4=視野端、0=不明。複数ビットの併存が可能。
バイナリでは同レコードのオフセット6、レコード長84バイトは維持。属性欠落・旧予約値0は不明。
局所面延長との比較による証拠であり、物体の真の境界や仮説全体の棄却の確定情報ではない。
バイナリでは84バイトのノードレコード先頭から5バイト目（0起点）に格納、0がfalse、1がtrue。
予約領域1バイトの利用によるレコード長・バージョンの維持。旧形式の予約領域0、またはJSON属性欠落時は候補扱いなし。
ROSメッセージ定義の互換性とは別のため、ROS送受信側には同一定義での再ビルド・再起動が必要。

### PoseArray表示

`geometry_msgs/msg/PoseArray`は`sources.list`で`type: "marker"`として公開。
`sources.setActive`で選択後、ROS Markerトピックを経由せず、gateway内でローカルZ軸の矢印へ変換。

```json
{"type":"stream.marker_array","tag":"/grasp_pose_cands","source_type":"pose_array","markers":[]}
```

- 1姿勢につき1本のローカル+Z矢印。長さ0.08 m、未評価色は水色。
- `markers`は毎回全置換。空配列は旧候補の消去。
- Markerの`frameId`は入力座標系、`header_stamp: [sec, nanosec]`は入力更新時刻。
- 非有限位置・無効クォータニオンは除外。入力配列添字をMarker IDとして維持。
- 入力publisherに合わせたreliability・durabilityを購読開始時に選択。
- 計画処理は不要。固定座標系へのTFは描画側で従来どおり必要。
- `<tag>/reachability_markers`も表示ONの場合、同じID・時刻・座標系・両端点の評価色だけを候補へ統合し、評価側の重複矢印は非描画。
- 候補レイヤーを非表示にした場合、評価側は独立表示。異なる名前へremapした評価トピックは自動統合の対象外。

### Stream Reset
```json
{ "type": "stream.reset", "topic": "/points", "tag": "/points" }
```

同名ROSトピックのpublisher GID更新時の描画キャッシュ初期化。フロントエンドは対象トピックの
点群・グラフ・ボクセル・Markerを消去し、後続メッセージで同じレイヤーを再構築する。レイヤーの
可視設定と手動変換は維持する。

### Edit Job Progress
```json
{ "type": "job.progress", "jobId": "job-...", "sessionId": "edit-...", "progress": 35, "stage": "processing" }
```

### Edit Job Completed
```json
{
  "type": "job.completed",
  "jobId": "job-...",
  "sessionId": "edit-...",
  "publishedTopic": "/source/edited",
  "pointCount": 1000,
  "durationMs": 250
}
```

### Edit Job Failed
```json
{
  "type": "job.failed",
  "jobId": "job-...",
  "sessionId": "edit-...",
  "error": { "code": "TF_UNAVAILABLE", "message": "..." }
}
```

## Method Groups
- `sources.*`
- `files.*`
- `publish.*`
- `rosbag.*`
- `gng.*`
- `params.*`
- `edit.*`

Refer to `doc/BACKEND_API.md` for concrete method parameters and response payloads.

The Topics checkbox disables a source with `sources.setActive({ active: false, removeLayer: true })`, so the corresponding scene layer is removed with the subscription.
