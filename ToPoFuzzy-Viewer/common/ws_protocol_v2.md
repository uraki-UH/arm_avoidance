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

点群の上限超過時のみ、有効XYZ点に対するバックエンドの完全ランダム抽出。領域分けなし、受信ごとに選び直し。上限以下・無制限設定では元順序のまま全有効点を配信。色・intensityは選択元と同一の点。バイナリ形式・メタデータの変更なし。

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

### 姿勢配列・候補表示

`geometry_msgs/msg/PoseArray`と`gng_control_msgs/msg/GraspCandidateArray`は`sources.list`で`type: "marker"`として公開。
`sources.setActive`で選択後、ROS Markerトピックを経由せず、gatewayから完全な姿勢を配信。

```json
{"type":"stream.marker_array","tag":"/grasp_pose_cands","source_type":"pose_array","update_id":1,"markers":[]}
```

- エントリは`id, ns, type:"arrow", action:0, frameId, pos:[x,y,z], orientation:[x,y,z,w]`。
- 色・寸法はブラウザのレイヤー設定。既定はローカル+Z、全長0.08 m、水色。完全な姿勢がある場合は補助2軸の表示が可能。
- `markers`は毎回全置換。空配列は旧候補の消去。
- Markerの`frameId`は入力座標系。
- 非有限位置・無効クォータニオンは除外。入力配列添字をMarker IDとして維持。
- 入力publisherに合わせたreliability・durabilityを選択。500 msごとに確認し、遅着・再起動・送信元混在にも追従。
- 計画処理は不要。PoseArrayは座標系不明・固定座標系へのTF欠落時に非表示。座標をworldとみなす代替描画はなし。
- 他のMarkerレイヤーとの自動照合・色統合なし。重複を避ける場合は表示レイヤーを選択。

候補配列ではMarker IDに候補の`id`を使用し、`state`を数値のまま付与。
表示側の既定パレットは未評価=黄、範囲内=明るい薄緑水色、範囲外=灰。ブラウザで変更可能。
`source_type: "pose_array"`は既存の姿勢描画・TF必須経路の識別用。新しい描画コンポーネントの追加なし。
`update_id`は候補集合の更新番号。同一集合の状態更新では維持、空配列を含む全置換で旧候補を消去。
配信元は候補生成ノードだけ。Viewerで別トピックの状態を突き合わせる処理はなし。

### 共通矢印スタイル

標準Markerの矢印では、`scale`と`color`をレイヤー内の`arrow_styles`辞書に集約。

```json
{"type":"stream.marker_array","tag":"/arrows","arrow_styles":{"0":{"scale":[0.008,0.016,0.02],"color":[0.2,0.8,1,1]}},"markers":[{"id":1,"ns":"normal","type":"arrow","action":0,"frameId":"world","pos":[0,0,0],"quat":[0,0,0,1],"points":[[0,0,0],[0,0,0.08]],"arrow_style_id":"0"}]}
```

候補配列は`arrow_style_id: "candidate_state"`を持ち、次の辞書を参照。

```json
{"candidate_state":{"anchor":"tip","enable_transverse_axes":false,"state_colors":{"0":"#f3da59","1":"#7ceeb6","2":"#c4c4c4"}}}
```

状態パレットは`arrow_visualization/state_colors.hpp`の共通sRGB定義から生成。
ROS Marker用には同じ値をlinear RGBへ変換。辞書は他の矢印スタイルと同じ送信省略・再接続規約。

- 同一設定の矢印は同じ`arrow_style_id`を参照。
- `arrow_styles`の省略は直前の辞書を保持。存在する場合は辞書全体を置換。`{}`は空辞書への更新。
- 初回・設定変更時・`request.state`・再接続時には完全な辞書を送信。レイヤー削除時は辞書も破棄。
- `markers`は従来どおり全置換。辞書の省略とは独立。
- `orientation`形式の姿勢入力には矢印ごとの描画設定なし。候補の状態色は辞書を参照。
- `scale`・`color`・`points`は省略可能。形式に応じて姿勢または端点を使用。
- publisher再起動の`stream.reset`後も次の配信で辞書を再送。
- 旧形式のインライン`scale`・`color`も入力可能。矢印以外のMarker規約は変更なし。
- 補助軸、anchor、ブラウザ設定の詳細は[共通仕様](arrow_visual_spec.md)を参照。

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
