# WebSocket Protocol v2

## Transport

`Mesh Models` のローカルメッシュ表示は本プロトコルの対象外。
RPC・stream・ROS topicの追加なし。[操作と範囲](../doc/MESH_MODELS.md)。

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

現行のグラフ配信は `TMG1` バイナリversion 2（ノード96バイト）。外側のWSプロトコルはv2のまま。
Viewerは旧バイナリversion 1（ノード84バイト）も読み込み可能。以下は互換JSON表現。

環境GNGの`/topological_map`は、ROSおよび配信時の`clusters[].nodeIds`が`nodes`の配列添字。
Viewerは受信直後に実ノードIDへ正規化。バイナリversion 1/2と互換JSONの両経路で同じ処理。
他のGraph（非平面成分・テンプレート等）の所属は元からIDであり、変換対象外。
トピック名の完全一致による区別で、数値からの添字／IDの推測や別名への自動適用はなし。
正規化後の描画・検査用スナップショットでは、所属は一貫してノードID。範囲外の添字は所属対応のみ除外。
ROSメッセージ・配信バイト列・WSバージョンの変更なし。

確定クラスタの`label`（4=Human、5=Car）を所属ノードの表示分類へ反映。
元の`nodes[].label`は保持し、Viewer側で再分類や確定判定なし。確定解除・クラス変更も次のGraphで更新。
属性色・明示的な単色指定など既存の優先順位を維持。
レイヤー欄の確定件数と`Clusters`切替を追加。人の円柱・車のボックスは既定OFF、所属ノード色とは独立。

ノードの`semanticLabel`は「把持ラベル」グループで表示・色・優先順位を設定。
0=通常、1=入力で指定された把持部位、2=候補到達性の未評価、3=到達範囲内、4=到達範囲外。
把持部位と到達性は異なる意味であり、1から到達可能性の推定は行わない。
HTMLのラベル付き`/semantic_points`は0/1のみを生成。専用`/handle_points`配信は廃止。
内部の子ラベルID`handle`とROS数値1は互換性のため維持し、GUI表記のみ「把持部位」へ変更。
旧`grasp_reachability`の非表示設定は到達性の子ラベルへ移管し、把持部位の表示・色は独立して保持。
境界候補とはグループ単位で優先順位を設定。統合前の2親項目のうち先に指定された位置へ移管。

```json
{ "type": "stream.graph", "graph": { "timestamp": 0, "nodes": [{ "id": 1, "x": 0.0, "y": 0.0, "z": 0.0, "isGoal": false, "is_boundary_candidate": true, "num_safe_states": 0, "num_danger_states": 0, "num_collision_states": 0 }], "edges": [], "clusters": [] } }
```

`is_boundary_candidate` は `TopologicalNode` のGNG側境界候補属性。Viewer内での次数判定なし。
`boundary_evidence` は候補付近の観測証拠ビット。1=遮蔽、2=自由空間、4=視野端、0=不明。複数ビットの併存が可能。
バイナリではノードレコードのオフセット6。属性欠落・旧予約値0は不明。
局所面延長との比較による証拠であり、物体の真の境界や仮説全体の棄却の確定情報ではない。
境界候補フラグはノードレコード先頭から5バイト目（0起点）に格納、0がfalse、1がtrue。
旧形式の予約領域0、またはJSON属性欠落時は候補扱いなし。
ROSメッセージ定義の互換性とは別のため、ROS送受信側には同一定義での再ビルド・再起動が必要。

version 2は末尾に集約元姿勢の表示用件数を追加。先頭84バイト、Header、edge、clusterの配置は変更なし。

| node内offset | 型 | 属性 |
|---:|---|---|
| 84 | `uint32` | `num_safe_states`（安全） |
| 88 | `uint32` | `num_danger_states`（危険） |
| 92 | `uint32` | `num_collision_states`（衝突／使用不可） |

合計が正の場合はsafe・danger・collisionの色を件数割合で混色。`label`や判定ロジックの変更なし。
件数のみの変更も描画更新対象。欠落・不正値・合計0は従来のlabel色へ復帰。
ノード詳細に件数と割合を表示。元姿勢の構成比であり、安全確率ではない。
単色・目標・semantic・境界などの明示的色指定は優先。詳細は[GNG仕様](../../gng_vlut_system/docs/TECHNICAL_SPEC.md#134-rosパラメータとトピック)を参照。

`/nonplane_components`のROS所属配列も、Viewerでは既存`TMG1`へ変換。`sources.list`の型は`nonplane_component`のまま、tagも元トピック名を維持。Marker JSONの併送なし。元ノードID・Graph対応属性・成分所属・実エッジを保持し、平面側接続端点だけは成分所属から除外。通常Graphと同じ`stream.topological_map.applied`で描画完了を通知。構築条件・空成分・再購読は[非平面成分のGraph表示](../doc/BACKEND_API.md#非平面成分のgraph表示)を参照。

補助平面入力のROS購読は発行元の存在に追従。停止時は購読と平面キャッシュを破棄し、復帰時に再接続。WSフィールド・バイナリ形式・選択項目の保持仕様の変更なし。

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

### Text Marker

`visualization_msgs/msg/Marker`と`MarkerArray`の`TEXT_VIEW_FACING`は、通常Markerと同じ配信経路で
`type: "text"`と`text`本文を保持。特定のトピック名への依存なし。

- 位置は`frameId`・`pos`と既存の手動変換を適用。文字面はカメラへ正対。
- `scale[2]`を大文字高さの基準[m]として使用。`scale[0:2]`は文字寸法に不使用。
- `color`のRGB・透明度を使用。空本文、無効または正でない高さ、削除actionは非表示。
- 改行対応。ブラウザ内のCanvasTextureを本文変更時だけ生成し、交換・非表示時に解放。
- 外部フォント取得なし。長文はテクスチャ辺長2048以内に縮小し、表示上の寸法比を保持。

### Stream Reset
```json
{ "type": "stream.reset", "topic": "/points", "tag": "/points" }
```

`stream.reset`は描画キャッシュ削除の互換イベント。配信元の停止・GID変更時は通常の
`stream.delete`等の削除イベントを配信し、点群・グラフ・ボクセル・Markerの旧Scene Layersを削除。
Streamsの選択は維持し、停止中も`sources.list`へ`active: true`の項目を残す。
同名配信元の復帰時は購読を生成し直し、後続データからレイヤーを再構築。
Marker系のQoSも再評価。グラフの未ACK・送信済み版番号・未送信キャッシュは世代を跨いで保持しない。
確認周期は1秒。停止検出にはROS discoveryの反映時間も必要で、メッセージ間隔だけによる停止判定はなし。
同じブラウザセッションでは、同名トピックの色・ラベル・表示ON/OFFなどの設定を維持。
点群の透明度・手動変換も受信データとは別に保持し、復帰時の新しい点群へ適用。
古い点群バッファの保持なし。ブラウザ再読み込みを跨ぐ永続保存は対象外。

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
- `vehicle.register`

### 読取専用の候補切り出し

`edit.inspect_graph({ source_id, selection?, graph?, marker_array?, enable_bounds_only? })`で取得時のフレームをバックエンドへ送信。
`selection`は`{ kind: "node" | "cluster" | "component" | "marker", id, ns? }`。
通常応答は`{ source_id, selection, title, graph, frame_id, min_position, max_position, node_color, node_diameter }`。
`enable_bounds_only=true`ではホバー用途として`source_id, selection, frame_id, min_position, max_position, node_diameter`だけを返却。既定false。
`enable_bounds_only=true`かつ`selection`省略時は`{ bounds: [...] }`による物体単位の一括AABB。明示クラスタ・非平面component・SPHERE_LISTが対象、所属なしノードの推測なし。
切り出し後のエッジは配列添字、所属IDと座標系は維持。失敗コードは`INSPECTION_FAILED`。
読取専用で追加ストリームなし。所属解決・Marker部品合算・入力制限は[API仕様](../doc/BACKEND_API.md#候補の独立表示)を参照。

Refer to `doc/BACKEND_API.md` for concrete method parameters and response payloads.

The Topics checkbox disables a source with `sources.setActive({ active: false, removeLayer: true })`, so the corresponding scene layer is removed with the subscription.

### 車両表面の位置合わせ

`vehicle.register({ snapshot, dist_th?, support_dist_th? })`で選択クラスタの固定観測を照合。
複数候補、適合度、支持率、未対応率、姿勢、変換済み描画点群を返却。分類保留を明示。
処理は独立ノード。要求idのjobイベントを通知し、結果はRPC応答へ格納。
入力制限・座標系・応答フィールド・エラーは[API仕様](../doc/BACKEND_API.md#車両モデル照合)を参照。
