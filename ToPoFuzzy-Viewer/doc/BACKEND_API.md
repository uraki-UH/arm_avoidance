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

現行配信は `TMG1` バイナリversion 2（96バイト/node）。外側のWSプロトコルはv2のまま。
Viewerは旧version 1（84バイト/node）も読込可能。以下は互換JSON表現。

環境GNGの`/topological_map`に限り、配信時の`clusters[].nodeIds`はノード配列添字。
Frontend受信時にIDへ正規化し、描画・検査用スナップショットではノードIDとして利用。
他のGraphの既存ID方式は維持。トピック別名の自動推定や配信形式の変更なし。
確定クラスタのHuman/Car分類は所属ノード色・レイヤー内件数へ反映。`Clusters`操作で既存の円柱・ボックス表示を切替。
入力ノードの幾何ラベルは変更せず、追加ROSトピック・追加パラメータもなし。
詳細は[Graph Stream](../common/ws_protocol_v2.md#graph-stream)を参照。

```json
{ "type": "stream.graph", "graph": { "timestamp": 0, "nodes": [{ "id": 1, "x": 0.0, "y": 0.0, "z": 0.0, "isGoal": false, "is_boundary_candidate": true, "num_safe_states": 0, "num_danger_states": 0, "num_collision_states": 0 }], "edges": [], "clusters": [] } }
```

`is_boundary_candidate` はGNGからの境界候補フラグ。専用トピック・Viewer側での次数再集計は不要。
`boundary_evidence` はGNG実行側からの観測証拠ビット（遮蔽1・自由空間2・視野端4、0は不明）。併存可能。
実測レイと局所面延長の比較結果であり、真の物体境界の確定情報ではない。Viewerは受信属性の表示のみ。
バイナリでは候補フラグがノードレコード内オフセット5、証拠がオフセット6の各1バイト。詳細は `common/ws_protocol_v2.md` を参照。

`num_safe_states`・`num_danger_states`・`num_collision_states`はL0の集約元姿勢件数。
各`uint32`でnode内offset 84・88・92。`label`とは独立した色の混合とノード詳細表示用。
合計0は未収録扱い。旧version 1にはこの属性なし。ROS側で集計し、Viewer側で安全判定の再計算なし。
ROSメッセージの拡張に伴い送受信ノードの再ビルド・再起動とFrontendの更新が必要。

### 非平面成分のGraph表示

CPUの`/nonplane_components`はROSでは`std_msgs/msg/UInt32MultiArray`のまま。`sources.list`の型は`nonplane_component`、選択後のWS出力は同じtagの`TMG1` Graph。旧Marker JSON生成・追加ROS Graphトピックなし。

- バックエンドで`/topological_map`・`/plane_clusters`・成分所属の`frame_number`を照合。平面と元Graphの`frame_id`も一致必須。各入力の最新だけを保持し、到着時に再照合。全フレーム配信の保証なし、不一致フレームの混合なし。
- 補助入力`/plane_clusters`は発行元の存在中だけ購読。既存の1秒周期確認で接続・解除し、停止時に平面キャッシュも破棄。平面OFF時のSubscriberだけによるROSトピック維持なし。発行元の復帰時はtransient-localの最新データを再取得。選択済みStreams項目の保持は従来どおり。
- 元ノードID、Graph配信対応の法線・勝者入力共分散・ラベル・境界属性を保持。`clusters[].nodeIds`は非平面成分の元ノードID、`edges`は出力配列添字。
- 成分内の実エッジと平面への実接続エッジだけを保持。平面側端点もGraphへ含めるが成分所属には含めず、`nonplaneComponentId=UINT32_MAX`。Bbox・成分の独立表示は非平面所属ノードだけが対象。
- 成分ごとの共通Graph色を使用、平面側端点は灰色。法線・共分散の可視化は既存Graph設定。色・寸法は旧Marker表示と同一ではない。
- 空成分は空Graphで旧表示を消去。送信制御は通常Graphと共通の描画完了通知方式。全クライアント切断後の再接続では再購読が必要。
- `/nonplane_components`のBounding Box設定は未指定、GUI・独立ビュー選択は対象外。Graph表示・法線・共分散は維持。旧設定が残る場合はページ再読み込みが適用手順。

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

把持ラベル付き点群はHTMLの`/semantic_points`を使用。`/handle_points`専用配信は廃止し、把持部位の生成機能は維持。
GNGの数値ラベル1はViewerで「把持部位」と表示し、到達性2〜4と共通の「把持ラベル」グループへ統合。
ROSメッセージ形式の変更なし。[ラベル値と設定移管](../common/ws_protocol_v2.md#graph-stream)を参照。

- `sources.list`
- `sources.setActive` (`{ sourceId, active, removeLayer? }`)
  - `active=false, removeLayer=true` stops the subscription and emits stream deletion events for the corresponding scene layer.

配信元の停止時は旧Scene Layersと送信待ちを削除し、Streamsの選択を維持。
可視化設定は削除せず、同じブラウザセッション内の同名トピック復帰時に再適用。
点群の表示ON/OFF・透明度・手動変換も保持し、旧点群バッファだけを破棄。
停止中も選択済み項目は`sources.list`に残り、同名配信元の復帰時に自動再購読。
グラフの描画完了待ちも初期化。ROS discovery反映後、1秒周期で確認。
詳細は[ストリームの停止・復帰](../common/ws_protocol_v2.md#stream-reset)を参照。

`PoseArray`と`gng_control_msgs/msg/GraspCandidateArray`も表示対象。`sources.list`の型は`marker`で、元のROSトピック名をsource IDとして使用。
例: `/grasp_pose_cands`を有効化すると`source_type: "pose_array"`付きの`stream.marker_array`を配信。
外部のMarker変換ノード・把持計画launchへの依存なし。空候補、再接続キャッシュ、購読解除も既存ストリーム契約に準拠。
他のMarkerとの自動照合・色統合なし。詳細は`common/ws_protocol_v2.md`を参照。
候補配列はID・位置・完全な姿勢・状態を配信。別のreachability購読なし。
色・寸法・基準位置・補助軸はブラウザの共通矢印設定。把持候補は共有スタイルで矢先位置基準・補助軸OFFを指定。
標準Markerの矢印は`arrow_style_id`で共有`arrow_styles`辞書を参照。辞書は初回・変更・再接続時に配信し、省略時は前回値を保持。
候補状態色も同じ辞書の`candidate_state.state_colors`で配信し、ROSとViewerで共通のsRGB定義を使用。
Marker・PoseArray・候補の購読QoSは送信元に追従し、遅着・再起動時も必要に応じて再購読。
標準`TEXT_VIEW_FACING`の文字は`type: "text"`・`text`として配信し、カメラへ正対する文字として描画。
`/grasp_pose_refined/markers`の棄却理由も同じ経路を使用。高さ・色・改行・削除は[文字Marker仕様](../common/ws_protocol_v2.md#text-marker)を参照。
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

- 入力: `{ source_id, selection?, graph?, marker_array?, enable_bounds_only? }`。`graph`または`marker_array`は取得時の受信フレーム全体で、ブラウザ側のノード切り出しなし。
- `selection`: `{ kind: "node" | "cluster" | "component" | "marker", id, ns? }`。`marker`のみ`ns`必須。
- 通常出力: `{ source_id, selection, title, graph, frame_id, min_position, max_position, node_color, node_diameter }`。`selection`は解決後の所属。座標系は入力元、寸法はノード中心群のAABB、単位m。
- `enable_bounds_only=true`では`{ source_id, selection, frame_id, min_position, max_position, node_diameter }`だけを返却。ノード列・エッジ列の返送とエッジ再対応なし。既定falseで通常取得の挙動を維持。
- `enable_bounds_only=true`かつ`selection`省略時は`{ bounds: [...] }`で物体単位の範囲を一括返却。明示クラスタ・非平面component・SPHERE_LIST候補が対象で、所属なしノードを物体として推測せず、空入力は空配列。候補ごとの全ノード再走査なし。
- ノードクリックは`clusters[].nodeIds`の明示所属を優先し、次に有効な`nonplaneComponentId`を利用。所属なしは単一ノード、複数クラスタへの所属はエラー。形状・接続からの物体推測なし。
- 所属はノードID、`edges`は配列添字。切り出し先の添字へ再対応し、選択クラスタの属性は保持。
- `SPHERE_LIST`は同一source・namespace・ID単位。既存把持候補の`grasp_plane`と`grasp_nonplane`だけは同じIDの部品を合算。別namespaceの同一IDは混入なし。
- Markerの`pos`・`quat`はバックエンドで適用。候補部品のframe不一致はエラー。両端が選択点に一致する既存`LINE_LIST`だけをエッジ化し、補間・メッシュ生成なし。
- Markerの表示色・直径は選択した部品から取得。一般グラフでは`node_color=null`、`node_diameter=0`。
- 入力グラフ上限は20万ノード・100万エッジ。空候補、消失したID、曖昧な所属、不正座標・エッジ等は`INSPECTION_FAILED`。

独立ビューは取得時点の固定表示。「最新を取得」で同じsource・所属IDの最新受信フレームを再要求。
取得失敗時は直前の表示を保持しエラーを表示。主画面のカメラ、TF、ROSデータへの変更なし。
独立ビューのXYZ軸は既定OFF。操作欄の「XYZ軸」で表示切替。ビューを閉じて開き直すとOFF、XYZ寸法の数値表示は常時維持。
「Bbox」も既定OFFで独立した表示切替。受信済みの`min_position`・`max_position`を黄色の線枠で表示。「最新を取得」で範囲を更新、閉じて開き直すとOFF。主画面のBounding Box設定・当たり判定とは非連動、追加RPC・ROS購読なし。
フッターはノード数・エッジ数・XYZ寸法の1行、座標系の説明行なし。独立ビュー内の`nonplane_components`は`nonplane_`へ表示のみ短縮（見出し例: `nonplane_7`）。元source ID・RPC・ROSトピック名は維持、元名はsource表示のツールチップで確認可能。
`map`必須の編集RPCとは別用途であり、独立表示にはTFへの変換不要。

ホバー枠・枠内クリック・一括範囲取得と、ノード・クラスタ・Markerの直接クリックによる独立表示は、Viewerのトピック別Graph設定`enable_bounding_box=true`だけが対象。真偽値の明示指定があるGraphだけGUIの`Bounding Box`を表示し、falseでも再ON可能。`/grasp_pose_cands/Tmap`だけ既定でtrueを指定。`/nonplane_components`・`/topological_map`を含む他グラフは未指定のためGUI・選択機能なし。明示的なOFFは受信更新でも保持。法線などGraph設定を持たないMarkerからの独立表示なし。全OFF時は判定タイマーも停止。これはViewer内の表示設定であり、ROSメッセージやRPCフィールドの追加なし。
ONのトピックは初回から物体全体のAABBで判定し、ホバーでの点・球メッシュのraycastなし。主画面と同じTF・手動表示変換・線枠の余白を適用。OFFで枠・当たり判定・直接選択・詳細取得RPCを停止し、取得中のOFFによる遅延表示・エラー表示も抑止。既に開いた独立ビューの固定表示は維持するが、「最新を取得」には元トピックのONが必要。可操作性楕円体の詳細選択は別機能として維持。
判定は10 Hz、範囲取得は可視ソース全体で同時1件・最大4 Hz。ソースの受信フレームが同じなら再取得なし。重なりはカメラから近い枠を優先。
枠内の点のない場所からも独立ビューの選択が可能。5pxを超えたドラッグ後のクリックは除外。
境界の判定外れは350msの猶予。キャンバス退出・ドラッグ・編集モード・購読解除時は即時非表示。古い要求の遅延応答による枠の再表示なし。
更新中と一過性の取得失敗は直前の範囲を維持し、600ms以上の連続失敗で解除。推定値への代替なし。点群全体・ロボットメッシュはホバー対象外。

## Notes
- Legacy RPC method names are intentionally unsupported in v2.
- Edit regions must be sent in `map` frame.
- If TF to `map` is unavailable, commit fails with `job.failed`.
- Default output topic is `<sourceTopic>/edited`.
