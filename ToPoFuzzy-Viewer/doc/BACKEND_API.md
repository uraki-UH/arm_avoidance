# ToPoFuzzyViewer Backend API

## Endpoint

`ws://127.0.0.1:9001`。WS v2の要求・応答・イベント、Origin制限、表示専用`/observe`の許可操作は[通信仕様](../common/ws_protocol_v2.md)。
バイナリ点群は[PCDX](../common/protocol.md)、Graphは`TMG1`。

## Sources / Stream Control

- `sources.list`
- `sources.setActive({ sourceId, active, removeLayer? })`
- `request.state`: RPCではなく`{ type: "request.state", topic?: "/対象トピック" }`。

`active=false, removeLayer=true`は購読停止とレイヤー削除。
配信元の停止・復帰、トピック限定再取得、点数制限、Graphの所属ID、MarkerのACK・QoS・矢印スタイルは[WS v2](../common/ws_protocol_v2.md)を参照。

### 非平面成分のGraph表示

CPUの`/nonplane_components`はROSでは`std_msgs/msg/UInt32MultiArray`のまま。`sources.list`の型は`nonplane_component`、選択後のWS出力は同じtagの`TMG1` Graph。旧Marker JSON生成・追加ROS Graphトピックなし。

- バックエンドで`/topological_map`・`/plane_clusters`・成分所属の`frame_number`を照合。平面と元Graphの`frame_id`も一致必須。各入力の最新だけを保持し、到着時に再照合。全フレーム配信の保証なし、不一致フレームの混合なし。
- 補助入力`/plane_clusters`は発行元の存在中だけ購読。既存の1秒周期確認で接続・解除し、停止時に平面キャッシュも破棄。平面OFF時のSubscriberだけによるROSトピック維持なし。発行元の復帰時はtransient-localの最新データを再取得。選択済みStreams項目の保持は従来どおり。
- 元ノードID、Graph配信対応の法線・勝者入力共分散・ラベル・境界属性を保持。`clusters[].nodeIds`は非平面成分の元ノードID、`edges`は出力配列添字。
- 成分内の実エッジと平面への実接続エッジだけを保持。平面側端点もGraphへ含めるが成分所属には含めず、`nonplaneComponentId=UINT32_MAX`。Bbox・成分の独立表示は非平面所属ノードだけが対象。
- 成分ごとの共通Graph色を使用、平面側端点は灰色。法線・共分散の可視化は既存Graph設定。色・寸法は旧Marker表示と同一ではない。
- 空成分は空Graphで旧表示を消去。送信制御は通常Graphと共通の描画完了通知方式。全クライアント切断後の再接続では再購読が必要。
- `/nonplane_components`のBounding Box設定は未指定、GUI・独立ビュー選択は対象外。Graph表示・法線・共分散は維持。旧設定が残る場合はページ再読み込みが適用手順。

## File

ローカルメッシュの `Mesh Models` はFrontend内だけの表示機能。
以下の点群ファイルRPCは使用せず、ROS配信やサーバーへのアップロードもなし。
[対応形式・操作・制限](MESH_MODELS.md)。

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

## 取付校正

`viewer_param_node`からROS側の`/sensor_calibration/request`・`/sensor_calibration/response`へ委譲。同じv2要求形式でCLIからも利用可能。

- `calibration.list` (`{ source_frame? }`): 対象・校正版・TF反映状態・更新可否。
- `calibration.apply` (`{ mount_id, revision, source_frame, delta:{pos,rot_deg,scl}, operation_id }`): Viewerの増分補正。角度はEuler XYZ [deg]、スケールは1。操作IDによる再送の二重適用防止。
- `calibration.consumer` (`{ mount_id, node_name, revision, mode:"tf"|"restart" }`): 利用側の反映確認。node_nameは完全修飾名。tfは各測定で最新TFを使用する契約、restartは再初期化必須。5秒以内の継続通知が必要。

`source_frame`は取付child自身または固定内部TFで接続された測定フレーム。world等へ変換済みの点群は対象外。
`is_tf_applied`はTF配信、`status=ready`は利用側を含む更新可能状態。IMU反映待ちはpending、未対応・未接続はblocked、配信元の重複はconflict。
選択点群の表示補正はTF反映確認後に0へ初期化。拒否時は保持。`stream.capabilities.sensor_calibration`非対応の旧バックエンドへの要求送信なし。

## Edit Session (AABB / map frame)

- `edit.openSession` (`{ sourceTopic, targetFrame="map" }`)
- `edit.addRegion` (`{ sessionId, min:[x,y,z], max:[x,y,z], frameId="map" }`)
- `edit.removeRegion` (`{ sessionId, regionId }`)
- `edit.clearRegions` (`{ sessionId }`)
- `edit.getSession` (`{ sessionId }`)
- `edit.commit` (`{ sessionId, transform:{position,rotation,scale}, outputTopic? }`)
  - 即時応答は`{ jobId }`。結果は`job.*`イベント。
  - `rotation`はThree.js Euler XYZ [rad]（Rx*Ry*Rz）。描画と同じ`TF × 手動変換`の合成。削除領域はmap座標。
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

独立ビューは取得時点の固定表示。「最新を取得」で同じsource・所属IDを再要求し、失敗時は直前の表示を保持。TF変換不要、元ROSデータの変更なし。

主画面の候補選択はトピック別`enable_bounding_box=true`だけが対象。`/grasp_pose_cands/Tmap`は既定ON、`/topological_map`と`/nonplane_components`は対象外。
OFF時は枠・当たり判定・選択・詳細要求を停止し、遅延応答も抑止。既存の独立ビューは保持するが再取得には元トピックのONが必要。
範囲取得は全可視ソースで同時1件・最大4 Hz、判定10 Hz。同一フレームは再取得せず、重なりは近い枠を優先。
TF・手動変換を適用した物体全体のAABBを使用。5 px超のドラッグ後はクリック除外、判定外れの猶予350 ms、連続取得失敗600 msで解除。
キャンバス退出・ドラッグ・編集・購読解除時は即時非表示。点群全体・ロボットメッシュは対象外。
独立ビューのXYZ軸・Bboxは初期OFF、主画面の設定と非連動。

## 車両モデル照合

`vehicle.register({ snapshot, dist_th?: 0.25, support_dist_th?: 0.35 })`。
`snapshot`は`edit.inspect_graph`のcluster/component応答。座標系と全選択ノードを保持して送信。
`viewer_vehicle_registration_node.py`が別ワーカーで処理し、同時実行は1件。通信側は60秒待機。
返却は`{ source_id, selection, frame_id, timestamp, observation_kind: "gng_nodes", state, message, candidates, elapsed_ms, dist_th, support_dist_th, limitations }`。
`state`は`class_candidate`（形状候補）、`ambiguous`（複数候補）、`insufficient`（判定保留）。
各候補は`model_id, label, model_note, dimensions_m, yaw_deg, translation, match_ratio, support_ratio, unmatched_ratio, inlier_rms_m, compatibility, rank_score, num_inliers, num_observed`。
描画用に変換済みXYZの平坦配列`matched_positions, unmatched_positions, outlier_positions`と全体の`min_position, max_position`を含む。Frontendでの点単位処理なし。
一致ノードなしの`inlier_rms_m`はnull。入力・仮定・指標の定義は[車両照合仕様](VEHICLE_REGISTRATION.md)。
エラーは`INVALID_PARAMS, BUSY, REGISTRATION_FAILED`。`job.progress/completed/failed`は要求idを`jobId`、`sessionId: "vehicle"`として通知。
完了イベントには`durationMs, message`、結果本体はRPC応答。編集点群の`publishedTopic`なし。

## Notes

旧RPC名は非対応。編集領域は`map`座標系、TF欠落時は`job.failed`。既定出力は`<sourceTopic>/edited`。
