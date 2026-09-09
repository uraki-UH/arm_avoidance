# 2026-09-09 - GNG境界候補のノード属性化

## Summary

CPU版GNGでの低次数境界候補判定と、既存 `/topological_map` 内のノード属性による配信。
Viewerでは受信属性による既存ノードの色分け。専用トピック、次数再集計、別オーバーレイは不要。

## Changed

- `TopologicalNode.msg` に `bool is_boundary_candidate` を追加。
- `graspnet.yaml` の `boundary.enable_candidates: true`、`boundary.max_neighbors: 4` による起動時設定。
- 判定式は `gng_get_node_num_neighbors(node.id) <= max_boundary_neighbors_`。
- GNG要約ログ末尾に `Boundary: 0.00 ms (45)` 形式の判定時間と候補数を追加。
- Viewerの専用オーバーレイを撤回し、「Labels → Semantic labels」内の「境界候補」で色分けを切り替え。
  表示UIまで削除した変更の訂正。`BoundaryNodeOverlay.tsx` の再追加なし。

## Added

- `gng_get_node_num_neighbors(node_id)`：内部 `Node::edge_num` の定数時間参照。
  無効ノードの返値は `UINT32_MAX`。
- ROS属性からWebSocketバイナリへの転送と、フロントエンドでの復元。
- グラフ差分判定への候補フラグ追加。座標・時刻が同じ場合のフラグ単独更新にも対応。
- API検査、隔離ROS・WebSocket受信検査、境界属性の復元・更新検査。

## Fixed

GNG内の判定結果をグラフ本体に格納。別トピックとのフレーム対応付けが不要な属性配信。
並行作業で追加されたグラフバイナリ転送を保持し、その送受信経路にも属性を追加。

## Removed

直前の試作に含まれた以下の専用出力・設定を廃止。

- `/boundary_candidates`（候補添字列）。
- `/boundary_candidates/markers`（描画用マーカー）。
- `boundary.marker_size`（専用マーカーの廃止に伴う設定削除）。
- `BoundaryNodeOverlay.tsx` とGraphRenderer内の呼び出し。
- 独立した境界候補ボタン・候補数表示・境界候補専用枠。
  表示切り替えはLabels内のSemantic labelsへ統合、HANDLEと同列の複数選択項目。
- 専用オーバーレイの描画テスト。属性の受信・更新テストは維持。

## Behavior Impact

- GNG学習則、ノード座標、接続、ラベル、平面クラスタ抽出に変更なし。
- 候補判定は入力点群処理時のみ。入力停止中の追加学習タイマーなし。
- 判定対象は学習グラフ全体の隣接数。平面クラスタへの所属に非依存。
- GNG側判定はノード数に比例。辺の再走査、法線計算、角度計算なし。
- Viewerの「Labels → Semantic labels → 境界候補」は既定ON。`is_boundary_candidate` がtrueの既存ノードをオレンジ色で表示。
  HANDLEと境界候補は同時ONが可能。それぞれの操作で他方の設定は変更なし。
  OFF時は他の選択ラベルに従う表示・色分け。semantic・goalノードにも適用。
  Semantic labelsの上下ボタンで色の優先順位を変更可能。既定値は従来どおり境界候補が先頭。
  `node_label_priority` をレイヤーごとに保持。旧 `overlap_label_priority` は読み込み互換のみ。
  定義一覧によるGUI・描画の共通化と追加手順は [ラベル拡張ノート](2026-09-09_viewer_node_label_registry.md) を参照。
  優先側がOFFならONの側を表示。優先度変更による可視ノード数・OR判定への変更なし。
  既存InstancedMesh内の色更新のみで、ノード重複・メッシュ追加・追加の次数計算なし。
  表示判定は「通常ラベルON または HANDLE選択かつ該当 または 境界候補選択かつ該当」。
  通常ラベル全OFFでもHANDLEのみ・境界候補のみの表示が可能。重複該当ノードは一度だけ描画。
  通常・goalノードで共通の判定。静的グラフにも通常ラベル設定を伝達。
  レイヤーとNodesの表示スイッチは全体のON/OFFとして維持。TF・手動変換も既存経路を使用。
  色分けON時はマテリアルを白に設定し、発光色にもインスタンス色を適用。
  既存6分類のラベル数・All/None操作は維持。境界候補は分類追加ではなく独立した色分け属性。
- GPU版での候補判定は対象外。未設定のフラグはfalse。
- 通常の実行先を上書きしない隔離ビルドでの検証。実行中Viewerへの反映は未実施。

## Topics / Params / Messages

追加トピック・追加launch引数なし。既存 `/topological_map` の `nodes[].is_boundary_candidate` を使用。

| YAML設定 | 宣言時既定値 | graspnet.yaml | 用途 |
| --- | --- | --- | --- |
| `boundary.enable_candidates` | false | true | 候補判定の有効化 |
| `boundary.max_neighbors` | 4 | 4 | 候補の隣接数上限 |

機能OFF時は全ノードfalse。隣接数0も上限内なら候補。

WebSocketの `TMG1` バイナリでは、84バイトのノードレコード内オフセット5に0/1を格納。
既存予約領域の利用によるレコード長の維持。JSON互換表現では `is_boundary_candidate`。
古いバイナリの予約領域0はfalse、JSON属性欠落時は未設定として保持。

ROSメッセージ定義は変更済み。送信側・受信側を同一定義で再ビルドし、再起動が必要。
`TopologicalMap` 利用パッケージや、そのノードを埋め込んだ別メッセージの利用側も反映対象。
古いROS定義との混在運用は不可。

## Verification

コンテナ `gng_cpu_container` 内の隔離ビルド先：`/tmp/gng-boundary-field.GoijXn`。
通常の `/ros2_ws/install` の上書きなし。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
colcon --log-base /tmp/gng-boundary-field.GoijXn/log build \
  --build-base /tmp/gng-boundary-field.GoijXn/build \
  --install-base /tmp/gng-boundary-field.GoijXn/install \
  --packages-select ais_gng_msgs gng_cpu ais_gng topo_fuzzy_viewer \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DGNG_BUILD_BENCHMARKS=ON -DBUILD_TESTING=OFF
```

- 4パッケージのビルド成功。既存コード由来のコンパイル警告あり。
- `timeout 30s /tmp/gng-boundary-field.GoijXn/build/gng_cpu/gng_neighbors_api_test`：成功。
  学習20フレームで次数APIと出力グラフの次数が一致。
- フロントエンド `npm run lint`：成功。
- `npm run build -- --configLoader runner --outDir /tmp/topo-boundary-field-dist`：成功。
- `timeout 70s node tests/boundary_attributes.test.mjs`：成功。
  バイナリ復元、フラグ単独更新、Labels内のGUI既定値・切り替え通知、dynamic/staticの既存ノード色更新を検証。
  メインのグラフ設定に独立した境界候補ボタンがないことも確認。
  semantic・goalノード、ラベル非表示、Nodes OFF、空グラフ、追加メッシュ・重複ノードなしを確認。
  GPUを模擬したコンポーネント検査であり、実ブラウザでの描画検査とは別。
- GUI復旧後のlint・ビルドは成功。ビルド出力先は `/tmp/topo-boundary-gui.tzNRpd`。
- Labels内への移動後もlint・ビルド・GUI切替テストは成功、全検証プロセスは終了済み。
  ビルド出力先は `/tmp/topo-boundary-labels.z1jhHD`。既存ROS・Viewerの再起動なし。
- Semantic labelsへの統合後も同じlint・ビルド・テストは成功。
  同一枠内のHANDLE・境界候補と、ON/OFFの全組み合わせ・独立した更新を確認。
  ビルド出力先は `/tmp/topo-labels-group.Trjihh`。検証プロセスはすべて終了済み。
- OR表示の回帰テストは修正前に失敗、修正後に成功。動的・静的の双方で3選択の全8組み合わせを確認。
  重複ノード、goal、クラスタ由来HANDLEも検査。lint・ビルドも成功。
  ビルド出力先は `/tmp/topo-labels-or.HswZdz`。検証プロセスはすべて終了済み、既存ノードの再起動なし。
- 重複時の優先色について、GUI既定値・更新通知、色のみの更新、OFF側の除外、クラスタ由来HANDLE・goalの検査成功。
  動的・静的の双方で確認。lint・ビルドも成功、出力先は `/tmp/topo-labels-priority.WdGKuf`。
  検証プロセスはすべて終了済み。既存ROS・Viewerの再起動なし。

ROS・WebSocket検証の起動コマンド：

```bash
source /tmp/gng-boundary-field.GoijXn/install/local_setup.bash
export LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/torch/lib:$LD_LIBRARY_PATH
timeout --signal=INT --kill-after=15s 85s python3 \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/boundary_candidates_ros_test.py \
  --executable /tmp/gng-boundary-field.GoijXn/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --gateway-executable /tmp/gng-boundary-field.GoijXn/install/topo_fuzzy_viewer/lib/topo_fuzzy_viewer/viewer_ws_gateway_node
```

- ROSドメイン189、WebSocketポート19091による既存実行環境からの分離。
- 初回受信検査でViewer転送値の不一致を検出。変更途中でコンパイルされた送信部分の再コンパイル後、再検査成功。
- 上限4・上限0・機能OFFの3条件で、それぞれROS 10フレーム・WebSocket 3フレームの検査成功。
  ROS属性・バイナリ属性と実グラフ次数の一致、座標系の維持、専用トピック不在を確認。
- 入力停止後の学習出力停止を3条件で確認。
- 検証用GNG・gatewayはすべて停止・回収済み。終了後のプロセス一覧でも残存なし。
- 既存のノード・bag再生・Viewerの停止や再起動なし。
- 実ブラウザ画面の目視検証は未実施。

## Risk / Notes

- 低次数は境界の証明ではなく一次候補。高次数の境界を見逃す可能性あり。
- 遮蔽／真の境界／隙間の追加判定は未実装。
- 時間ログは属性判定と候補数集計のみ。描画・ROS配信時間は含まず、処理時間の上限保証でもない。
- 445ノード・71候補でのログ例は `Boundary: 0.00 ms (71)`。小数第2位丸めの表示であり、処理時間ゼロという意味ではない。
- 再ビルド時に変更中のヘッダーを読み込む可能性への注意。変更完了後の再コンパイルと送受信検査が必要。
