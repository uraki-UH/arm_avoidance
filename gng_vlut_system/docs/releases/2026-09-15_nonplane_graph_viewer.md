# 2026-09-15 - 非平面成分のViewer Graph表示

## Summary

ROS側の所属配列を維持したまま、Viewer向け非平面表示をMarkerからGraphへ変更。

## Changed

- `/topological_map`・`/plane_clusters`・`/nonplane_components`の同一フレーム照合によるバックエンド変換。各入力の到着時に最新データを再照合。
- 元ノードID、法線、勝者入力共分散、ラベル、境界属性を既存TMG1形式で配信。成分ごとの色はGraph共通実装。
- 成分内エッジと平面への接続エッジを保持。平面側端点は灰色の未所属ノードとして表示し、非平面成分のBbox・独立ビューには含めない。

## Added

- `/nonplane_components`に既定OFFのBounding Box切替。ONで成分の独立ビューを選択可能。他Graphの未指定設定と把持候補Tmapの既定ONを維持。
- 属性・ID・エッジ・範囲のC++テスト、隔離ROSとWebSocketによる配信テスト。

## Fixed

- 所属配列が先に届いた場合にも、残りの入力到着時に同一フレームを再照合。
- 共通所属色テストの一時出力を、書込み不可のnode_modules配下からtests配下へ移動。

## Removed

- 非平面専用Marker JSON生成。ROSでのTopologicalMap二重publishは追加なし。

## Behavior Impact

- 通常Graphの描画完了通知による配信制御、法線・共分散・ノード/エッジ表示設定を共用。
- 空成分は空Graphで表示を消去。全クライアント切断時は購読を解放する既存仕様を維持、再接続後は再購読。
- 旧Markerとの色・寸法の完全互換なし。独立ビューは所属ノードと内部エッジのみ、主画面は平面接続エッジも表示。

## Topics / Params / Messages

- ROS `/nonplane_components`: `std_msgs/msg/UInt32MultiArray`、変更なし。
- WSのsource型: `nonplane_component`、tag: `/nonplane_components`、配信: 既存`TMG1`。
- ROS launch引数・パラメータ・メッセージ定義の変更なし。
- 詳細仕様: [BACKEND_API.md](../../../ToPoFuzzy-Viewer/doc/BACKEND_API.md#非平面成分のgraph表示)。

## Verification

Dockerでの実行コマンド:

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 360s colcon build --packages-select topo_fuzzy_viewer --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON && timeout -s INT -k 5s 120s ctest --test-dir /ros2_ws/build/topo_fuzzy_viewer --output-on-failure -R "^(test_nonplane_graph|test_graph_inspection)$"'
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 90s python3 /ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/test_nonplane_stream.py'
```

配信テスト内部の起動コマンド:

```bash
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 /ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node --ros-args -p port:=19092
```

同じPythonプロセス内に`nonplane_stream_test`を作成。入力の全6到着順、元ID、所属、実エッジ、Marker非配信、再接続・再購読、空成分、購読解除を確認。スクリプトのfinallyでテストノードと専用gatewayを終了。

frontendディレクトリでの実行コマンド:

```bash
node --test tests/cluster_graph.test.mjs tests/inspection_bbox_gate.test.mjs tests/candidate_hover_frame.test.mjs tests/cluster_detail_panel.test.mjs tests/boundary_evidence.test.mjs
npm run lint
npm run build -- --configLoader runner --outDir /tmp/nonplane-graph-build-0915
```

backendビルド、CTest 2対象、frontend 5ファイル・lint・本番ビルドに成功。配信テスト初回の端点数期待値、再接続時の再購読漏れ、frontendの旧Bbox期待値を修正後に成功。既存のコンパイル警告とViteのchunkサイズ警告あり。

## Risk / Notes

- 最新入力だけの照合であり、入力の欠落・フレーム跨ぎの到着順によって表示更新を飛ばす場合あり。不一致フレームの合成・全履歴保持なし。
- 成分IDはフレーム内の所属情報であり、永続物体IDとしての追跡保証なし。
- バックエンドを再起動し、ページを再読み込みしてStreamsの`/nonplane_components`を選択。稼働中Viewerへの再起動操作は未実施。
- 実ブラウザ/WebGLの目視操作と実GNG入力の描画確認は未実施。配信検証は合成データ。
- テストプロセス・一時出力の終了と削除を確認。既存Viewer・ロボット関連PIDは維持。CPU GNGのPID更新は観測したが、本作業からの停止・再起動操作なし。
