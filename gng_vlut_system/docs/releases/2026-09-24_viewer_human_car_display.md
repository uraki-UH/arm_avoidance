# 2026-09-24 - Viewerへの人・車の確定分類表示

## 1. 要約

`/home/uraki/uraki_ws/ToPoFuzzy-Viewer`で環境GNGの確定分類を所属ノードの色へ反映。レイヤー内の件数とクラスタ形状の切替を追加。

- 確定クラスタの`label`が4ならHuman、5ならCarとして所属ノードを表示。元のノードの幾何ラベル・座標・エッジは保持。
- Humanは赤紫`#d946ef`、Carは青紫`#8b5cf6`。分類解除後は従来のノード色へ復帰。
- Human／Carの表示チェックも所属ノードへ適用。有効な境界・把持属性との既存OR表示と色優先順位は維持。

- レイヤー欄に確定クラスタ件数`Human: N / Car: N`。
- `Clusters`切替。ONで既存の人の円柱・車のボックスを表示。
- 実際のGraphRendererと受信デコーダーを使う回帰テスト`frontend/tests/human_car_display.test.mjs`。

環境GNGの所属配列添字とViewer内のノードIDの取り違え。受信時に`/topological_map`だけ添字からIDへ正規化。元からID方式の非平面・テンプレートGraphは変更対象外。バイナリversion 1/2と互換JSONの両経路へ適用。

## 2. 条件・検証

- Streamsで`/topological_map`を選択し、受信した確定分類を表示。トピックの自動選択なし。
- `Clusters`の既定OFFと既存設定を保持。所属ノードの分類色は形状表示のON/OFFとは独立。
- 単色・目標・境界・把持・非平面成分等の既存色指定を維持。元の`nodes[].label`は分類値で上書きしない。
- Frontendの更新とブラウザ再読み込みで適用。今回分のROS再ビルド・ノード再起動は不要。

追加・削除なし。ROSメッセージ、WSバージョン、配信バイト列の変更なし。受信後の`GraphData.clusters[].nodeIds`と検査用スナップショットはノードID方式。

[Graph Stream仕様](../../../ToPoFuzzy-Viewer/common/ws_protocol_v2.md#graph-stream)と[Backend API](../../../ToPoFuzzy-Viewer/doc/BACKEND_API.md#streamgraph)を参照。

Frontendディレクトリで実行:

```bash
timeout 90 npm run lint
timeout 90 node --test tests/human_car_display.test.mjs tests/cluster_graph.test.mjs tests/l0_state_colors.test.mjs tests/marker_array_renderer.test.mjs tests/boundary_evidence.test.mjs tests/boundary_label_modal.test.mjs tests/stream_restart.test.mjs tests/graph_topic_names.test.mjs tests/cluster_detail_panel.test.mjs tests/inspection_bbox_gate.test.mjs
```

lintと13テストが成功。新規テストは修正前に所属の不一致で失敗し、修正後に成功。添字とIDが数値的に重なる入力、無効添字、他GraphのID維持、色・確定解除・分類切替・非表示・単色・把持属性優先・形状切替を確認。実描画コンポーネントと代替GPUレンダラーによる検証であり、実GPUのスクリーンショット確認ではない。

ワークスペースルートで実行:

```bash
docker compose exec -T frontend sh -c 'pwd && npm run build'
```

TypeScript／Vite本番ビルド成功。チャンクサイズ警告のみ。起動中のViteのHTTP配信に、ノード分類処理・Human／Car件数・Clusters切替を確認。

Backendは既存の稼働環境を上書きしない一時出力先でビルド:

```bash
docker exec gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws/src/ToPoFuzzy-Viewer/backend && MAKEFLAGS=-j2 timeout 360 colcon --log-base /tmp/viewer-human-car-check-JRNlR9/log build --packages-select topo_fuzzy_viewer --symlink-install --build-base /tmp/viewer-human-car-check-JRNlR9/build --install-base /tmp/viewer-human-car-check-JRNlR9/install --cmake-args -DCMAKE_BUILD_TYPE=Release'
docker exec gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 60 ctest --test-dir /tmp/viewer-human-car-check-JRNlR9/build/topo_fuzzy_viewer --output-on-failure'
docker exec gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 30 ctest --test-dir /tmp/viewer-human-car-check-JRNlR9/build/topo_fuzzy_viewer -R "^test_(pointcloud_sampling|graph_inspection|nonplane_graph)$" --output-on-failure'
```

ビルド成功、既存ソース・外部依存の警告あり。機能テスト3/3成功。追加の全CTestは5/11成功、未変更Backendのcopyright・cpplint・flake8・lint_cmake・pep257・uncrustifyが失敗。全体チェック成功としては扱わず、今回の変更に無関係な一括整形は未実施。

**制約**

- 入力済みの確定結果の可視化であり、分類器の精度向上や実データの正解率の検証ではない。実ROSから実ブラウザ画面までの目視確認は未実施。
- 添字の正規化対象はトピック名が完全一致の`/topological_map`。任意の別名への自動適用なし。IDと添字の数値だけによる推測なし。
- 所属ラベルの索引はGraph更新時に1回構築。各ノードから全クラスタを走査する処理なし。
- 既存ROS・bag・Webサーバーへの停止／再起動操作なし。今回の検証プロセスは有限時間で全終了。
- 終了後に専用一時ビルド領域`/tmp/viewer-human-car-check-JRNlR9`（2.1GB）を削除。ソースから再生成可能。コンテナ状態と既存Viewer・bag・ROSデーモン・ViteのPID維持、検証子プロセスの残存なしを確認。
