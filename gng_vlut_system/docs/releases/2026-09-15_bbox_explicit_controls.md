# 2026-09-15 - 明示指定トピックだけのBounding Box GUI

## 1. 要約

Graph設定`enable_bounding_box`の未指定と明示falseを区別。未指定トピックはBounding Box GUIなし、明示falseはGUIを残して再ON可能。既定の明示指定は`/grasp_pose_cands/Tmap`のtrueだけ。

## 2. 条件・検証

通常の`/topological_map`やロボットGNGにはGUI・枠判定・独立表示選択なし。トピック・ROSパラメータ変更なし。独立ビュー内のBboxは別設定として維持。既存タブの状態を強制変更せず、ページ再読込で新既定値を適用。

下記を`ToPoFuzzy-Viewer/frontend`で実行。GUI構造・OFFからの再ON、RPC抑止・遅延応答、ホバー、独立ビュー、lint・本番ビルドに成功。初回ホバーテストの旧false期待値による失敗を修正し、再検証成功。

```bash
timeout 60s node tests/inspection_bbox_gate.test.mjs
timeout 60s node tests/boundary_evidence.test.mjs
timeout 60s node tests/candidate_hover_frame.test.mjs
timeout 60s node tests/cluster_detail_panel.test.mjs
timeout 120s npm run lint
timeout 120s npm run build -- --configLoader runner --outDir /tmp/codex-grasp-bbox-build.LmLZpv
```

全コマンド終了、一時出力削除済み。ROS・開発サーバーへの起動停止操作なし。

**制約**

実ブラウザ操作は未検証。ビルドには既存の大きなchunk警告あり。
