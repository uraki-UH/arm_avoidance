# 2026-09-15 - 独立表示のBbox連動

## Summary

主画面のBbox OFFでもノード・Markerの直接クリックで独立ビューを開けた状態を修正。ホバー枠の経路だけに適用されていたフラグを、直接選択・詳細取得にも適用。

## Changed

- ノード・クラスタ・Markerの直接選択を元トピックの`enable_bounding_box=true`に限定。
- RPC開始時と応答時のフラグを確認。OFF中の送信と、取得待機中のOFF後の表示・エラーを抑止。
- `/grasp_pose_cands/Tmap`は既定ON。他トピックは既定OFF。ユーザーが選んだOFFは保持。

## Behavior Impact

- ノード・エッジ自体の表示は不変。可操作性楕円体の詳細表示は別機能として維持。
- 既に開いた独立ビューは固定表示を維持。最新取得には元トピックのBbox ONが必要。
- 独立ビュー内の「Bbox」は従来どおり別の表示設定で既定OFF。
- 更新後の新規読み込みで既定値を適用。既存タブのOFFを強制変更しない。

## Topics / Params / Messages

ROS・RPC形式・バックエンド処理への変更なし。[現行仕様](../../../ToPoFuzzy-Viewer/doc/BACKEND_API.md#候補の独立表示)を更新。

## Verification

frontendディレクトリでの有限検証コマンド:

```bash
node --test tests/inspection_bbox_gate.test.mjs tests/candidate_hover_frame.test.mjs tests/boundary_evidence.test.mjs tests/cluster_detail_panel.test.mjs tests/marker_array_renderer.test.mjs
npm run lint
npm run build -- --configLoader runner --outDir /tmp/topofuzzy_bbox_gate_dist
```

バックエンドの必須ビルド確認:

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 360s colcon build --packages-select topo_fuzzy_viewer --symlink-install --parallel-workers 1'
```

Appの実接続式・非同期コールバックのテストで、未指定/OFF/ON、編集モード、ノード・クラスタ・Marker選択、取得中OFF後の成功・失敗応答、再ONを確認。ホバーと既存Marker描画、既定値・明示OFF、独立ビュー内Bbox既定OFFも検証。

## Risk / Notes

実ブラウザ操作とGPU描画は未検証。新規ROSノード・開発サーバーの起動なし。
