# 2026-09-09 - 境界ラベルの折りたたみと原因別設定

## Summary

重複ラベルの境界関連5項目を「境界候補」1項目へ整理。

## Changed

- 見出しを「Semantic labels」から「重複ラベル」へ変更。「複数選択可・上ほど色を優先」は見出しの横へ配置、狭い画面のみ折り返し。
- 一覧は「境界候補」「HANDLE」。優先順位は長押しドラッグと「先頭へ」で変更。詳細は `2026-09-09_viewer_label_priority_drag.md` を参照。
- 原因別の設定は「原因別の表示・色」内。初期状態は折りたたみ。
- 展開行は全幅・高さ44pxを確保。ホバー・キーボードフォーカス表示と開閉矢印を追加。
- 親OFF時は原因別表示も一括OFF。子の選択状態は維持。
- 自由空間の証拠の既定色は青（`#2196f3`）。GUIでの個別色指定は維持。

## Added

- 視野端・遮蔽の証拠・自由空間の証拠・原因不明の表示フィルタと色選択。
- 定義の `parent_id` による親子分類、LayerSettingsの `node_label_colors`。
- 親子のON/OFF、色、優先順位、実UIの構造とイベントの回帰テスト。

## Fixed

- 原因別フィルタOFF時の、全候補表示への迂回による色分けの残留を解消。

## Removed

- 原因別ラベルの一覧への独立表示のみ。受信属性・判定処理の削除なし。

## Behavior Impact

- 新規設定の原因別フィルタは全ON。既存の各ON/OFF・原因別相対順は保持。
- 旧設定でHANDLEを子ラベル間へ配置していた場合、親の「境界候補」とHANDLEの順に統合。子ラベルによる親OFFの上書きは廃止。
- 原因別の複数証拠はOR表示。複数一致の色は詳細欄の上の項目を優先。
- 原因別項目をすべてOFFにした場合、境界としての強調なし。通常ラベルやHANDLEによる独立した表示は維持。
- 今後の原因追加はラベル定義への `parent_id` 付き項目追加で対応。

## Topics / Params / Messages

ROSトピック・パラメータ・メッセージ・WebSocket形式の変更なし。

## Verification

- `node tests/boundary_evidence.test.mjs` 成功。
- `node tests/boundary_label_modal.test.mjs` 成功。Portalのみを置換した実コンポーネントの構造・イベント検査。実ブラウザ操作の検査ではない。
- `npm run lint` 成功。
- ホスト側の通常出力先ビルドは既存distへの書込権限不足。`npm run build -- --configLoader runner --outDir /tmp/topo-boundary-labels.ISHTxq` は成功。
- 次のコンテナ内ビルドも成功、通常の配信用distを更新。既存の大きなバンドル警告あり。

```bash
docker exec gng_cpu_container bash -lc '
cd /ros2_ws/src/ToPoFuzzy-Viewer/frontend
timeout --signal=INT --kill-after=5s 90s npm run build -- --configLoader runner
'
```

ビルド・単体テストのプロセスは終了済み。検証用サーバー・ROSノードの起動なし。
既存Viewerやブラウザの再起動なし。表示の反映にはブラウザの再読み込みが必要。

## Risk / Notes

- GNG・バックエンドは今回未変更。
- 既存の手編集・並行作業の変更は保持。通常ラベルのAll/Noneの意味は変更なし。
