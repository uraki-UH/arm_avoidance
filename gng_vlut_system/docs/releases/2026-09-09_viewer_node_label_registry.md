# 2026-09-09 - Viewerノード可視化ラベルの拡張用定義一覧

## Summary

HANDLE・境界候補の表示定義を `nodeLabelRegistry.ts` へ集約。
重複可能なラベルのGUI、OR表示、色の優先順位を共通化。
今回はViewer側のみ。遮蔽境界・自由空間側境界の判定やROS属性の追加は対象外。

## Changed

- `ToPoFuzzy-Viewer/frontend/src/features/visualization/nodeLabelRegistry.ts`：ID、表示名、色、既定ON/OFF、既定順位、受信属性の該当判定を定義。
- `GngLabelModal.tsx`：定義一覧からSemantic labels内のON/OFFと上下ボタンを生成。
- `GraphRenderer.tsx`：通常分類ONまたは有効な属性ラベル該当によるノード表示。該当する有効ラベルのうち、最優先の色を適用。
- 描画ユーティリティへ解決済みの色を渡す方式。ラベル追加時の専用描画分岐が不要。

## Added

- レイヤー設定 `node_label_visibility`：安定したラベルIDをキーとする表示状態。
- レイヤー設定 `node_label_priority`：先頭ほど優先度の高いラベルID配列。
- `tests/graph_labels.test.mjs` と `npm run test:labels`：第3ラベル追加、旧設定互換、GUI操作、動的・静的描画の検査。

## Fixed

- HANDLEと境界候補の2択に固定されない優先順位管理。
- 未知・重複した優先順位IDの除外、追加ラベルの既定順による補完。
- 旧設定 `enable_boundary_highlight`、`visibleSemanticLabels.handle`、`overlap_label_priority` の読み込み互換。新設定の明示値を優先。

## Removed

- ラベルごとの専用GUI・ノード描画分岐。専用オーバーレイの追加なし。
- ROSメッセージ・トピック・既存の境界判定機能の削除なし。

## Behavior Impact

- Labels → Semantic labelsの上下ボタンによる優先順位変更。上ほど優先、OFF項目は色決定から除外。
- 通常ラベルが全OFFでも有効な属性ラベルに該当するノードは表示。Nodes全体のOFFは引き続き優先。
- 重複ノードの多重描画なし。優先順位は表示色だけに影響し、受信属性やノード数への変更なし。
- 初期状態は境界候補・HANDLEともON、境界候補が最優先。
- 既存6分類のAll/Noneは通常分類のみが対象。

## Topics / Params / Messages

今回の追加・変更なし。既存の `semanticLabel` と `is_boundary_candidate` を参照。
ビットマスクへの移行なし、Viewer側での次数や幾何情報の再計算なし。

## Verification

`ToPoFuzzy-Viewer/frontend` で以下を実行、正常終了。

```bash
npm run test:labels
npm run lint
npm run build -- --configLoader runner --outDir /tmp/topo-label-registry.gSxgpZ
```

- テスト限定の第3ラベルで、定義追加だけによるGUI生成・単独表示・3項目の優先色変更を検証。
- 通常分類・HANDLE・境界候補の全8選択、goal、クラスタ由来HANDLE、明示的なsemanticLabel=0、Nodes OFF、空グラフを動的・静的双方で検証。
- 同一座標・時刻のままの色更新と重複描画の不在を検証。
- 実コンポーネントによる検査、GPU描画のみ代替。実ブラウザ画面の目視検証は未実施。
- ビルド時にバンドルサイズ警告あり。既存dist・ROSの実行環境への上書きなし。
- 既存markerテストの併行実行は一時出力先の権限不足により失敗。今回のラベルテストは書き込み可能なtests配下へ変更して成功、既存markerテストへの変更なし。
- 検証コマンドはすべて終了済み。ROS・Viewerの常駐プロセスの起動・停止・再起動なし。

## Risk / Notes

今後のラベル追加手順：

1. 既に受信可能な属性なら、`node_label_definitions` に一意なID、名前、色、既定設定、`is_match` を追加。
2. 判定対象は受信済みノード属性のみ。属性欠落・未判定を肯定判定として扱わない条件の指定。
3. 新しい受信属性が必要なら、GNG側の判定、ROSメッセージ、転送、復元、差分更新判定も別途拡張。定義追加だけで未配信情報の取得は不可。
4. 可視化の追加後に `npm run test:labels` とlint・ビルドで確認。

将来の遮蔽境界・自由空間側境界は、境界候補とは別の独立属性として追加可能。
Viewerの優先色は表示上の選択であり、観測の確信度・境界の種類の確定とは別。
今回のテスト用ラベルはテスト内だけの定義、実GUIへの追加なし。
