# 2026-09-14 - 把持候補矢印の表示数制限

## Summary
把持候補の矢印設定へ「上位N件表示（0: 全件）」スライダーを追加。

## Changed
既存Markerレイヤー設定に`max_visible_candidates`を保持。初期値0、候補の受信順で描画数を制限。並べ替えや新しい評価式なし。

## Added
既存描画テストへ表示数変更、全件復帰、空入力、候補数超過、通常Markerへの非適用の確認を追加。

## Fixed
色・座標系ごとの描画グループ化より前に上位候補を選択。入力停止中の設定変更も再描画へ反映。

## Removed
なし。

## Behavior Impact
表示上限のみの変更。`/grasp_pose_cands/Tmap`の対象ノードグラフやホバー枠、ROS配信数、選定・計画には非適用。設定は現在のViewerセッション内で保持し、リロード時は全件へ復帰。

## Topics / Params / Messages
ROS側の変更なし。トピック名ではなく既存の候補用共有スタイルで識別するため、候補トピックの名前変更にも対応。

## Verification
frontendディレクトリで以下に成功、すべて終了済み。実THREE・React Three Fiberと模擬rendererで入力停止中の件数変更を確認。ROS・サーバーの起動なし。

```bash
timeout 60s npm run test:markers
timeout 120s npm run lint
timeout 120s ./node_modules/.bin/tsc -p tsconfig.app.json --noEmit --incremental false
```

## Risk / Notes
受信順であり、スコア順位への再評価ではない。実ブラウザでのスライダー操作・GPU描画は未検証。
