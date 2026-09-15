# 2026-09-15 - 候補独立ビューの情報整理

## Summary

フッターの情報を1行に集約し、非平面候補の表示名を短縮。

## Changed / Removed

- ノード数・エッジ数・XYZ寸法を同じ行へ配置。幅不足時は横スクロール。
- 「座標系: ... / 元シーン・TFの変更なし」の説明行を削除。
- 独立ビューの`nonplane_components #7`を`nonplane_7`へ短縮。source表示も同じ名称で短縮、ツールチップは元名を維持。

## Behavior Impact / Topics / Params / Messages

表示だけの変更。ROSトピック名、選択ID、RPCの再取得先、主画面の表示は変更なし。

## Verification

`ToPoFuzzy-Viewer/frontend`で下記の有限コマンドに成功、実行終了済み。

```bash
timeout 60s node --test tests/cluster_detail_panel.test.mjs
timeout 60s node tests/cluster_detail_panel.test.mjs
timeout 120s npm run lint
timeout 120s npm run build -- --configLoader runner --outDir /tmp/codex-cluster-info-build.F86o2R
```

実パネルのHTML出力で短縮名・単一行・説明削除・元source不変・他名称不変・XYZ軸の既定OFFを確認。一時出力は削除済み、ROS・開発サーバーへの起動停止操作なし。

## Risk / Notes

テストではCanvasのみ省略。ブラウザの実クリック・描画確認は未実施。ビルドには既存の大きなchunk警告あり。
