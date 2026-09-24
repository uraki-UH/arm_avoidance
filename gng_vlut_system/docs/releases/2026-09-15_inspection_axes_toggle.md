# 2026-09-15 - 候補独立ビューのXYZ軸切替

## 1. 要約

バウンディングボックスから取り出した独立3DビューのXYZ軸を既定非表示へ変更。

操作欄に「XYZ軸」チェックボックスを追加。主画面の軸、候補選択、XYZ寸法の数値表示は変更なし。

## 2. 条件・検証

独立ビュー内だけの表示切替。閉じて開き直した際はOFF。設定の永続保存なし。

`ToPoFuzzy-Viewer/frontend`で下記の有限コマンドに成功、実行終了済み。

```bash
timeout 120s npm run lint
timeout 120s npm run build -- --configLoader runner --outDir /tmp/codex-cluster-axes-build.g3UvUJ
```

一時ビルド出力は削除済み。ROS・開発サーバーの起動や停止なし。

**制約**

ブラウザでの実クリック・描画確認は未実施。ビルドには既存の大きなchunk警告あり。
