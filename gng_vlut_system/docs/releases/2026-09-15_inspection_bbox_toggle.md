# 2026-09-15 - 独立ビューのBbox切替

## 1. 要約

独立3Dビューの「XYZ軸」の隣に、既定OFFの「Bbox」チェックボックスを追加。

## 2. 条件・検証

受信済みの最小・最大座標による黄色のAABB線枠。寸法はフッターのXYZ寸法と一致し、ノード半径の余白なし。「最新を取得」で枠の範囲も更新。閉じて開き直すとOFF、永続保存なし。主画面の枠・選択設定・XYZ軸とは非連動。

変更なし。追加購読・RPC・点群再走査なし。

`ToPoFuzzy-Viewer/frontend`で下記の有限コマンドに成功、実行終了済み。

```bash
timeout 60s node tests/cluster_detail_panel.test.mjs
timeout 120s npm run lint
timeout 120s npm run build -- --configLoader runner --outDir /tmp/codex-cluster-bbox-build.NZDY4n
```

Bboxの既定OFFと既存の情報表示をHTML出力で確認。一時出力を削除、検証プロセスの残留なし。ROS・開発サーバーへの起動停止操作なし。

**制約**

ブラウザ上での切替・線枠描画は未検証。ビルドには既存の大きなchunk警告あり。
