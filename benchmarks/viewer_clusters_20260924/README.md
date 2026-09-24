# Viewer Clusters描画の比較

## 1. 要約

クラスタごとのメッシュ・材質・イベント登録を、形状と選択状態ごとのインスタンス描画へ変更。速度矢印も全クラスタで一括描画。

| クラスタ数 | 速度矢印 | 更新CPU ms 前→後 | WebGL要求数 前→後 |
| --- | --- | --- | --- |
| 100 | OFF | 2.1 → 0.3 | 200 → 4 |
| 100 | ON | 4.3 → 0.4 | 312 → 6 |
| 1,000 | OFF | 15.4 → 0.7 | 2,000 → 4 |
| 1,000 | ON | 38.9 → 1.4 | 3,142 → 6 |

## 2. 条件・検証

- Chrome headless・SwiftShader、400×400、合成クラスタを非重複配置。ノード・エッジOFF、選択無効。10回ウォームアップ後の30更新の中央値。更新CPUはReact反映と行列・色の更新、グラフ生成とWebGL描画は別区間。
- 変更前はcommit `51c154c2df675601242547b0eed3ac72660be0ed`のGraphRenderer、変更後は作業ツリー。数値は[summary.json](summary.json)。ソフトウェア描画のため実GPUや交差点全体のFPSへの換算不可。
- 分離配置のRGBAピクセルは4条件すべて一致。半透明メッシュが互いに重なる部分は、形状ごとの描画順への変更により混色が変化する可能性あり。
- 回帰9件で位置・回転・寸法・人と車の色・選択解除・ドラッグ除外・表示切替・容量再利用・速度矢印行列を確認。lintとFrontend本番ビルド成功。
- 初回ブラウザ試験の他描画イベント混入とカメラ自動拡大を修正し、空画像の測定値は不採用。表は修正後の測定のみ。

再現コマンド（`ToPoFuzzy-Viewer/frontend`）:

```bash
node --test tests/cluster_batch.test.mjs tests/human_car_display.test.mjs tests/candidate_hover_frame.test.mjs tests/inspection_bbox_gate.test.mjs tests/marker_array_renderer.test.mjs
node tests/cluster_render_browser.test.mjs /tmp/cluster_render_20260924.json
npm run lint
```

ビルドはリポジトリルートで`docker compose exec -T frontend npm run build`。ホスト側ビルドは既存`.vite-temp`の所有権で失敗したため、通常コンテナ内で成功を確認。
ブラウザ比較スクリプトが専用プロファイルでChromeを起動し、finallyでプロセスグループを停止。試験Chromeは全終了、既存のViewer・ROS・bagは停止・再起動なし。
