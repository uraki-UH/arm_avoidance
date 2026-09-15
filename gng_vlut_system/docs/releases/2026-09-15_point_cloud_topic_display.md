# 2026-09-15 - 点群トピック別の表示設定

## Summary

Viewerの既存Display操作を共通／トピック別で切替可能。[利用方法](../../../ToPoFuzzy-Viewer/README.md#点群トピック別の表示設定)を記載。

## Added

Displayタブへ対象トピック選択と「共通設定に戻す」を追加。点サイズ、不透明度、RGB／単色／Heatmap、単色、Heatmap配色・範囲を個別設定。

## Behavior Impact

- 未設定トピックは従来どおり共通設定。個別変更したトピックは設定一式を保持し、共通設定変更の対象外。
- 点群バッファとは別のトピックID別stateで保持。配信停止・再開で設定を維持し、明示削除・ブラウザリロードで解除。
- 個別の表示変更では点群の再処理・再転送なし。既存の点群受信処理やTF変換は変更なし。
- 個別Heatmapでは全点群のboundsを自動適用せず、範囲の設定値を使用。

## Topics / Params / Messages

ROSトピック・パラメータ・メッセージ変更なし。GNG・ROS送信・点群の実データには影響なし。

## Verification

frontendディレクトリで実行:

```bash
node --test tests/point_cloud_display.test.mjs tests/stream_restart.test.mjs
npm run lint
npm run build -- --configLoader runner --outDir /tmp/topofuzzy-point-cloud-display-build
```

2テストファイル、lint、本番ビルドに成功。新規テストでは実Appの設定更新処理とR3Fの2点群Rendererを使用し、個別の色・サイズ・不透明度0・Heatmap範囲・共通設定変更からの独立・共通への復帰を確認。表示変更前後のgeometryとposition attributeのversion一致で、頂点バッファ再転送がないことを確認。既存ストリーム復帰テストも成功。

初回テストのJSX変換設定とテスト用scene取得を修正後に成功。初回本番ビルドは既存`node_modules/.vite-temp`への書込権限エラーで失敗し、所有権変更をせずconfigLoader runnerで成功。Viteの既存大容量chunk警告は残存。

検証コマンドはすべて終了済み。テストの一時bundle・本番検証出力は削除。ROSノード・サーバーの新規起動や既存プロセスの再起動なし。

## Risk / Notes

ブラウザの実GUI操作と実GPU描画は未検証。永続保存は未実装。
