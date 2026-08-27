# 2026-08-27 - TopoFuzzy marker overlay priority

## Summary

TopoFuzzy ViewerのMarkerArray表示を、点群、voxel、graph、ロボットより常に手前へ描画するよう変更した。

## Changed

- cube、sphere、cylinder、line、list、arrowの全マーカーを最終の透明描画キューへ統一した。
- マーカーの`renderOrder`を1000へ上げ、深度テストと深度書込みを無効にした。
- 共用の`DirectionalArrow`は、マーカーから指定された場合だけ最前面描画を有効にする。

## Behavior Impact

- 他の形状と重なったマーカーは遮蔽されず、常に確認できる。
- マーカー同士は深度で隠し合わず、描画順に重ねて表示される。
- GNG graphで使用する矢印の既存の深度描画は変更しない。

## Topics / Params / Messages

- ROS topic、parameter、message型の変更なし。
- Viewer設定の追加なし。MarkerArrayは既定で最前面表示になる。

## Verification

- frontendコンテナ内の`npm run lint`: 成功。
- frontendコンテナ内の`npm run build`: TypeScriptおよびVite production build成功。
- 稼働中のViewerへChromeで接続し、1440x900と390x844の両viewportで
  100,000点の点群と矢印マーカーを同時描画できることを確認した。

## Risk / Notes

- マーカーは遮蔽物の裏側にある場合も透過して見える。これは位置確認を最優先するための仕様である。
