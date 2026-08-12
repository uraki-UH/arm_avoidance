# 2026-08-12 - Viewer topic uncheck removes scene layer

## Summary

ToPoFuzzy ViewerのTopicsチェックをオフにしたとき、対応するScene Layerも削除するようにした。

## Changed

- Topicsのオフ操作で`sources.setActive`へ`removeLayer=true`を渡す。
- レイヤー削除時に、描画反映待ちの同一topicグラフ更新も破棄する。

## Added

- なし。

## Fixed

- ROS購読を停止してもScene Layersに最後の表示内容が残る問題を修正した。
- 削除直前に受信したグラフが次の描画フレームで復活する競合を防止した。

## Removed

- なし。

## Behavior Impact

- Topicsをオンからオフへ切り替えると、購読停止と対応Scene Layerの削除が同時に行われる。
- 再度オンにすると、新しく受信したデータからScene Layerが再作成される。

## Topics / Params / Messages

- ROS topic、parameter、message定義の変更はない。
- WebSocket RPCの既存任意引数`removeLayer`をTopics UIから使用するようにした。

## Verification

- Frontend lint成功。
- Docker内でFrontend production build成功。

## Risk / Notes

- Scene Layer内の表示設定は、topicを再度オンにしたとき既定値から再生成される場合がある。
