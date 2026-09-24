# 2026-08-12 - Viewer topic uncheck removes scene layer

## 1. 要約

ToPoFuzzy ViewerのTopicsチェックをオフにしたとき、対応するScene Layerも削除するようにした。

- Topicsのオフ操作で`sources.setActive`へ`removeLayer=true`を渡す。
- レイヤー削除時に、描画反映待ちの同一topicグラフ更新も破棄する。

- ROS購読を停止してもScene Layersに最後の表示内容が残る問題を修正した。
- 削除直前に受信したグラフが次の描画フレームで復活する競合を防止した。

## 2. 条件・検証

- Topicsをオンからオフへ切り替えると、購読停止と対応Scene Layerの削除が同時に行われる。
- 再度オンにすると、新しく受信したデータからScene Layerが再作成される。

- ROS topic、parameter、message定義の変更はない。
- WebSocket RPCの既存任意引数`removeLayer`をTopics UIから使用するようにした。

- Frontend lint成功。
- Docker内でFrontend production build成功。

**制約**

- Scene Layer内の表示設定は、topicを再度オンにしたとき既定値から再生成される場合がある。
