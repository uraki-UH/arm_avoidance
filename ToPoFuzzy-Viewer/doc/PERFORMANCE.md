# 描画性能

評価対象: ROS 2 LiDAR点群、約16万点/フレーム・10 Hz。
この文書には測定機材・生ログ・再現条件を揃えたFPS／遅延／GPUメモリの実測結果なし。
従来記載の30〜60 FPS・転送10 ms未満は目標値であり、達成確認済みの値ではない。

描画実装: [PointCloudRenderer](../frontend/src/features/visualization/PointCloudRenderer.tsx)。
点群はThree.js Points、Graphは[GraphRenderer](../frontend/src/features/visualization/GraphRenderer.tsx)。
表示点数制限・中間フレームの扱いは[WS v2](../common/ws_protocol_v2.md)。

測定時はGPU・ブラウザ・描画面積・入力点数／Hz・表示レイヤーを固定し、受信Hz・描画FPS・操作遅延を区別。
バックエンドの点群削減と描画のみの設定変更も別条件として比較。
500k点60 FPS・5M点上限などの容量保証は未検証。
