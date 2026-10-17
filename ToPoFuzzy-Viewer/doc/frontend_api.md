# Frontend構成

Frontendは表示と入力操作を担当。点群の削除・フィルタ・座標変換の計算はバックエンドへ依頼。
通信契約は[WS v2](../common/ws_protocol_v2.md)、全体構成は[Architecture](ARCHITECTURE.md)。

| 責務 | 実装 |
| --- | --- |
| 画面・状態の組立て | [App.tsx](../frontend/src/App.tsx) |
| パネル・タブ | [layout](../frontend/src/layout/) |
| 共通入力部品 | [SharedControls.tsx](../frontend/src/components/ui/SharedControls.tsx) |
| WS接続・受信・RPC | [useWebSocket.ts](../frontend/src/hooks/useWebSocket.ts) |
| 点群・Graph・ロボット描画 | [visualization](../frontend/src/features/visualization/) |
| 表示設定・ラベル優先順位 | [graphLayerSettings.ts](../frontend/src/features/visualization/graphLayerSettings.ts) |
| 矢印・楕円体の共通描画 | [SharedRenderers.tsx](../frontend/src/features/visualization/SharedRenderers.tsx) |
| 編集操作 | [manipulation](../frontend/src/features/manipulation/) |
| 入出力・ROS入力選択 | [io](../frontend/src/features/io/) |
| 監視・分析 | [analysis](../frontend/src/features/analysis/) |
| テンプレート照合UI | [templateMatching](../frontend/src/features/templateMatching/) |
| 車両位置合わせUI | [vehicleRegistration](../frontend/src/features/vehicleRegistration/) |
| 型・バイナリ復号 | [types](../frontend/src/types/)・[utils](../frontend/src/utils/) |

点群のRGBはsRGB入力からlinear作業色空間へ変換し、tone mappingなしで描画。
矢印の寸法・姿勢・基準位置は[共通仕様](../common/arrow_visual_spec.md)。
