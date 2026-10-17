# ToPoFuzzy Viewer & Editor

ROS 2の点群・Graph・ロボット姿勢を表示し、点群編集をバックエンドへ依頼するWeb Viewer。

## Getting Started

ROS側のビルド・起動は[ROS 2接続](doc/ROS2_BRIDGE.md)。Frontendは次のとおり。

```bash
cd ToPoFuzzy-Viewer/frontend
npm ci
npm run dev
```

表示: `http://localhost:5173`。接続先: `ws://127.0.0.1:9001`。
Docker・GPU起動は[ワークスペース手順](../RUN_GUIDE.md)。

表示ON/OFFとROS処理の開始・停止は独立。
点群の削除・変換はバックエンド担当。編集領域は`map`座標系、TF欠落時は失敗。

## 点群トピック別の表示設定

トピック別設定は未指定時に共通値へ追従し、値の変更時に個別化。解除で共通値へ復帰。
同名トピックの停止・復帰では保持し、明示的なレイヤー削除・リロードで解除。ROS入力・学習への影響なし。

## ロボット学習結果のレイヤー表示

同じ名前空間の`Tmap_static_L0`受信時は`Tmap_static`を初期非表示。L0・L1は表示を維持し、手動選択を優先。
補助ボクセルは初期非表示。Scene Layersの名前はROSトピック末尾。

Low polygonは形状の丸みだけを削減し、ノード・エッジの件数を維持。
確定Humanは赤紫、Carは青紫。Clustersの人円柱・車ボックスは初期OFFで、所属ノード色とは独立。属性色・単色指定を優先。

## ロボット候補のリンク別表示設定

Visualのリンク別設定は同じロボット項目の全候補へ反映し、全体設定より優先。
Collisionは全体設定を使用。candidateの色解除後はURDF色へ復帰。

## メッシュモデルの直接表示

メッシュ形式はOBJ・PLY・STL・GLB/glTF・FBX。[形式・操作・制限](doc/MESH_MODELS.md)。
URDFメッシュの更新は`robot_viewer_bridge_node`の再起動で反映。[共通アセット](../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/docs/ASSET_SOURCES.md#livox-mid-360)。

## 仕様・開発

| 対象 | 参照 |
| --- | --- |
| 構成・責務 | [Architecture](doc/ARCHITECTURE.md)・[Frontend](doc/frontend_api.md) |
| API・通信 | [Backend API](doc/BACKEND_API.md)・[WS v2](common/ws_protocol_v2.md) |
| 車両テンプレート照合 | [車両照合](doc/VEHICLE_REGISTRATION.md) |
| MCAP直接再生 | [Standalone MCAP](doc/STANDALONE_MCAP.md) |
| 性能評価の状態 | [Performance](doc/PERFORMANCE.md) |

## License

MIT
