# ToPoDualArm-Max Simulator

標準・LongのURDF/STLを使ったブラウザシミュレータ。別PCの構築は[RUN_GUIDE](../../RUN_GUIDE.md)を参照。

## 最初に起動する

Node.js 22以上・WebGL 2対応ブラウザ。初回ビルドはワークスペース全体の配置が必要。

```bash
npm ci
npm run build
node app/server.mjs
```

表示: `http://127.0.0.1:8877/?model=long`。終了: Ctrl+C。`file://`での直接オープンは非対応。
生成済みUI・Three.jsを含む配布物はNode.jsだけで起動可能。UI変更後は再ビルド。

### Windows

展開後に`START.cmd`。起動・停止のオプションは[Start-Simulator.ps1](Start-Simulator.ps1)・[Stop-Simulator.ps1](Stop-Simulator.ps1)を参照。

### Windows / Linux / macOS共通

`node app/server.mjs`または`npm start`。Linux/macOSでは`sh start.sh`も使用可能。
ポート変更は環境変数`PORT`。既定はlocalhost待受。

### 描画が重い場合

Docker・ROS連携付きの起動はホストのworkspaceルートで:

```bash
bash scripts/open_dual_arm_nvidia.sh long
```

## 物体のホバー選択・ドラッグ配置

| 操作 | 動作 |
| --- | --- |
| 左クリック／Ctrl・⌘＋クリック | 単体／複数選択 |
| Delete | 選択物体の削除 |
| 左ダブルクリック | 物体設定 |
| Alt＋左ドラッグ | 移動候補。離した後も持ち直し可能 |
| Alt＋右ドラッグ | その場のYaw回転 |
| Alt＋ホイール／Alt＋Shift＋ホイール | Pitch／Roll回転。Ctrl併用で細かい刻み |
| T／Shift＋T | Yawを±15°回転 |
| 候補上の左ダブルクリック／Enter | 配置確定。Alt併用可 |
| Alt＋Esc／Esc | 回転だけ初期化／選択・候補の解除 |
| 通常の左／右ドラッグ | 視点回転／平行移動 |


## 編集・移植

[DEVELOPMENT](docs/DEVELOPMENT.md)・[ARCHITECTURE](docs/ARCHITECTURE.md)・[HANDOVER](docs/HANDOVER.md)を参照。
元URDF・STLは`app/source.urdf`・`app/meshes/`（Long）、`app/models/standard/`（標準）。
UIビルドにはworkspaceの`ToPoFuzzy-Viewer/frontend/src/`・`libs/ros_visualization_web/`も必要。

## 受領時の確認

```bash
node tools/verify.mjs
node --test tests/server.test.mjs
```

### Longの基準フレーム

Longは`base_link`、標準は`base_footprint`。URDF・ブリッジの更新後は対象ノードを再起動。