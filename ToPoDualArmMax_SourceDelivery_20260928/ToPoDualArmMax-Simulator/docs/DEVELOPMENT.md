# 開発・移植ガイド

## ソース変更

1. `npm ci` でlockfile通りに依存を導入し、`npm run build` で型検査とUI生成を実行。
2. READMEの手順でHTTPサーバーを起動。
3. 新しいROS UIは `src/` のTypeScript/React、既存シーン・センサは `app/` のJavaScript。
4. `npm run dev` はUIの監視ビルド。更新後はブラウザ再読み込み。`server.mjs` 自体の変更はサーバー再起動が必要。
5. `npm run typecheck`・`npm run test:ros`・`npm test` と、変更に応じたブラウザ検証。

Viewerの操作UI・各Renderer・WS通信は `ToPoFuzzy-Viewer/frontend/src/`、共通型・WS v2復号・GNG描画補助は `libs/ros_visualization_web/`。
Simulatorはこれらのソースを直接参照します。npm依存はSimulator側へ固定し、Viewerのnode_modulesは不要です。
`src/ros-results.tsx` は遅延起動、`viewer-integration.tsx` は画面の接続、`graph-scene.ts` はFiberと既存描画の接続、`frame-tree.ts` はTF合成を担当します。
Viewerの `embedding.tsx` がScene・レイアウト・TF・メッシュURLの差し替え口です。Simulator固有のRendererを複製しないでください。
生成先 `app/generated/` はGit管理対象外であり、配布時にはビルド済みファイルも同梱してください。
単体ZIPでの起動には生成済みファイルを使用し、ソースの再ビルドにはワークスペースの共有ソース配置も必要です。
ビルド成果物にはReact・Fiber・Viewer UIを含み、ブラウザ実行時のnpm・CDN参照はありません。UIのCSSはShadow DOM内へ限定しています。
依存更新時は `THIRD_PARTY_NOTICES.md` と `licenses/ROS-VIEWER-LICENSES.txt` の同梱版・本文も更新してください。

Three.jsは `app/index.html` のimport mapによる固定版を共用し、UIバンドルへの二重収録を防いでいます。
ViewerとはThree.jsの版が異なるため、共有部品を変更した場合は両方の型検査・ビルド・描画テストが必要です。
依存追加・更新時は `npm audit` を確認してください。脆弱性検出0件は安全性全体の保証ではありません。

## URDF / STLの変更とキャッシュ再生成

Python 3.10以上とNumPyは、この工程だけで使用します。

```text
python -m pip install -r tools/requirements-mesh.txt
python tools/rebuild_meshes.py --model all
python tools/qa_reference.py
python tools/qa_reference.py --standard
```

標準のみは `--model standard`、Longのみは `--model long`。元のZIPや開発者のDownloadsフォルダーは不要です。各URDFに対する相対パスでSTLを読み、現在の配置を保ったまま描画キャッシュを作成します。旧キャッシュを自動削除する処理はありません。

URDFに記載された関節名、左右7軸、首・腰・カメラ/TCPリンク名はフロントエンドの設定と対応しています。別機種へ移植するときは、`robot.js` の `chain()` / `tcp()`、`app.js` のホーム・プリセット・軸操作、各センサの取付先も更新してください。

`qa_reference.py` はNumPyで独立にリンク変換を計算します。ブラウザ側の順運動学と突き合わせる基準です。URDF更新時は基準も再生成してください。

## MID-360形状 / 走査データ

実行には、同梱済みSTLと `.f32` があれば十分です。STEP変換をやり直す場合:

```text
python -m pip install -r tools/requirements-cad.txt
python tools/convert_mid360.py
```

走査方向を再抽出する場合は、`docs/ASSET_SOURCES.md` の公式LVX2記録または先頭16 MiBを別途用意して実行します。元記録そのものは送付物に含めていません。

```text
python tools/extract_mid360_reference.py path/to/Indoor_sampledata.lvx2
```

先頭5秒・100万スロット、方向不明はゼロを維持する処理です。CADや実測由来データの条件は素材一覧に従ってください。

## D435i実機校正の読み込み

`tools/export_d435i_calibration.py` はSDK `pyrealsense2` と接続された実機がある場合の補助ツールです。通常起動時には実行しません。シミュレータの「RGB-D → 校正と精度検証」でJSONを読み込みます。実機個体の校正値がない標準状態では公称値を使う近似です。

## 別サーバー / VMへの配置

`app/` をHTTP配信ルートにしてください。ES Modules、Worker、WASM、GLB、URDF、バイナリキャッシュを配信できる必要があります。JavaScriptはJavaScript MIME、WASMは `application/wasm` を設定します。同梱Nodeサーバーでは設定済みです。

Nodeサーバー自体はWindows固有APIを使用しません。Linux / macOS上での実機実行は本送付時には行っていないため、移植先で `npm test` とブラウザQAを行ってください。ROS 2との結合は `integrations/ros2/README.md` を参照してください。
