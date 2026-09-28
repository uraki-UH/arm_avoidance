# 開発・移植ガイド

## ソース変更

1. READMEの手順でHTTPサーバーを起動する。
2. `app/` 内のソースを編集する。UIは `index.html` とCSS、動作は各JSモジュール。
3. ブラウザを再読み込みする。自動ビルド・ホットリロードはありません。`server.mjs` 自体を変更した場合は、HTTPサーバーも停止・再起動してください。
4. `node --test tests/server.test.mjs` と、変更に応じたブラウザ検証を行う。

ブラウザのimport mapは `app/index.html` にあります。ライブラリの版を変更する場合は互換性を検証し、該当ライセンスも更新してください。実行時の依存物はすべて `app/vendor/` に固定版を同梱しており、`npm install` で版が変わる構成ではありません。

## URDF / STLの変更とキャッシュ再生成

Python 3.10以上とNumPyは、この工程だけで使用します。

```text
python -m pip install -r tools/requirements-mesh.txt
python tools/rebuild_meshes.py --model all
python tools/qa_reference.py
python tools/qa_reference.py --standard
```

標準のみは `--model standard`、Longのみは `--model long`。元のZIPや開発者のDownloadsフォルダーは不要です。各URDFに対する相対パスでSTLを読み、現在の配置を保ったまま描画キャッシュを作成します。旧キャッシュを自動削除する処理はありません。

URDFに記載された関節名、左右7軸、首・腰・カメラ/TCPリンク名はフロントエンドの設定と対応しています。別機種へ移植するときは、`robot.js` の `chain()` / `tcp()`、`app.js` のホーム・プリセット・軸操作、各センサーの取付先も更新してください。

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
