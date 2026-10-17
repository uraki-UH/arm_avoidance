# 開発・移植ガイド

## ソース変更

Simulatorルートで実行:

```bash
npm ci
npm run build
npm run typecheck
npm run test:ros
npm test
```

起動は[README](../README.md)、監視ビルドは`npm run dev`。
更新後はブラウザ再読込、`server.mjs`変更後はサーバー再起動。
責務・接続構成は[ARCHITECTURE](ARCHITECTURE.md)を参照。

共有ソースはワークスペースのViewer・`libs/ros_visualization_web/`を直接参照。
依存導入はSimulator側のみ。配布時はGit管理外の`app/generated/`も同梱。
共有部品の変更時はThree.jsの版が異なるViewer側も型検査・ビルド・描画確認が必要。
依存更新時は`npm audit`を確認し、`THIRD_PARTY_NOTICES.md`・`licenses/ROS-VIEWER-LICENSES.txt`も更新。

## ブラウザの処理コスト計測

```bash
npm run test:performance -- --output ../../log/artifacts/browser_performance/before
npm run test:performance -- --output ../../log/artifacts/browser_performance/after --baseline ../../log/artifacts/browser_performance/before/report.json
# CPUプロファイルを伴う原因調査。通常計測とは別の実行。
npm run test:performance -- --output ../../log/artifacts/browser_performance/profile --profile
```

Node.js 22以上・ビルド済みUI・Chromeが必要。専用サーバーと一時プロファイルのChromeを使用し、終了・失敗時に停止。
既存ブラウザ・ROSノードへの接続なし。出力先は毎回別のディレクトリを指定。
GPU・試験条件・検証オプションは[性能試験の引数定義](../tests/performance.browser.mjs)を参照。
実GPU指定時のソフトウェア描画への代替は失敗扱い。

`render_submit`はCPUの描画送信と待機、RGB-Dの`render_ms`は読出し完了までの時間。
GPU単体時間・ネットワーク・DDS・ROS計算とは別。
`view_frame`は実描画間隔、`frame`は描画省略を含むコールバック間隔。
条件と出力ハッシュは`report.json`、CPUプロファイルは`*.cpuprofile`へ保存。

比較は条件を揃えて交互に3回以上。センサ出力は完全一致、プレビュー色のみRGB8で1段階差を許容。
時間上限は既定の合否条件に含まれず、性能劣化の判定には`--max-regression-percent`を指定。

## RGB-D点群カーネルの再生成

配布済みの`app/rgbd-points.wasm`をブラウザから読込。実行時のネイティブライブラリは不要。
`native/rgbd_points.cpp`を変更した場合は、Clangとwasm-ldのある環境で次を実行。

```bash
bash tools/build_rgbd_points.sh /path/to/clang++
npm run test:rgbd
```

生成したwasmもソースと合わせて更新。倍精度の演算順を保持し、fast-math・積和融合は無効。
参照版とのバイト一致は`test:rgbd`、実GPUは性能試験の`--verify-depth --verify-profile`で確認。

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

SDK・実機がある環境で任意に実行。取得コマンド・校正JSONの単位と制約は[RGB-D校正](RGBD_README.md#公称値と実機校正)を参照。

## 別サーバー / VMへの配置

`app/` をHTTP配信ルートにしてください。ES Modules、Worker、WASM、GLB、URDF、バイナリキャッシュを配信できる必要があります。JavaScriptはJavaScript MIME、WASMは `application/wasm` を設定します。同梱Nodeサーバーでは設定済みです。

Nodeサーバー自体はWindows固有APIを使用しません。送付時点ではLinux / macOS上の起動は未検証のため、移植先で `npm test` とブラウザQAを行ってください。ROS 2との結合は[ROS連携](../integrations/ros2/README.md)を参照。

## 汎用モータ計算

```bash
python3 -B -m libs.motor_model --config libs/motor_model.example.json
```

workspaceルートで実行。設定は[入力例](../../../libs/motor_model.example.json)、APIは[motor_model.py](../../../libs/motor_model.py)を参照。
入力例は未校正の仮想値。シミュレータ駆動への自動適用・実機通信なし。
