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

## ブラウザの処理コスト計測

```bash
npm run test:performance -- --output ../../log/artifacts/browser_performance/before
npm run test:performance -- --output ../../log/artifacts/browser_performance/after --baseline ../../log/artifacts/browser_performance/before/report.json
# CPUプロファイルを伴う原因調査。通常計測とは別の実行。
npm run test:performance -- --output ../../log/artifacts/browser_performance/profile --profile
```

Node.js 22以上、ビルド済みUI、Chromeが必要。`CHROME_BIN`で実行ファイルを指定可能。
専用HTTPサーバーと一時プロファイルのChromeを起動し、終了・失敗時に両方を停止。
既存ブラウザ・ROSノードへの接続なし。出力先は毎回別のディレクトリを指定。
既定GPUはMesa。NVIDIAは`--gpu nvidia`、描画環境なしでの動作確認は`--gpu software`。
実GPU指定時のソフトウェア描画への代替は失敗扱い。GPU名・ブラウザ版・画面寸法・モデル・ソースのSHA-256を`report.json`へ保存。

既定の試験は通常描画、画角表示、LiDAR、RGB-D、GNG静止表示、GNGの10 Hz更新。
`graph_stream`は1ノードだけの更新、`graph_stream_all`は全ノードの更新。両方の条件で差分更新の効果と上限を確認可能。
`--cases idle,lidar,rgbd,graph_static,graph_stream`で選択可能。
各条件は新しいページから開始。Longモデル、1440×1000のウィンドウ、既定校正、センサの既定品質を使用。
GNGは`--nodes 10000`が既定で、1万ノード・3万辺の固定入力。
WS v2バイナリを受信ハンドラへ渡し、実際の復号・React更新・共有Rendererを測定。
ネットワーク転送・DDS・ROS側の計算時間は含まない。

`--duration-ms 3000`で各条件の計測期間、`--captures 10`でセンサの取得回数を指定。
センサは両方を満たすまで逐次取得。初回のBVH・シェーダー構築は`warmup`へ別記。
LiDARは同じ走査開始位置を使用。センサ出力配列のSHA-256も保存し、基準結果との比較時に完全一致を確認。
フレーム間隔のp50/p95/max、50 ms超の回数、メインスレッドのタスク時間、描画呼出し時間、センサ処理時間、GC後のヒープ差分を記録。
`render_submit`はCPUからの描画送信とそこで生じた待機の時間であり、GPU単体の実行時間ではない。
RGB-Dの`render_ms`は描画・非同期読出し完了までの経過時間。CPU画像処理は`compute_ms - render_ms`。
CPUプロファイルは`*.cpuprofile`としてChrome DevToolsへ読込可能。

方式の比較オプションは[性能試験の引数定義](../tests/performance.browser.mjs)を参照。
比較軸は画像読出し方式、プレビュー表示、深度色付け、点群生成、通常ビューの描画抑制、Graph形状品質。

- 基準比較: 旧条件`--points-backend javascript --view-scheduling continuous`と新条件`--points-backend wasm --view-scheduling priority`。
- 同等性: センサ出力は完全一致、プレビュー色だけRGB8で1段階差を許容。表示条件の異なる`--baseline`比較は拒否。
- 検証: `--verify-depth --verify-profile`で最大1280×720、遮蔽・外部回転・同期／非同期取得・代替形式・資源再利用・異常後復旧・プレビューを確認。
- 指標: `view_frame`は実描画間隔、`frame`は描画省略を含むコールバック間隔。`--suppress-main-view`はGPU競合の切分け専用。
- 軽量Graph: 球・円柱の分割数だけ変更。件数・位置・色・選択の検証はViewerの`tests/graph_geometry_browser.test.mjs`、差分更新はSimulatorの`npm run test:graph`。
- 深度形式: 対応GPUはR32F、非対応GPUはRGBA8のfloat32ビット保持。両者とも1画素4バイトで転送量削減とは別。

GPU負荷・温度を考慮して方式を交互に3回以上比較。
時間上限は既定の合否条件に含まれず、入力・描画成立と例外の有無で判定。
`--max-regression-percent`指定時はフレームp95・センサ平均取得時間の悪化も判定。
設定・形式・品質・実行環境・センサ出力のハッシュは`report.json`に保持。

## RGB-D点群カーネルの再生成

配布済みの`app/rgbd-points.wasm`をブラウザから読込。実行時のネイティブライブラリは不要。
`native/rgbd_points.cpp`を変更した場合は、Clangとwasm-ldのある環境で次を実行。

```bash
bash tools/build_rgbd_points.sh /path/to/clang++
npm run test:rgbd
```

ワークスペースの既存Docker環境では次のコマンドを使用可能。

```bash
docker exec gng_cpu_container bash /ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/tools/build_rgbd_points.sh /opt/emsdk/upstream/bin/clang++
```

コンパイラオプションは倍精度の演算順を保持し、fast-math・積和融合を無効化。生成したwasmもソースと合わせて更新。
`test:rgbd`で参照版とのバイト一致、欠損・丸め境界・バッファ再利用・読込失敗時の代替を検査。
`--verify-depth --verify-profile`による実GPUでの検査も実施。

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

`tools/export_d435i_calibration.py` はSDK `pyrealsense2` と接続された実機がある場合の補助ツールです。通常起動時には実行しません。出力JSONをRGB-D校正に使用。実機個体の校正値がない標準状態では公称値を使う近似です。

## 別サーバー / VMへの配置

`app/` をHTTP配信ルートにしてください。ES Modules、Worker、WASM、GLB、URDF、バイナリキャッシュを配信できる必要があります。JavaScriptはJavaScript MIME、WASMは `application/wasm` を設定します。同梱Nodeサーバーでは設定済みです。

Nodeサーバー自体はWindows固有APIを使用しません。送付時点ではLinux / macOS上の起動は未検証のため、移植先で `npm test` とブラウザQAを行ってください。ROS 2との結合は `integrations/ros2/README.md` を参照してください。

## 汎用モータ計算

```bash
python3 -B -m libs.motor_model --config libs/motor_model.example.json
```

workspaceルートで実行。設定は[入力例](../../../libs/motor_model.example.json)、APIは[motor_model.py](../../../libs/motor_model.py)を参照。
入力例は未校正の仮想値。シミュレータ駆動への自動適用・実機通信なし。
