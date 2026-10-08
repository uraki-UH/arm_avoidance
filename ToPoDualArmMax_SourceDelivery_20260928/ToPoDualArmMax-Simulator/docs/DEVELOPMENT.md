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
npm run test:performance -- --output /tmp/topo-perf-before
npm run test:performance -- --output /tmp/topo-perf-after --baseline /tmp/topo-perf-before/report.json
# CPUプロファイルを伴う原因調査。通常計測とは別の実行。
npm run test:performance -- --output /tmp/topo-perf-profile --profile
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

RGB-Dの非同期読出しは`--readback-mode batch`が既定。画像群を1つの転送バッファへまとめ、完了待ちとCPUコピーを各1回に集約。
`--readback-mode separate`で従来の画像別読出しと比較可能。
`--show-preview`でRGB-Dタブを表示して計測。既定は非表示で、プレビュー描画を保留。
表示条件が異なる結果同士の`--baseline`比較は拒否。
`methods.webgl.getBufferSubData`にCPUコピーの回数・時間、`methods.webgl.getParameter`にGPU状態照会の回数・時間を記録。
`--verify-depth`では1280×720までの出力一致、外部回転、同時取得、RGBA float代替、バッファ再利用、失敗後の復旧・破棄時の中断、再表示時のプレビュー一致も検査。

描画品質の比較は`--graph-quality standard` / `--graph-quality compact`。
軽量表示はノード・辺の件数を維持し、球・円柱の分割数だけを変更。
標準への復帰・位置・色・選択対象はViewerの`node tests/graph_geometry_browser.test.mjs`で検証。
差分更新の姿勢・色・並替え・属性交換・辺再接続の検査はSimulatorの`npm run test:graph`。

深度読出し方式の比較は`--depth-readback float` / `--depth-readback packed`。既定の`auto`は単一成分float対応GPUでR32F、非対応GPUでRGBA8を選択。
`--verify-depth`を加えると、複数解像度・同期／非同期・ステレオ・遮蔽・対象抽出の出力一致と幾何精度も検査。
低水準APIの`depth_readback: 'float'`はfloat形式を指定。従来の画像別読出しとの比較には、さらに`enable_batched_readback: false`を指定。
RGBA8格納には[GLSLのfloatBitsToUint](https://registry.khronos.org/OpenGL/specs/es/3.2/GLSL_ES_Specification_3.20.html)を使用。float32のビット表現を保持し、CPUで行反転と復元を実施。
単一成分float読出しに対応するGPUでは従来方式も1画素4バイトのため、転送量の削減なし。
`report.json`には方式・描画品質・実際の描画ターゲット形式も保存。異なる方式・品質の性能比較は可能で、センサ出力の一致検査は維持。


性能はGPU負荷や温度により変動するため、同条件で3回以上の反復を推奨。
`--baseline`は計測条件・描画環境とセンサ出力を照合し、各条件の増減率を保存。
`--max-regression-percent 20`を併記すると、フレームp95またはセンサ平均取得時間の悪化が許容率を超えた場合に終了コード1。
既定では機種依存の時間上限なし。入力・描画の成立と例外の有無だけを合否判定。

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
