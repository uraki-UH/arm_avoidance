# ToPoDualArm-Max Simulator — ソースコード送付版

標準（無印）・Long対応 / 送付版 1.0.0 / 2026-09-28

提供URDF・STLを用いたブラウザ型シミュレータです。編集可能なソース、両モデルの元データ、描画キャッシュ、依存ライブラリを同梱しています。Codex・送付元PCのアカウント・開発フォルダーは不要です。

## 最初に起動する

必要なもの: **Node.js 22以上**とWebGL 2対応のブラウザ（Chrome / Edge等）。Node.js 24.19.0で検証しています。Node.jsは https://nodejs.org/ から導入し、`node --version` が実行できる状態にしてください。実行用Python、npm依存パッケージのインストール、ビルド作業は不要です。

### Windows

1. ZIPをすべて展開する。ZIPの中から直接実行しないでください。
2. `START.cmd` をダブルクリックする。
3. 自動で開いたブラウザを使用する。複数のブラウザやタブから同じURLへ接続できます。

既定ポートは8877です。別フォルダーのシミュレータなどが使用中なら、8878～8892から空きを選びます。同じ送付フォルダーからの再実行はサーバーを共用します。起動ログは `runtime/` に保存されます。

```powershell
# 別のポートから探す / ブラウザを開かずに起動
powershell -NoProfile -ExecutionPolicy Bypass -File .\Start-Simulator.ps1 -Port 9000 -NoBrowser
# この送付フォルダーが起動したサーバーだけを停止。起動時のポートを指定
powershell -NoProfile -ExecutionPolicy Bypass -File .\Stop-Simulator.ps1 -Port 9000
```

### Windows / Linux / macOS共通

このREADMEのあるフォルダーで実行します。

```text
node app/server.mjs
```

表示された `http://127.0.0.1:8877/` を開きます。終了は端末でCtrl+C。`npm start` も同じ動作です。Linux / macOSでは `sh start.sh` でも起動できます。ポートを変える場合はPowerShellで `$env:PORT='9000'`、Linux / macOSでは `PORT=9000 sh start.sh` とします。

`index.html` の直接オープン（file://）は対応しません。起動後、標準のシミュレータ機能は外部サイトへの通信なしで動作します。サーバーはローカルPCの127.0.0.1にのみ待ち受けます。

### 描画が重い場合（受領環境での調整）

視点の慣性（OrbitControls damping）を無効化し、マウス操作に直接追従させています。反映にはページ再読み込みが必要です。

初期描画品質を「標準」に変更しています。SSAOを無効にし、画面のピクセル比を1に設定。標準品質では画面全体の後処理用中間バッファを経由せず直接描画し、描画バッファの常時保持も無効化。PNG保存は保存直前の再描画で対応。高精細表示は「シーンと描画 → 描画品質」で選べます。RGB-Dの取得解像度・点群数・校正値は変更しません。

NVIDIAで開く場合は、サーバー起動後にワークスペースのルートで以下を実行します。

```bash
bash scripts/open_dual_arm_nvidia.sh
```

専用ランチャーはURL入力不要です。Longは末尾に `long` を指定。topofuzzy viewerと別のChromeプロファイル、X11、1440×1000の初期ウィンドウで開きます。サーバーの起動は別途必要です。既存の汎用ランチャーも引き続き利用可能で、Markdownリンク形式の引数はURLに補正します。

RGB-D連続取得では、計測・表示処理にかかった時間と同じ長さの休止を確保し、画面操作の時間を残します。指定Hzは上限であり、負荷に応じて取得頻度が下がります。非表示点群の色計算も省略。解像度・点数は維持します。VM・AI側の送信頻度制御は別処理です。変更はページ再読み込みで反映されます。

RGB-D取得中に表示用シャドウマップを更新しないよう修正しました。センサー用に物体を非表示化した状態の影が通常画面へ残る問題への対処です。反映にはページ再読み込みが必要です。

2026-10-01の標準モデル・静止画面ではRGB-D取得前後4回の画素差0を確認。ユーザー画面のちらつきとの同一原因や体感改善は未確認です。画素計算・プレビュー描画・GPU負荷は残るため、非同期化だけで操作の引っかかりが解消する保証はありません。動的場面・Longでの描画性能も未検証です。

深度画像はfloat32の1成分でGPUから読み出し、不要なRGBA成分の転送を省きます。非対応GPUでは従来のRGBA形式へ自動復帰します。

RGB-DのGPU読み出しは非同期化し、完了待ち中も画面更新を継続します。二重取得を防止し、モデル・校正変更前の取得結果は破棄します。開発用 `simulator.rgbd.capture()` の戻り値はPromiseのため `await` が必要です。低水準の `sensor.capture()` は既定で同期処理を維持しています。

GBMの `Permission denied` は、この環境ではNVIDIA描画が正常な場合にも発生しました。警告の有無だけでGPU使用・停止原因を判断しないでください。起動スクリプトは検証済みの `gl-egl` 経路を維持しています。

受領後の変更により、元の `MANIFEST.json` に対する検証では `app/app.js`、`app/index.html`、`app/rgbd-ui.js`、`app/rgbd-core.js`、`app/vm-ai.js`、`app/qa-models.js`、本READMEなどが変更済みとして表示されます。点群送信・物体編集の追加ファイルも元のMANIFESTの検証対象外です。

## 操作と収録機能

- 上部タブで標準 / Longを切り替え。URL末尾 `?model=standard` / `?model=long` でも指定できます。
- 手先マーカーの移動・回転、位置/姿勢IK、グリッパー、首2軸、腰Yaw。
- テーブル・物体・車両の配置、寸法、位置・姿勢・色の変更。
- D435i RGB-D / MID-360点群取得、距離・高さの着色、撮影データ保存。
- ポーズ・動作シーケンス・シーンの保存。Node版の保存先は `app/exports/`（初回保存時に作成）。

モデル切り替え時は環境・視点・センサー設定を保持し、姿勢とシーケンスをモデル別に保持します。ページ再読み込みを越えて残す場合はJSON保存を使用してください。モデル識別のない従来のポーズJSONはLong用として読み込みます。

これは運動学・描画・幾何センサーのシミュレータです。物理的な接触力、動力学、衝突回避や実機への制御指令は実装していません。センサーの再現範囲は [センサー仕様](docs/SENSOR_FIDELITY.md) を参照してください。

## ROS 2への点群送信・物体編集

再読み込み後の「ROS 2送信」タブで、RGB-D全体／対象物体の完全表面／対象物体の遮蔽付きRGB-Dを選択できます。
RGB-D全体では、深度画像・CameraInfo・画素対応PointCloud2も同時送信可能（既定ON）。更新後はブリッジも再起動してください。
送信には独立ブリッジ `integrations/ros2/pointcloud_bridge.py` を起動し、送信先を指定してください。
[起動・トピック・制限](integrations/ros2/README.md#独立した点群送信) を参照。GNG・FVGの処理結果を待たずに送信できます。

「環境」の追加位置で座標系（テーブル／world）とXYZ［mm］を指定し、物体・車両を追加します。
指定値は物体原点の位置であり、自動接地ではありません。追加後は既存の位置欄で変更可能です。
3D上の物体を右クリックして「この物体を削除」を選択できます。右ドラッグは従来の視点操作です。

「ROS 2送信」内の「ロボットとROS」でロボット全体のworld配置、姿勢・TFの定期送信、ROS関節軌道の再生を指定できます。
実機指令は行いません。[往復連携のトピック・制限](integrations/ros2/README.md#ロボット配置tfros軌道の往復2026-10-01追加)を参照してください。

## 編集・移植

|場所|内容|
|---|---|
|`app/`|HTML / CSS / JavaScript、Node HTTPサーバー|
|`app/source.urdf`, `app/meshes/`|LongのURDF / 元STL|
|`app/models/standard/`|標準のURDF / 元STL / 専用キャッシュ|
|`app/cache/`, `app/assets/`|描画用データ、車両、MID-360 CAD / 走査参照|
|`app/vendor/`|実行に必要なThree.js、BVH、Dracoを固定版で同梱|
|`tools/`|メッシュ再生成、独立FK基準生成、ファイル検証|
|`tests/`|配信・保存テストと元バージョンの検証結果|
|`integrations/ros2/`|独立点群送信、および外部AI環境との連携コード|
|`docs/`, `licenses/`|構成、移植手順、機能説明、第三者条件|

[開発・移植ガイド](docs/DEVELOPMENT.md) / [構成説明](docs/ARCHITECTURE.md) / [引継ぎと検証範囲](docs/HANDOVER.md)

## 受領時の確認

```text
node tools/verify.mjs
node --test tests/server.test.mjs
```

最初のコマンドは `MANIFEST.json` のSHA-256と全収録ファイルを照合します。ソースを編集・キャッシュを再生成した後は、意図した変更も差異として表示します。HTTPテストは空きポートで起動し、終了時にテスト用サーバーと保存データを片付けます。

ブラウザ検証は起動URLの末尾を `/?model=standard&qa=models` に変更します。テスト中は姿勢と表示を操作するため、作業中の画面とは別タブで開いてください。結果はページ内の非表示 `#model-qa-results` とコンソールに出力します。

## VM・AiS-GNG-FVGについて

連携用フロントエンド・ROS 2 HTTPブリッジ・形式解析コードは含みます。**AiS-GNG-FVG本体、学習用バイナリ、FVG observer、ROS独自メッセージパッケージ、VMイメージは別途必要**です。通常のシミュレータはこれらなしで起動できます。[VM連携の導入手順](integrations/ros2/README.md) を参照してください。

## 第三者素材・ライセンス

独自実装と提供URDF/STL等の扱いは [LICENSE.md](LICENSE.md)、第三者ライブラリ・素材は [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) に整理しています。全ファイルを一律のOSSライセンスとして扱ってはいません。素材別の確認済み条件と未確認点も一覧に記載しています。
