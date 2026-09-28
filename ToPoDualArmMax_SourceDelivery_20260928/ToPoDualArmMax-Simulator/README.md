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

## 操作と収録機能

- 上部タブで標準 / Longを切り替え。URL末尾 `?model=standard` / `?model=long` でも指定できます。
- 手先マーカーの移動・回転、位置/姿勢IK、グリッパー、首2軸、腰Yaw。
- テーブル・物体・車両の配置、寸法、位置・姿勢・色の変更。
- D435i RGB-D / MID-360点群取得、距離・高さの着色、撮影データ保存。
- ポーズ・動作シーケンス・シーンの保存。Node版の保存先は `app/exports/`（初回保存時に作成）。

モデル切り替え時は環境・視点・センサー設定を保持し、姿勢とシーケンスをモデル別に保持します。ページ再読み込みを越えて残す場合はJSON保存を使用してください。モデル識別のない従来のポーズJSONはLong用として読み込みます。

これは運動学・描画・幾何センサーのシミュレータです。物理的な接触力、動力学、衝突回避や実機への制御指令は実装していません。センサーの再現範囲は [センサー仕様](docs/SENSOR_FIDELITY.md) を参照してください。

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
|`integrations/ros2/`|任意のROS 2連携コード。外部AI環境が必要|
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
