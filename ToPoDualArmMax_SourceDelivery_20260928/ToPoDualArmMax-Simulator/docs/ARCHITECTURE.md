# 構成と変更箇所の案内

## データの流れ

```text
models.js → 選択モデルのURDF + assets.json → robot.js → Three.js描画
                         ↑ 関節角                   ↓ リンク変換
                  app.js（マーカー / IK）     RGB-D / MID-360点群
                                                      ↓ 任意
                                    vm-ai.js → HTTP → ROS 2ブリッジ
                                                      ↓
                                        外部AiS-GNG / FVG observer
                                                      ↓
                                    同一時刻の処理結果 → ブラウザ表示
```

## フロントエンド

|モジュール（app/以下）|責務|
|---|---|
|`app.js`|シーン初期化、UI、モデル切替、姿勢・シーケンス、IK呼出、更新ループ|
|`models.js`|モデルID、URDF・メッシュ一覧のURL、保存名|
|`robot.js`|URDFのリンク/関節構築、順運動学、減衰最小二乗IK|
|`environment.js`, `environment-editor.js`|テーブル・物体配置と編集、シーンJSON|
|`vehicles.js`|GLB読み込み、車体寸法・材質・軸合わせ|
|`rgbd-core.js`, `rgbd-ui.js`|GPU深度・RGB、逆投影、校正、画像/点群保存|
|`lidar-core.js`, `lidar-worker.js`, `lidar-ui.js`|三角形レイキャスト、取付、取得状態管理|
|`measured-scan.js`|MID-360公式記録由来の走査方向参照|
|`pointcloud-colors.js`, `capture-zip.js`|点群の表示色、PLY、保存ZIP|
|`vm-ai.js`, `vm-packet.js`|任意のVM入力・結果描画、バイナリ形式|
|`server.mjs`|静的配信、健康確認、ファイル保存|

Three.jsのES Modulesをブラウザのimport mapで解決します。バンドラーは不要です。`app/` の相対関係を維持すれば静的HTTPサーバーでも配信できます。ただし `/api/export` がないサーバーでは、保存先はブラウザのダウンロードになります。サーバーの配信ルートは `app/` です。

## モデル・座標・保存形式

- 長さはメートル、関節角はラジアン。UIではmm・度へ変換します。
- URDF世界座標は `base_footprint`、X前・Y左・Z上。センサー光学座標との変換は各取得フレームに保存します。
- 標準とLongはそれぞれのURDFを読みます。標準の手首7軸目はURDF上でZ軸continuous、LongはX軸revoluteです。
- `assets.json` はURDFのメッシュ名を描画用 `.bin` へ対応付けます。`.bin` はu32頂点数・u32index数、その後float32のXYZ+法線、最後にu32の三角形index（little-endian）。
- メッシュ元STLのSHA-256を一覧に保存しています。標準版の同一STLはLongのキャッシュを再利用するため、`app/cache/` も必要です。
- ポーズ形式は `topo-motion-studio/1`、モデル識別は `model`。シーン形式は `topo-workspace/2`。
- センサー保存ZIP内の `frame.json` に `robot_model`、撮影時の関節角、校正・座標変換を含みます。

## 状態と並行実行

姿勢・シーンは各タブのメモリ内にあります。サーバーが同じでもタブ間で自動共有しません。保存名には時刻とUUIDが入り、同時保存でも衝突しません。

モデル変更時は旧ロボットをシーンから外し、センサーを新しいリンクへ接続します。LiDAR Workerを再作成して旧応答を破棄し、RGB-DとAIの古い点群表示を消去します。モデルごとの姿勢・キーフレームはページを開いている間保持します。
