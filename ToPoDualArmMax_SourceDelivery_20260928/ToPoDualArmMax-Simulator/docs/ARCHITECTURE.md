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

`src/ros-results.tsx` はROSパネルの遅延起動、`viewer-integration.tsx` はToPoFuzzy-ViewerのApp埋め込み。
Viewer側の `embedding.tsx` を通じて画面・シーン・TF解決を差し替えます。
`graph-scene.ts` はFiberのシーングラフを既存シーンへ追加し、WebGL描画とセンサー更新はSimulatorの既存ループに集約します。
`frame-tree.ts` はTF合成・期限管理・フレーム内キャッシュと、選択モデルの名前空間からSimulatorの実リンクへの対応を担当します。TF未解決のフレームはUIに表示し、表示基準の変更はユーザー操作で行います。
`graph-scene.ts` の全体表示操作は可視オブジェクトの境界を使用し、要求時だけカメラを調整します。ROS表示用マテリアルはスタジオの霧から除外します。
`libs/ros_visualization_web` は共通型・WS v2復号・GNG描画補助。
UI・Renderer・通信フックは `ToPoFuzzy-Viewer/frontend/src` の同一ソースを使用し、ROS処理本体はバックエンドに保持します。
Markerの基本形状は更新時にマテリアルの値だけを変更し、ボクセルは容量を保持したInstancedMeshへ反映します。個数の減少だけでバッファを縮小・再生成しません。

Three.jsのES Modulesをブラウザのimport mapで解決します。新しいROS結果パネルはViteで `src/` から `app/generated/` へビルドします。`app/` の相対関係を維持すれば静的HTTPサーバーでも配信できます。ただし `/api/export` がないサーバーでは、保存先はブラウザのダウンロードになります。サーバーの配信ルートは `app/` です。

## モデル・座標・保存形式

- 長さはメートル、関節角はラジアン。UIではmm・度へ変換します。
- URDFルートはLongが`base_link`、標準が`base_footprint`。X前・Y左・Z上。センサ光学座標との変換は各取得フレームに保存。
- 標準とLongはそれぞれのURDFを読みます。標準の手首7軸目はURDF上でZ軸continuous、LongはX軸revoluteです。
- `assets.json` はURDFのメッシュ名を描画用 `.bin` へ対応付けます。`.bin` はu32頂点数・u32index数、その後float32のXYZ+法線、最後にu32の三角形index（little-endian）。
- メッシュ元STLのSHA-256を一覧に保存しています。標準版の同一STLはLongのキャッシュを再利用するため、`app/cache/` も必要です。
- ポーズ形式は `topo-motion-studio/1`、モデル識別は `model`。シーン形式は `topo-workspace/2`。
- センサ保存ZIP内の `frame.json` に `robot_model`、撮影時の関節角、校正・座標変換を含みます。

## 状態と並行実行

姿勢・シーンは各タブのメモリ内にあります。サーバーが同じでもタブ間で自動共有しません。保存名には時刻とUUIDが入り、同時保存でも衝突しません。

モデル変更時は旧ロボットをシーンから外し、センサを新しいリンクへ接続します。LiDAR Workerを再作成して旧応答を破棄し、RGB-DとAIの古い点群表示を消去します。モデルごとの姿勢・キーフレームはページを開いている間保持します。

## ROS連携の拡張・通信契約

ROS側の相互接続は同一`ROS_DOMAIN_ID`。従来VM・AI経路ではAI・observer・timing・bridgeが対象、VMを跨ぐlocalhost接続はSSH転送等による到達経路が別途必要。独立ROS 2連携とは別経路。

- launch拡張: [plugins](../integrations/ros2/plugins/)の登録JSON。例は`robot_tf.json`、型・起動管理は[ros_plugins.py](../integrations/ros2/ros_plugins.py)。ROS package・launchは事前導入、選択肢追加だけならJS分岐の追加不要。
- 物理方式: 登録JSONとPythonアダプタ。`prepare`・`create`、生成オブジェクトの`update`・`apply`・`status`・`close`、目標設定の`set_targets`。例は[gng_avoidance.py](../integrations/ros2/gng_avoidance.py)・[OSCBF](../integrations/ros2/oscbf_avoidance/README.md)。
- ROS入力: `category: input`の登録JSON。例は[ros2_topics.json](../integrations/ros2/plugins/ros2_topics.json)。`interfaces.contracts`で役割・ROSメッセージ型・トピック引数を宣言し、利用側の`input_bindings`で既存アダプタへ対応付け。設定欄は定義から生成、共通APIは`resolve_input`・`bind_input`。
- 入力設定の所有者: `ros-plugins.js`。ブラウザ内に保存、物理GNGと実験が開始時の`{plugin_id, parameters}`を参照。実験側へのトピック設定の複製なし。物理ブリッジは起動時のdomain、実験ワーカーは依頼ごとのdomainと専用namespace。
- launch所有権: 停止・切断・ページ再読込みで所有子ノードも終了。外部プロセスは対象外。APIは`GET /api/plugins`・WebSocket `/plugins`、登録外の任意コマンド実行は非対応。
- 姿勢配信: ブリッジHTTPポート+1の`/joints`、1～200 Hz。MuJoCo実行中は位置・速度・駆動トルク、物理OFFでは位置のみ。電流・接触力・clockの配信なし。実測表示と駆動指令は別トピック。実装は[joint_stream.py](../integrations/ros2/joint_stream.py)。
- VM入出力: [protocol.py](../integrations/ros2/protocol.py)・[compact_frame.py](../integrations/ros2/compact_frame.py)。stamp・frame・sequenceが揃った結果だけを表示、欠落時は更新待ち。HTTP・形式解析だけではAI処理を実行不可。

ROS入力実験は`simulate.py`の既存往復・GNG回避処理を使用。入力元bag・センサドライバは起動停止の対象外。
生成障害物との切替は同じワーカー、ROSコンテキストと所有launchは実験ごとに終了。
逐次表示は既存の有界フレーム窓を共用。`input_source: ros2`では球の座標は不要、`has_contact: null`は実接触未判定。

## 往復基準の接続契約

`shuttle_trajectory.py`は追跡障害物とbag非停止実験で共通の基準軌道生成・追従処理。
bagからの入力は点群とTF、往復目標はタスク設定。記録済みの関節指令による駆動なし。
`/sim/nonstop/reference`は保持配信の`std_msgs/String`、形式は`topo-shuttle-reference/1`。`positions`は`joint_names`順のrad、`task.reference_path`はworld座標のm。
回避側はMuJoCo非停止実験だけでモデル・関節順序・可動域を照合。`joint_stream.py`から`ros-joints.js`への基準線配信は`/sim/joint_states`の読取り専用接続、センサ入力・実機指令との分離。

## モータ表示の契約

物理実行中のスライダーは目標角、数値・グラフは実測値。ROS追従中は表示専用。

- 角加速度・ジャーク: 角速度の差分推定。ノイズの影響あり。
- トルク: アクチュエータ駆動力。接触力を含まない値。
- 電流: 実測または校正係数による推定。推定値は別表示。
- 欠測・失効・未計算: 「—」。ゼロ補完なし。
- グラフの尺度・表示上限: 表示用。実機の駆動制限とは独立。

実機値の入力と換算はパネルの設定JSONを使用。同じeffort列を電流とトルクの両方には使用不可。
DynamixelPresentの再送鮮度は識別できないため、安全監視の代用には非対応。
共通実装は[motor-monitor.js](../app/motor-monitor.js)を参照。
