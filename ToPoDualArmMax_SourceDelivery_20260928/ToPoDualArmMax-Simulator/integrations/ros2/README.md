# 任意: ROS 2 / VMware / AiS-GNG-FVG連携

通常のシミュレータを起動する際、このフォルダーの設定は不要です。

## 含めたもの / 別途必要なもの

|同梱コード|用途|
|---|---|
|`robot_bridge.py`|HTTP入力をPointCloud2 / JointStateとして発行し、処理結果を配信|
|`server.py`, `protocol.py`, `compact_frame.py`|同一stamp・座標系の結果結合、入力形式・長さ・整列の検査|
|`timing_model.py`, `timing_overlay.py`|AiS / FVGの実測処理時間集計・表示|
|`test_transport.py`|合成パケットのCDR整列・破損テスト|

別途、Ubuntu等のROS 2環境（元環境はHumble）、NumPy、`rclpy`、`sensor_msgs`、`std_msgs`、`visualization_msgs`、`ais_gng_msgs/TopologicalMap`、AiS-GNG-FVG本体、FVG observerが必要です。Overlay使用時は `rviz_2d_overlay_msgs` も必要です。

**AI本体・observer・独自メッセージパッケージ・VMイメージはこの送付物に含めていません。** 別プロジェクトで管理されている実装を受領・セットアップしてください。アルゴリズムとobserverには、以下のメッセージを発行する互換版が必要です。

## 起動例（Ubuntu / Bash）

送付フォルダーをVMへコピーし、そのルートへ移動します。ROSと外部AIワークスペースのセットアップをsourceした端末で、AI本体とobserverを起動してください。例の `~/ais_ws` は配置先に置き換えます。

```bash
source /opt/ros/humble/setup.bash
source ~/ais_ws/install/setup.bash
export ROS_DOMAIN_ID=57
export ROS_LOCALHOST_ONLY=1
python3 integrations/ros2/robot_bridge.py
```

HTTPは `127.0.0.1:8878` です。`TOPO_VM_PORT` で変更可能です。ROS domainを変える場合は、AI・observer・timing・bridgeの全プロセスで `ROS_DOMAIN_ID` を揃えてください。`timing_overlay.py` は別端末で必要なセットアップをsourceして実行します。

ホストPCからVMの127.0.0.1へ接続する場合、OS標準のOpenSSHを使います。`vm-user` と `vm-host` は受領先のアカウント・ホスト名です。

```text
ssh -N -L 8878:127.0.0.1:8878 vm-user@vm-host
```

ホスト側ブラウザで `http://127.0.0.1:8878/` を開き、「VM・AI」から開始します。VMの認証情報・SSH秘密鍵・ホスト鍵設定は同梱していません。アプリが自動でVMを起動・再構成する処理も含めていません。

## 接続仕様

|方向|インターフェース|条件|
|---|---|---|
|ブラウザ → bridge|`POST /api/input`, `X-ToPo-VM: 1`|TPC1 + UTF-8 JSON + 4 byte alignment + float32 XYZ|
|bridge → AI|`/scan`, `sensor_msgs/PointCloud2`|全有効XYZ、メートル、`base_footprint`|
|bridge → ROS|`/topo/joint_states`, `sensor_msgs/JointState`|撮影時の関節角、同じheader stamp|
|AI → bridge|`/topological_map`, `ais_gng_msgs/TopologicalMap`|独自メッセージ定義が必要|
|observer → bridge|`/ais_gng/fvg_frame`, `std_msgs/ByteMultiArray`|`compact_frame.py` の厳密なスナップショット形式|
|observer → bridge|`/fvg_observer/add`, `/delete`, `/memory`|`visualization_msgs/MarkerArray`|
|timing → bridge|`/ais_gng_fvg/processing_metrics`, `std_msgs/String`|JSON、同一フレームの処理時間|
|bridge → ブラウザ|`GET /api/status`, `/api/frame?after=N`|TFV1。`app/vm-packet.js` で復号|

受信した6種類の結果はstamp・座標系・シーケンスを合わせます。単に点群と地図を異なる時刻のまま重ねるAPIではありません。必要なメッセージの一部がない場合、GUIのグラフは更新されません。

同時送信は1タブです。他タブが送信中なら409になります。センサー計測と3D描画はブラウザ、AI本体・FVG処理は接続先で実行します。ROS 2版の保存操作はブラウザダウンロードへフォールバックします。

```text
python3 integrations/ros2/test_transport.py
```

この送付版でROS 2実行環境への再接続試験は実施していません。Pythonの構文検査・形式テストと、ブラウザ単体の検証を区別しています。
