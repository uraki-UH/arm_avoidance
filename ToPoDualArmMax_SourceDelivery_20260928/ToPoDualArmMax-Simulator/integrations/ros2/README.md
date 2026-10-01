# 任意: ROS 2 / VMware / AiS-GNG-FVG連携

通常のシミュレータを起動する際、このフォルダーの設定は不要です。

## 独立した点群送信

ブラウザの「ROS 2送信」タブ専用。GNG・FVG・NumPy・独自メッセージへの依存はありません。
ROS 2環境に `rclpy`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`tf2_msgs`、`trajectory_msgs` が必要です。送付アプリのルートで実行します。

```bash
source /opt/ros/humble/setup.bash
python3 integrations/ros2/pointcloud_bridge.py
```

既存ブラウザ（8877）を再読み込みし、送信先 `http://127.0.0.1:8879` を指定します。
点群種別と対象物体を選び、「1回送信」または「連続送信」を押してください。
「環境で選択中の物体を使用」で対象を同期できます。送信上限は既定2 Hz、同時取得・送信は1件です。
完全表面は既定10,000点。RGB-Dの点数・解像度は現在のセンサー設定に従います。

このワークスペースの既存Docker（hostネットワーク）からは次の起動も可能です。

```bash
docker exec -it gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash && python3 /ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/pointcloud_bridge.py'
```

| 種類 | ROS 2トピック | 内容 |
| --- | --- | --- |
| RGB-D全体 | `/sim/rgbd/points` | 現在のカメラの有効深度点 |
| 対象の完全表面 | `/sim/object/full_points` | メッシュ面積に比例する指定点数のサンプル |
| 対象の遮蔽付きRGB-D | `/sim/object/visible_points` | シーン全体と対象単体の深度が一致する有効点 |

上表は全て `sensor_msgs/PointCloud2`、XYZのfloat32、メートル、`base_footprint` 座標です。RGB画像・色フィールドの転送はありません。
完全表面は非表示メッシュ・裏面・内部面も含み、外皮の集合演算ではありません。同一形状・姿勢・点数では結果を再利用します。
遮蔽付きではロボット・他物体も遮蔽物です。視野外や全面遮蔽では0点を送信し、古い点群の再送はしません。
透明材質も幾何深度として扱い、完全に同一深度で重なる面の物体識別はできません。

`/sim/joint_states` は取得時の関節角、`/sim/points/info` は取得時刻・物体ID・姿勢等のJSONです。
ROSのheader stampは受信時のROS時計で、関節角と共通。ブラウザ取得時刻は `captured_at_ms` に別記録し、時計同期は行いません。取得時のロボット状態を含む入力ではTFも共通stampで配信します。
物体IDは現在のブラウザ内のIDです。対象削除時は対象物体の送信を停止します。
GNGの入力トピックを上表に合わせ、`ROS_DOMAIN_ID` をブリッジと揃えてください。GNGの範囲設定・入力座標系も別途整合が必要です。
実機への関節指令・衝突回避制御は行いません。

別PCのROSへ送る場合はブリッジの `--host` と `--allow-origin http://ブラウザ配信元:8877` を明示し、ブラウザの送信先を変更します。
初期状態はlocalhostのみ。停止は起動端末のCtrl+Cです。既存のVM・AIタブ用8878ブリッジとは独立しています。

形式の回帰試験は `python3 -m unittest discover -s integrations/ros2 -p test_pointcloud_bridge.py` で実行できます。
2026-09-29にHumbleでブラウザ→HTTP→3種類のROS点群の受信を確認。GNG学習・実機接続はこの試験の対象外です。

### 深度画像と画素対応点群（2026-10-01追加）

ブリッジを再起動し、ブラウザを再読み込みしてください。
「RGB-D：シーン全体」で「深度画像・CameraInfo・画素対応点群も送信」（既定ON）を選ぶと、既存点群と同じ取得フレームから次の3トピックも配信します。
対象物体モードには適用しません。チェックを外すと従来のXYZのみの送信形式に戻ります。

| トピック | メッセージ・内容 |
| --- | --- |
| `/sim/camera/depth/image_rect_raw` | `sensor_msgs/Image`、32FC1、光軸方向の深度［m］、無効画素0 |
| `/sim/camera/depth/camera_info` | `sensor_msgs/CameraInfo`、取得に使用した内部パラメータ、歪みなし |
| `/sim/camera/depth/points` | `sensor_msgs/PointCloud2`、XYZ、画像と同じwidth・height、無効画素XYZは全成分NaN |

3トピックは共通header、frame_idは `sim_camera_depth_optical_frame`（X右・Y下・Z前方）。
画像の `(u,v)` に対応する点群のバイト位置は `v * row_step + u * point_step`、point_stepは12です。
点群は `is_dense=false`。深度0の画素を除去せず位置を保持し、全画素無効でも画像寸法を維持します。
内部パラメータはK/P、Rは単位行列、Dは0。RGB画像への位置合わせは行いません。
既存 `/sim/rgbd/points` はbase_footprint座標の有効点のみで、header stampは追加3トピックと共通です。
カメラからworldへの列優先4×4変換は `/sim/points/info` の `depth_image.optical_to_world` に収録。ロボット配置は `robot_state.base_to_world`、カメラからbase_footprintへの変換は `/sim/tf` に配信します。

ブラウザでは追加描画なし。深度float32をHTTPで追加転送し、ブリッジで逆投影して画素対応XYZを生成します。
848×480の場合、HTTP追加量は約1.63 MB／フレーム、ROSの画像＋画素対応XYZは約6.51 MB／フレーム（メタデータ除外）。
通信量・CPU処理は増えます。指定Hzは上限であり、負荷に応じて低下します。
RealSenseの16UC1深度を前提とする購読側では、32FC1［m］への対応が必要です。

ROS環境での回帰試験：`python3 -m unittest discover -s integrations/ros2 -p test_depth_output.py`。
Humble実受信で848×480全画素の対応・無効値・共通時刻・既存点群との有効点数一致を確認済み。

### ロボット配置・TF・ROS軌道の往復（2026-10-01追加）

ブリッジ再起動・ページ再読み込み後、「ROS 2送信」内の「ロボットとROS」を使用します。
配置はworld基準のXYZ［m］とroll/pitch/yaw［deg］。回転順はURDFと同じZYXです。
「配置を適用」でロボット全体を移動し、環境の物体はその場に残ります。
配置はモデル別の現在のブラウザ内だけに保持し、再読み込み・ポーズJSONには保存しません。

| トピック | 型・用途 |
| --- | --- |
| `/sim/base_pose` | `geometry_msgs/PoseStamped`、world内のbase_footprint配置 |
| `/sim/joint_states` | `sensor_msgs/JointState`、関節角 |
| `/sim/tf` | `tf2_msgs/TFMessage`、world→base_footprint→URDF各リンク、および校正後カメラ |
| `/sim/robot_description` | `std_msgs/String`、選択モデルのURDF、transient local |
| `/sim/command/standard/joint_trajectory` | `trajectory_msgs/JointTrajectory`、標準モデルへの再生指令 |
| `/sim/command/long/joint_trajectory` | 同上、Longモデルへの再生指令 |

点群送信時は取得開始時の配置・関節角・各リンクTFを保持し、点群と共通stampで配信します。
有効XYZの点群は配置の逆変換を適用したbase_footprint座標、画素対応点群はカメラ光学座標です。
「姿勢・TFを定期送信」をONにすると、点群なしでも上限10 Hzで現在姿勢を送ります。
受信時のROS時計を使用するため、実センサーとの取得時刻同期を保証する仕組みではありません。

TFは `/sim/tf` のみへ配信。利用ノードで `/tf:=/sim/tf` をremapしてください。
URDFのメッシュ参照先は元のままです。RViz等で形状も表示する場合は参照先のメッシュ配置が別途必要です。
同じモデルのTFをrobot_state_publisherから重複配信しないでください。状態・指令は1つのブラウザタブで使用します。

軌道再生は明示的にONにした場合のみ有効。ON以前の最新指令は再生せず、その後に受信した指令を対象とします。
ROS側から、現在表示中モデルのトピックへ送ってください。例は首Yawの目標0.1 rad、到達時間2秒です。

```bash
ros2 topic pub --once /sim/command/standard/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: [neck_pan_joint], points: [{positions: [0.1], time_from_start: {sec: 2, nanosec: 0}}]}'
```

header stampは0、positionsのみ、time_from_startは正の厳密増加、最終時刻120秒以内、最大1,000点に対応します。
開始姿勢は受信時のブラウザ姿勢。指定関節だけを線形補間し、URDFの可動域・区間速度上限を検査します。
加速度・トルク・接触・衝突の保証はありません。velocities・accelerations・effort付き軌道は拒否します。
通信時の最新指令保持は5秒。連続した指令は最新を優先し、再生中に新しい有効軌道を受けた場合は現在姿勢から置換します。
「軌道停止・受信OFF」、Escape、タブ非表示、モデル切替で停止。通常の手動関節操作も現在の軌道再生を停止します。
通信エラーでは軌道受信・姿勢定期送信を停止します。再開はチェックを入れ直してください。

実機への指令はありません。自己点群除去・回避経路生成・GNG/VLUTへの自動接続は別処理です。
移動配置を使った従来VM・AIタブとの統合は未検証です。この座標整合の検証対象は独立ROS 2送信タブです。
回帰試験：ROS環境で `python3 -m unittest discover -s integrations/ros2 -p test_robot_exchange.py`。

## 含めたもの / 別途必要なもの

|同梱コード|用途|
|---|---|
|`robot_bridge.py`|HTTP入力をPointCloud2 / JointStateとして発行し、処理結果を配信|
|`server.py`, `protocol.py`, `compact_frame.py`|同一stamp・座標系の結果結合、入力形式・長さ・整列の検査|
|`timing_model.py`, `timing_overlay.py`|AiS / FVGの実測処理時間集計・表示|
|`test_transport.py`|合成パケットのCDR整列・破損テスト|

以下の従来VM・AI連携には別途、Ubuntu等のROS 2環境（元環境はHumble）、NumPy、`rclpy`、`sensor_msgs`、`std_msgs`、`visualization_msgs`、`ais_gng_msgs/TopologicalMap`、AiS-GNG-FVG本体、FVG observerが必要です。Overlay使用時は `rviz_2d_overlay_msgs` も必要です。

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

従来のVM・AI連携については、この送付版でROS 2実行環境への再接続試験は実施していません。Pythonの構文検査・形式テストと、ブラウザ単体の検証を区別しています。
