# 任意: ROS 2 / VMware / AiS-GNG-FVG連携

通常のシミュレータを起動する際、このフォルダーの設定は不要です。

## サーバーとROSブリッジの一括起動

ホストのワークスペースルートで実行（起動済みの `gng_cpu` サービスを使用）：

```bash
bash scripts/open_dual_arm_nvidia.sh long
```

サーバー・ROS/物理ブリッジの起動確認後にGPU用ブラウザを開きます。未起動のサービスはDocker内で背景起動し、既存サービスは再利用します。ブラウザ終了後もバックエンドは継続します。新規起動時に表示する `docker compose exec gng_cpu kill -INT <PID>` で、その起動が管理するサービスだけを停止できます。ログはコンテナ内の `/tmp/topo-simulator-<UID>-8877-8879.log`。

Docker内で操作中ならアプリのルートで `bash start_ros.sh`。Node.js 22以上とROS Humble環境が必要です。
アプリは `http://127.0.0.1:8877/?model=long`、ROS送信先は `http://127.0.0.1:8879`。
ブラウザのMID-360有効化・連続取得開始と「ROS2連携 → 連続送信」は手動です。
旧版が残っている場合は `bash start_ros.sh --restart` で同じ配置・ポートの起動元とブリッジを停止して再起動します。別配置・別ポートのプロセスは対象外です。管理中のブリッジはPythonソース更新を検知して自動再起動します（外部起動したサービスの再利用中は対象外）。
Docker内で `bash start_ros.sh` を直接実行した場合は、Ctrl+Cで今回起動したサービスだけを停止。既存サービスの設定は変更しません。ブリッジのROSドメインは起動環境の `ROS_DOMAIN_ID` に従います。
別ポートは `bash start_ros.sh --port 8880 --bridge-port 8881`。ブラウザのURLと送信先も表示された値に変更してください。

## ROS処理結果の表示専用接続

通常の `bash start_ros.sh` で起動します。UI未生成時やThree.js本体の欠損時は依存取得とビルドを自動実行します。生成済みUIのソース更新時は、Simulatorルートで `npm ci && npm run build` を実行してください。Dockerの依存準備・更新手順は[アプリREADME](../../README.md)を参照してください。
点群送信用ブリッジとは別に、同じROSドメインで更新済みのViewerゲートウェイを起動します。
ToPoFuzzy-Viewerの画面起動は不要です。

```bash
source /ros2_ws/install/setup.bash
source /ros2_ws/src/ToPoFuzzy-Viewer/backend/install/setup.bash
ros2 run topo_fuzzy_viewer viewer_ws_gateway_node
```

詳細選択には `viewer_edit_node`、詳細のモデル照合には `viewer_vehicle_registration_node.py` も必要です。同じセットアップをsourceした別端末で起動します。

```bash
ros2 run topo_fuzzy_viewer viewer_edit_node
# モデル照合用の別端末
ros2 run topo_fuzzy_viewer viewer_vehicle_registration_node.py
```

「ROS 2連携 → ROS Scene Layers」の既定接続先は `ws://127.0.0.1:9001/observe`。変更時だけ「接続・座標設定」を開きます。
パネルの初回表示と接続先変更時に自動接続します。Topics行の `Online` をクリックすると切断し、手動切断後は `Offline` をクリックして再接続します。右端の更新アイコンで一覧を再取得できます。
`Topics` のチェックボックスで入力の購読・表示を切り替えます。ROS由来のロボットは `Robot · <名前>` として同じ一覧に表示し、初期状態は未選択です。URDFと最新姿勢を1件として保持し、選択時に描画します。Simulator本体のロボットはROS接続なしでも表示・動作します。
`Scene Layers` は目アイコンと、色・透明度などの表示設定です。削除ボタンは入力の選択解除と連動します。
ToPoFuzzy-Viewerの `App`・通信フック・各Renderer・設定パネルを共用しています。

| 表示 | 入力・操作 |
| --- | --- |
| GNG・軌道・認識／把持結果 | TopologicalMap。ノード・エッジ・クラスタ、属性によるラベル色、法線／速度、共分散／可操作性楕円体 |
| 平面・非平面成分 | PlaneClusterArray、`/nonplane_components`。同じframe_number・frame_idの `/topological_map` から所属ノードと内部エッジを復元 |
| 点群 | PointCloud2。色・透明度・ヒートマップ |
| Marker・姿勢 | Viewer対応のMarkerArray／PoseArray。矢印・基本形状・線・文字・姿勢軸 |
| ボクセル | voxel_msgs/Voxel。ラベル・色・透明度、差分更新 |
| ロボット | Viewerのrobot description／poseストリーム。URDF・姿勢候補・衝突形状・可操作性 |
| 詳細 | Bounding Box対応レイヤーを有効化し候補枠をクリック。独立3Dビュー・最新取得・モデル照合 |
| 表示設定 | View・Clip・Analyze、色・ラベル、表示変換、監視領域、ローカルMesh Models |

入力属性がない機能は表示されません。詳細選択の対応条件もViewerと共通で、環境 `/topological_map` は候補詳細選択の対象外です。
ROSノード起動停止・rosbag再生・サーバーファイル操作・点単位の編集・認識パラメータ変更はこの表示専用パネルに含みません。

- Topicsの選択: 通常トピックはROS購読の開始。ロボットは受信済みモデルと最新姿勢の描画開始。ゲートウェイの購読は接続中のクライアント間で共有。
- チェック解除: この画面の受信済みレイヤーと描画待ちデータを除去し、後着データの再表示を抑止。別画面の購読停止なし。ロボットのモデル・最新姿勢は再選択用に保持。
- 再接続: 同じ接続先では画面内の選択状態を保持。接続先変更・ページ再読み込みでは初期化。
- 表示基準: シーン原点に対応するROSフレームは既定 `world`。TF未解決のデータは非表示とし、パネルにフレーム名を表示。
- 選択モデルのフレーム: `world` 基準では、Longの `topo_dual_arm_max_long/<リンク名>`、標準モデルの `topo_dual_arm_max/<リンク名>` をSimulatorの実リンク配置に対応。モデル切替・手動配置・関節姿勢に追従。別モデルの名前空間は推測で対応しません。
- `base_footprint`: ROS側に当該TFがない場合だけ、Simulatorの同名基準リンク配置を使用。
- TFのない環境地図: `map を基準に表示` で、そのフレームをシーン原点として表示し、可視ROSデータ全体へカメラを移動。これは表示基準の指定であり、`map` とロボットの `world` の位置合わせには対応するTFが必要です。
- 表示サイズ: 広い地図で点・線が細い場合は、Scene Layersの対象レイヤーの色設定（Node／Edge）から `Node Size`／`Edge Width` を調整。既定のノード半径は0.003 m。
- 動的TF: 最終受信から2秒で失効。静的TFは切断まで保持。観測時刻への補間なし。
- 切断: 受信データとTFを消去。接続中の更新停止だけでは静的レイヤーを消去しません。
- 描画: 主シーンは既存のCanvas・描画ループを共用。ROS表示専用layer 3をセンサー撮影から除外。ROS表示用マテリアルはスタジオの霧の対象外。詳細3Dビューだけ独立Canvas。
- 更新: 同一描画フレーム内の受信をまとめて反映。TF行列はフレーム内で再利用。バッファと描画方式はViewerと共通。

ゲートウェイはlocalhost待受。Originは既定で `http://localhost:8877`・`http://127.0.0.1:8877` とViewerの5173を許可。
別ポートは `--ros-args -p 'allowed_origins:=["http://127.0.0.1:8882"]'` を指定します。
`/observe` は購読開始・一覧取得・描画確認と、保存を伴わない詳細計算／モデル照合だけを許可します。ROS操作・編集確定・購読解除要求は拒否します。
ログイン認証・遠隔公開は未対応。既存の `/` は従来Viewer用の操作入口です。

検証: `npm run test:ros`、`npm test`。実ROS・ブラウザ統合は `npm run test:browser`。
後者は既存 `gng_cpu_container`、更新済みbackendビルド、Chromeが必要です。専用ROSドメイン187・空きポートを使用し、終了時に試験プロセスを停止します。
GPU測定は `TOPO_TEST_GPU=mesa npm run test:browser`。通常試験はSwiftShaderであり、GPU性能の判断には使いません。

## 独立した点群送信

ブラウザの「ROS2連携」タブ専用。GNG・FVG・NumPy・独自メッセージへの依存はありません。
ROS 2環境に `rclpy`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`tf2_msgs`、`trajectory_msgs` と、C++17コンパイラ・実行Python用の開発ヘッダー（Ubuntuでは `g++`、`python3-dev`）が必要です。送付アプリのルートで実行します。

```bash
source /opt/ros/humble/setup.bash
python3 integrations/ros2/pointcloud_bridge.py
```

点群の非有限値検査・深度の逆投影・色付け・PointCloud2用配列の構築はC++で実行します。計算中はGILを解放し、HTTP・姿勢更新のPythonスレッドを実行可能にします。深度の逆投影と色付けは1回の画素走査に集約します。
ネイティブモジュールは起動時に必要な場合だけビルドし、OSの一時ディレクトリ内の `topo-pointcloud-native-<UID>` へソース・Python ABI別にキャッシュします。事前ビルドは `python3 integrations/ros2/native_points.py`。ビルド失敗時は起動エラーになり、低速なPython処理へ自動で戻りません。
`GET /api/points/status` の `pointcloud_backend: "cpp"` で稼働版を確認できます。既存ブリッジへの反映は再起動後。`start_simulator.py` が管理するブリッジは関連ソースの更新を検出して再起動します。

既存ブラウザ（8877）を再読み込みし、送信先 `http://127.0.0.1:8879` を指定します。
送信したいトピックにチェックを入れ、「連続送信」を押してください。RGB-D・MID-360・完全表面・遮蔽付き点群は複数選択できます。
ブリッジ起動だけでは `/sim/*` の配信口を作りません。最初の有効な送信で必要なトピックだけを生成し、3秒以上送信がない配信口を解除します。再送信で再生成します。深度画像・CameraInfo・画素対応点群も深度データ送信時だけ生成します。URDFのtransient local配信は初回送信後に保持し、後から接続する表示クライアントへモデルを渡します。
取得・Hz設定はRGB-D／LiDARタブ、物体点群の対象・点数・取得Hzは「環境 → 物体点群の取得」で指定します。送信側は点群を生成せず、選択したトピックの新規フレームを順番に送ります。同時通信は1件、待機中は各トピックの最新分だけを保持します。
完全表面は既定10,000点。RGB-Dの点数・解像度は現在のセンサ設定に従います。

このワークスペースの既存Docker（hostネットワーク）からは次の起動も可能です。

```bash
docker exec -it gng_cpu_container bash -c 'source /opt/ros/humble/setup.bash && python3 /ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/pointcloud_bridge.py'
```

| 種類 | ROS 2トピック | 内容 |
| --- | --- | --- |
| MID-360 | `/sim/lidar/points` | 現在の取付姿勢・走査設定で得た有効点 |
| RGB-D全体 | `/sim/rgbd/points` | 現在のカメラの有効深度点 |
| 対象の完全表面 | `/sim/object/full_points` | メッシュ面積に比例する指定点数のサンプル |
| 対象の遮蔽付きRGB-D | `/sim/object/visible_points` | シーン全体と対象単体の深度が一致する有効点 |

上表は全て `sensor_msgs/PointCloud2`、XYZのfloat32、メートル、`base_footprint` 座標です。RGB-D全体と遮蔽付きRGB-Dは `rgb`（packed FLOAT32）・`color_valid`（UINT8）付きで、point_stepは20 bytes。色が取得できない点は代替の灰色と `color_valid=0`。MID-360・完全表面はXYZのみで12 bytesです。RGB画像そのものは送信しません。
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

### GNGへの接続

ブラウザ点群とGNGの基準座標系は `base_footprint`。CPU用の `sim_rgbd.yaml` を使用します。

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=sim_rgbd.yaml
```

入力は `/sim/rgbd/points`、TF必須設定は有効。同じ座標系の入力は追加変換なしで学習します。
実機用の `graspnet.yaml` は `ToPoDualArm/base_link` を要求するため、トピック名だけの変更では代用できません。URDF内の `base_link` はロボット構造用として維持し、`base_footprint` と固定TFで接続します。

### MID-360の送信

1. ブリッジを起動し、ブラウザを再読み込み。
2. LiDARタブでMID-360を有効化。「腰上・前方45°の配置に戻す」で前下がりの配置、XYZ・RPY欄で調整。
3. LiDARタブの「連続取得 Hz」（既定10 Hz、0.1〜40 Hz）を設定し、「連続取得」を開始。
4. 「ROS2連携」で `/sim/lidar/points` にチェックを入れて「連続送信」。ROS側のHz設定は不要です。取得済みの最新フレームから送信し、その後は新規フレームだけを送ります。

点群は `/sim/lidar/points`、取得時の `base_footprint → sim_mid360_frame` は `/sim/tf` に共通stampで配信。点群は既にbase_footprint座標のため、取付変換の二重適用は不要です。ロボットのworld配置・関節角も取得開始時の状態を使用します。
`/sim/points/info` の `lidar` に走査開始秒・積分秒・スロット数・走査方式を収録。積分0.1秒は20,000スロットで、有効点数ではありません。intensity・点別時刻・IMU・Livox CustomMsgは出力しません。
連続取得Hzは実時間の取得上限で、実速度は描画と取得処理に依存。積分時間は1フレームの走査量であり、取得Hzとは別の設定です。ROS側は新規フレームだけを送り、送信中に複数取得された場合は最新分を使用。送信停止ではLiDAR取得は止めません。ROSの受信時刻とシミュレーション走査時刻は別です。GNGへ接続する場合は入力を `/sim/lidar/points` に設定し、ブリッジとROSドメインを揃えてください。

2026-10-07にLong・標準モデルからHumbleへの実受信、移動配置・25°／40°の取付で全点とTFの一致、連続送信、取得中リセットを確認。回帰17件成功。ブラウザ試験はSwiftShaderのためGPU性能・指定Hzの達成は未検証です。

### 深度画像と画素対応点群（2026-10-01追加）

ブリッジを再起動し、ブラウザを再読み込みしてください。
「RGB-D：シーン全体」で「深度画像・CameraInfo・画素対応点群も送信」（既定ON）を選ぶと、既存点群と同じ取得フレームから次の3トピックも配信します。
対象物体モードには適用しません。チェックを外すと追加の深度画像・CameraInfo・画素対応点群を送信せず、色付きの `/sim/rgbd/points` のみ送信します。

| トピック | メッセージ・内容 |
| --- | --- |
| `/sim/camera/depth/image_rect_raw` | `sensor_msgs/Image`、32FC1、光軸方向の深度［m］、無効画素0 |
| `/sim/camera/depth/camera_info` | `sensor_msgs/CameraInfo`、取得に使用した内部パラメータ、歪みなし |
| `/sim/camera/depth/points` | `sensor_msgs/PointCloud2`、XYZ・rgb・color_valid、画像と同じwidth・height、無効画素XYZは全成分NaN |

3トピックは共通header、frame_idは `sim_camera_depth_optical_frame`（X右・Y下・Z前方）。
画像の `(u,v)` に対応する点群のバイト位置は `v * row_step + u * point_step`、point_stepは色情報付きで20、色情報なしで12です。
点群は `is_dense=false`。深度0の画素を除去せず位置を保持し、全画素無効でも画像寸法を維持します。
内部パラメータはK/P、Rは単位行列、Dは0。RGB画像への位置合わせは行いません。
既存 `/sim/rgbd/points` はbase_footprint座標の有効点のみで、header stampは追加3トピックと共通です。
カメラからworldへの列優先4×4変換は `/sim/points/info` の `depth_image.optical_to_world` に収録。ロボット配置は `robot_state.base_to_world`、カメラからbase_footprintへの変換は `/sim/tf` に配信します。

ブラウザでは追加描画なし。深度float32をHTTPで追加転送し、ブリッジで逆投影して画素対応XYZを生成します。
848×480の場合、HTTP追加量は約1.63 MB／フレーム、ROSの画像＋画素対応点群は色情報付きで約9.77 MB／フレーム、色情報なしで約6.51 MB／フレーム（メタデータ除外）。
通信量・CPU処理は増えます。指定Hzは上限であり、負荷に応じて低下します。
RealSenseの16UC1深度を前提とする購読側では、32FC1［m］への対応が必要です。

ROS環境での回帰試験：`python3 -m unittest discover -s integrations/ros2 -p test_depth_output.py`。
C++処理の出力・並行実行・不正入力・ROSシリアライズ往復の検証は `python3 -m unittest discover -s integrations/ros2 -p test_native_points.py`。
実HTTP→ROS受信の検証は `ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1 python3 -m unittest discover -s integrations/ros2 -p test_pointcloud_native_http.py`。試験専用ブリッジは終了時に停止し、既存ブリッジは操作しません。
性能比較は `python3 integrations/ros2/benchmark_pointcloud_native.py --backend cpp --output /tmp/points-cpp.json`。`--backend python` は比較専用の従来ループで、配信処理には使用しません。既定は848×480・有効画素約30.5%、`--width`・`--height`・`--seed`で変更可能。初回ビルド・入力生成を除いた、検査・ROSメッセージ生成・シリアライズの時間を記録します。GPU取得・HTTP・DDSの所要時間は含みません。

Humble実受信で848×480全画素の対応・無効値・共通時刻・既存点群との有効点数一致を確認済み。

色はブラウザのdepth→color外部パラメータと遮蔽判定で対応付けたRGBを使用します。取付補正は校正JSONの `mount`、内部パラメータは `depth` に設定。補正済み光学フレームをTFへ反映し、base_footprint点群には変換を適用済みです。実機から校正値を推定する機能ではありません。更新時はブラウザとブリッジの両方を再起動してください。

### MuJoCoと点群

回避オプションのOSCBFは[専用フォルダの導入・検証手順](oscbf_avoidance/README.md)を参照。既定は回避なし。

一括起動後、「環境 → MuJoCo」で使用します。Python依存は `mujoco==3.3.7`、Dockerfileにも追加済みです。
未導入の場合はコンテナ内で `python3 -m pip install mujoco==3.3.7`。更新後は `bash start_ros.sh --restart` とブラウザ再読み込みです。

- ロボットと物体のON/OFFは、右クリックと「物理」タブで共通の「物理を有効にする」を使用。ロボットは既定ONで、物理タブでは物体未選択時にロボット全体が対象。物理開始後は基台固定の関節動力学を使用。標準・Longの同梱URDFから関節軸、慣性、可動範囲、トルク上限、グリッパー連動を生成。
- シミュレータ操作中の手動操作・軌道による角度変更は目標値として扱い、MuJoCoの結果を表示へ反映。ロボットの力学をOFFにすると描画のみとなり、ロボットの接触形状・接触力・関節指令は物理計算から除外。環境だけの物理実行やROS実測の直接表示との併用が可能。接触判定だけを返す検査機能は未実装。
- テーブル注視・ポーズ再生・VM AI動作・プリセットは、物理OFFならブリッジ不要。動作ボタンからの物理自動起動なし。ロボットの力学ONで明示的に物理を開始した場合のみ接続完了を待ち、物理時間で指令を進行。実行中の切断では再生を停止。
- 「関節固定・自己接触・回避」で選択した関節は開始時の角度を拘束し、駆動対象から除外。基台・URDF固定関節は固定。固定・連動・可動範囲はソルバによる拘束で微小な誤差あり。
- 自己接触は既定OFF。ONではURDF衝突メッシュの凸包同士を判定。初期姿勢の重なりや形状近似による押し戻しの可能性あり。
- 物体モードはON時の「固定／動的／姿勢指定／ヒンジ／スライド」の5種類。OFFは共通チェックで指定し、再度ONにすると同じページ内で前回の種類・質量・拘束を保持。固定はworld固定、姿勢指定は外部指定位置へ追従し、モータトルク制限の対象外。
- ヒンジ・スライドの軸と支点[m]は物体ローカル指定。開始位置0を含む範囲を入力して「拘束を適用」。画面単位はヒンジdeg、スライドm。開始配置からworldへ軸・支点を固定。物体同士をつなぐ拘束は未対応。
- 物体の質量[kg]・モード・拘束はシーンJSONへ保存。ロボットの固定選択・自己接触は画面内設定で、JSON保存対象外。
- 「物理を開始」で現在配置から生成。「停止」で配置を保持。配置編集・削除・モデル切替・基台移動・ページ非表示で停止し、変更後は再開始。
- 「落下する箱を追加」でテーブル上35 cmに箱を追加して開始。物理なしの物体もLiDAR・RGB-Dの対象。

駆動はPD（P=40、D=3）と重力・コリオリ補償、URDFのeffortで合計モータトルクを制限。velocityは目標の変化速度に適用し、外力による実速度の上限保証はありません。
関節ダンピング0.1、armature 0.001を付加。実機の摩擦・減速機・ドライバを同定したモデルではありません。拘束反力・接触力はモータトルクとは別です。
ロボットの接触はURDF衝突メッシュの凸包および箱・球・円柱、環境物体は箱近似。環境の慣性は形状・質量から生成。摩擦は共通値 `0.8 0.005 0.0001`、床はworld Z=-0.14 m。

固定刻み2 msで5ステップずつ計算。目標角送信は最大約30 Hz、結果反映は描画周期。実時間への追従保証はありません。
通信は関節WebSocketと同じポートの `/physics`。ブラウザごとの独立セッションです。
物体・関節の結果を描画シーンへ適用し、既存LiDAR・RGB-Dで取得。走査中の移動歪み、MuJoCo時計とROSの同期、`/clock`は未実装です。

2026-10-07：標準・LongのURDF運動学一致、関節追従、トルク上限、連動、固定・ヒンジ・スライド、落下・接触を含む8テスト成功。
Longブラウザで首0.4 rad・左肩0.3 radへの追従と左肩固定（約0.2992 rad）を確認。既存サーバーの `/api/status` は404でしたが、物理通信・姿勢反映は成功。
従来の環境物体検証では箱の天板上高さ0.35 m→約0 m、LiDAR対象点の平均world Z 0.550→0.200 m、保存・再読込を確認。

### s・r・fの接続選択

このワークスペースでは `ros2 launch gng_vlut_system robot_follow.launch.py` を使用できます。Longが既定、USBドライバは別起動です。「ROS2連携 → s・r・fの接続構成」で、r/fからsへの描画・力学追従、r/sからfへの追従を選択し、接続関係・入力鮮度・実機状態を確認できます。管理中の姿勢入力・送受信設定は共通構成へ従い、sの状態送信は1ブラウザに限定します。モデルが共通launchと一致しない場合は接続を止めます。

選択だけでは力学・実機追従を開始しません。力学は「sの力学を開始」、実機はlaunchの出力許可と「fの操作」が必要です。構成変更・入力失効・切断後の実機追従は明示再開始。共通管理中の入力失効は既定0.3秒です。通常の単独シミュレータは従来の操作を維持します。
設定一覧・起動引数・実機の小動作制限は[gng_vlut_systemの共通追従構成](../../../../gng_vlut_system/docs/dynamixel_sim_control.md#srfの共通追従構成)を参照してください。

### 姿勢・TFのWebSocket送受信

「ROS2連携 → ロボット状態の送受信」で通信上限を **1～200 Hz** に設定します。初期値100 Hz。
送信は `/sim/joint_states`、受信は指定した `sensor_msgs/JointState` トピック（初期値 `/joint_states`）。
「ロボット状態をROSへ送信（関節角・配置・TF）」1つで関節角・配置・TFを一括送信します。MuJoCo動作中は計算結果の位置・速度・アクチュエータ駆動力をJointStateへ送信します。effortは関節トルク／力で、接触力は含みません。物理OFFでは位置のみで、velocity／effortは空配列です。送信上限は1〜200 Hz、物理更新・描画より高い設定では同じ状態を再送します。時刻はROSブリッジの時計を使用し、MuJoCo時間による `/clock` 配信には対応しません。通常の状態送信は実機指令と独立、共通launch使用時だけ管理されたsの状態をfの追従元へ転送します。
共通管理なしの送信と受信追従は択一。共通launch管理中は検査済みの経路で送受信を併用し、関節角・配置・TFを一括配信。関節軌道再生とJointState追従も択一です。
受信は既知の関節名だけを反映。直接表示ではグリッパーとmimicだけをURDF開閉端へ飽和し、端点付近の実測角でも開閉表示を継続します。他の可動域外関節は関節名を警告し、その関節だけ前回表示を保持して正常な関節の更新を継続します。ROSの実測値への変更なし。非有限値・不明な関節名は受信停止。リーダー制御の検査・停止条件は別経路で維持。部分関節の更新に対応します。
通信は専用Workerで実行します。古い受信姿勢は蓄積せず、描画フレームごとに最新の一件を反映します。設定Hzは上限で、到達を保証するものではありません。
ブラウザ送信は表示中モデルの現在角を送ります。描画より高い頻度では同じ角度が繰り返される場合があります。
点群とTFは別経路。TF・Poseによるベース位置の受信追従は未対応です。
モデル切替・停止ボタンで関節通信を停止します。JointState追従は別パネルのクリックやページ非表示だけでは解除しません。背景タブの描画周期はブラウザに依存します。

「ロボット → 操作対象ロボット → 姿勢の入力元」で、シミュレータ操作／ROS実測の直接表示／ROS入力 → s（力学）を選択します。直接表示は同じJointState受信設定を使用し、MuJoCoを停止して手動関節更新を抑止します。通信停止時は最後の姿勢を保持し、シミュレータ操作へ明示的に戻すまで手動更新を抑止します。Viewerに表示したROSロボットは独立した表示個体で、この選択によって統合されません。

物理フォロワーは`/leader/joint_states`を受信し、ロボットを右クリックして「物理を有効にする」をONにし、「物理を開始」で追従を開始します。腕は最初の有効なリーダー姿勢と物理実姿勢を基準に保存し、角度差をMuJoCoのPDモータ目標へ入力。グリッパーは換算済み開度へ初回入力から追従。リーダーの入力専用開閉端設定は[gng_vlut_systemの手順](../../../../gng_vlut_system/docs/dynamixel_sim_control.md#実機リーダーフォロワー制御)を参照。描画姿勢へのリーダー角度の直接代入なし。URDFの可動域・指令速度・トルク上限と接触計算は既存の物理経路を使用します。入力に実時間のheader stampが必要、旧stamp再送による鮮度更新なし。

リーダー入力・物理実測・目標送信は描画周期から独立。入力失効1秒、通信OFF・切断・可動域超過で保持して追従停止。物理側にも独立した失効監視を配置し、ブラウザの更新停止でも旧目標を解除。通信復旧だけでは再開せず、「停止」→「物理を開始」で明示再開と基準姿勢の更新。ページ非表示では物理セッションを終了します。実機への指令なし。

実機Dynamixelの読取り・ID対応は[リーダー・フォロワー起動手順](../../../../gng_vlut_system/docs/dynamixel_sim_control.md#実機リーダーフォロワー制御)を参照。実機出力OFFでのリーダー配信が可能。新しいstamp付き通信と物理監視の反映には、下記のブリッジ再起動とページ再読み込みが必要です。
同じ欄で操作対象ロボットの表示と不透明度を調整できます。元の色・金属感・粗さを保持し、床などの共有材質は変更しません。ロボットの非表示は仮想センサの可視メッシュにも影響します。

Python依存は `tornado`（Ubuntuでは `sudo apt install python3-tornado`）。Dockerfileにも追加済み。
WebSocketはブリッジHTTPポートの次のポートを使用します（既定 `ws://127.0.0.1:8880/joints`）。
コンテナのポート公開を使う場合はHTTP用8879に加え8880も公開してください。hostネットワークでは追加公開不要です。
旧ブリッジは `bash start_ros.sh --restart` で更新し、ブラウザを再読み込みしてください。
2026-10-07、Long・分離ROSドメインで各4秒間の送信を測定し、設定1／100／200 Hzに対して約1.0／100.3／200.0 Hzを確認。200 HzのJointState入力で首関節0.2 radの反映と停止を確認。点群取得停止・ブラウザ前面表示での測定であり、RGB-D同時取得中の周期は未検証です。

### ロボット配置・TF・ROS軌道の往復（2026-10-01追加）

ブリッジ再起動・ページ再読み込み後、送受信は「ROS2連携」タブ、手動配置は「ロボット」タブの「ロボット配置」を使用します。
配置はworld基準のXYZ［m］とroll/pitch/yaw［deg］。回転順はURDFと同じZYXです。
「配置を適用」でロボット全体を移動し、環境の物体はその場に残ります。
配置はモデル別の現在のブラウザ内だけに保持し、再読み込み・ポーズJSONには保存しません。

| トピック | 型・用途 |
| --- | --- |
| `/sim/base_pose` | `geometry_msgs/PoseStamped`、world内のbase_footprint配置 |
| `/sim/joint_states` | `sensor_msgs/JointState`、関節角 |
| `/sim/tf` | `tf2_msgs/TFMessage`、world→base_footprint→URDF各リンク、および取得時のカメラ・MID-360 |
| `/sim/robot_description` | `std_msgs/String`、選択モデルのURDF、transient local |
| `/sim/command/standard/joint_trajectory` | `trajectory_msgs/JointTrajectory`、標準モデルへの再生指令 |
| `/sim/command/long/joint_trajectory` | 同上、Longモデルへの再生指令 |

点群送信時は取得開始時の配置・関節角・各リンクTFを保持し、点群と共通stampで配信します。
有効XYZの点群は配置の逆変換を適用したbase_footprint座標、画素対応点群はカメラ光学座標です。
配置・TF・関節角の定期送信は「ロボット状態の送受信」の1つの送信スイッチで開始・停止します。個別の送信選択はありません。同じスナップショットから、共通の1～200 Hz・ROS時刻で `/sim/base_pose`、`/sim/joint_states`、`/tf`・`/sim/tf` へ配信します。点群送信に付随する取得時の姿勢・TFは別経路です。
受信時のROS時計を使用するため、実センサとの取得時刻同期を保証する仕組みではありません。

TFは標準の `/tf` と互換用の `/sim/tf` に同じ内容を配信します。専用配信だけにする場合はブリッジ起動時に `--tf-topic /sim/tf` を指定し、利用ノードで `/tf:=/sim/tf` をremapしてください。
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
移動配置を使った従来VM・AIタブとの統合は未検証です。この座標整合の検証対象は独立ROS2連携タブです。
回帰試験：ROS環境で `python3 -m unittest discover -s integrations/ros2 -p test_robot_exchange.py`。

2026-10-01までの検証は標準モデル・ROS Humble。3種の点群の実受信、848×480全画素の深度・XYZ対応と共通時刻、移動配置（XYZ=(0.3,-0.2,0.1) m、yaw=25 deg）での点群・TF整合を確認。ROS軌道→ブラウザ首Yaw 0.1 rad→ROS関節状態の往復、未知関節・過速度・範囲外の拒否、停止操作を確認し、Python回帰15件が成功しています。

LongのRGB-D実送信・軌道往復、持続送信レート、大規模車両メッシュの性能、実RealSense購読アプリとの互換性、GNG/VLUTとの統合運転、実機動作は未検証です。

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

同時送信は1タブです。他タブが送信中なら409になります。センサ計測と3D描画はブラウザ、AI本体・FVG処理は接続先で実行します。ROS 2版の保存操作はブラウザダウンロードへフォールバックします。

```text
python3 integrations/ros2/test_transport.py
```

従来のVM・AI連携については、この送付版でROS 2実行環境への再接続試験は実施していません。Pythonの構文検査・形式テストと、ブラウザ単体の検証を区別しています。
