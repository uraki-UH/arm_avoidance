# 任意: ROS 2 / VMware / AiS-GNG-FVG連携

Ubuntuの別PCでNVIDIAなしにROS 2・CPU物理から準備する場合は、[ワークスペースの初回構築手順](../../../../RUN_GUIDE.md)を参照してください。

通常のシミュレータを起動する際、このフォルダーの設定は不要です。

## サーバーとROSブリッジの一括起動

ホストのワークスペースルートで実行（起動済みの `gng_cpu` サービスを使用）：

```bash
bash scripts/open_dual_arm_nvidia.sh long
```

サーバー・ROS/物理ブリッジの起動確認後にGPU用ブラウザを開きます。未起動のサービスはDocker内で背景起動し、既存サービスは再利用します。ブラウザ終了後もバックエンドは継続します。新規起動時に表示する `docker compose exec gng_cpu kill -INT <PID>` で、その起動が管理するサービスだけを停止できます。ログはコンテナ内の `/tmp/topo-simulator-<UID>-8877-8879.log`。

ROS環境は共通の`start_ros.sh`で読込み。ROS本体に加え、Dockerでは`/ros2_ws/install/local_setup.bash`から独自メッセージ・GNGノードを使用します。`pointcloud_bridge.py`の単体起動も同じ環境入口を経由します。

Docker内で操作中ならアプリのルートで `bash start_ros.sh`。Node.js 22以上とROS Humble環境が必要です。
アプリは `http://127.0.0.1:8877/?model=long`、ROS送信先は `http://127.0.0.1:8879`。
ブラウザのMID-360有効化・連続取得開始と「ROS2連携 → 連続送信」は手動です。
旧版が残っている場合は `bash start_ros.sh --restart` で同じ配置・ポートの起動元とブリッジを停止して再起動します。別配置・別ポートのプロセスは対象外です。管理中のブリッジはPythonソース更新を検知して自動再起動します（外部起動したサービスの再利用中は対象外）。
Docker内で `bash start_ros.sh` を直接実行した場合は、Ctrl+Cで今回起動したサービスだけを停止。既存サービスの設定は変更しません。ブリッジのROSドメインは起動環境の `ROS_DOMAIN_ID` に従います。
別ポートは `bash start_ros.sh --port 8880 --bridge-port 8881`。ブラウザのURLと送信先も表示された値に変更してください。

## ROS launch プラグイン

ブラウザの選択肢・引数入力と、ROS側の起動・準備判定・停止を登録定義で接続する共通枠組み。アルゴリズム本体はROSノードまたは既存のPython実装へ配置。登録・型検証・launch管理・WebSocket接続は`ros_plugins.py`に集約。

### 利用

「ROS2連携 → ROS launch プラグイン」で機能を選び、開始／停止。モデルは開始時にブラウザで表示中のロボットを使用。初期プラグイン`robot_tf`は名前空間`browser_preview`の`robot_state_publisher`。モデル・名前空間の入力は不要。関節指令の出力なし。引数を持つ機能では開始中・実行中の編集不可。ログ欄は末尾8 KiB。

GNG／OSCBFは「物理 → 回避モジュール」から選択。設定欄も同じ登録定義から生成。GNGのROS launchは物理セッションの開始・終了と連動。OSCBFは既存Pythonフィルタの互換アダプタ。

既存のブラウザサーバーとROSブリッジの更新が必要。動作停止後にSimulatorルートで`bash start_ros.sh --restart`を実行し、ブラウザを再読み込み。エージェントによる既存サービスの停止・再起動は未実施。

### launchの追加

`plugins/`にJSONを追加。起動対象のROS packageとlaunchは事前導入が必要。次の例の`my_planner`は利用者のpackage名に置換。

```json
{
  "version": 1,
  "id": "my_planner",
  "category": "launch",
  "label": "経路計画",
  "description": "ROSノードによる経路計画",
  "parameters": {
    "namespace": {"type": "string", "label": "名前空間", "default": "browser_planner"},
    "publish_hz": {"type": "number", "label": "配信 Hz", "default": 10, "min": 1, "max": 30}
  },
  "launch": {
    "package": "my_planner",
    "file": "planner.launch.py",
    "arguments": {
      "namespace": {"parameter": "namespace"},
      "publish_hz": {"parameter": "publish_hz"}
    },
    "ready": {
      "kind": "topic",
      "name": "/{namespace}/status",
      "message_type": "std_msgs/msg/String",
      "max_wait_sec": 20
    }
  }
}
```

ブラウザ再読み込みで一覧と設定欄を反映。launchの追加だけならJSの分岐追加は不要。`parameters`の型は`string`／`topic`／`bool`／`number`／`integer`。`choices`は選択肢、`min`／`max`は数値範囲、`default`は既定値、`is_required`は必須指定。`topic`は絶対ROSトピック名。`is_hidden`はUI非公開の接続用引数。

`launch.arguments`は利用者入力の`parameter`、接続処理が渡す`context`、固定値のいずれか。引数はシェルを介さず配列で`ros2 launch --noninteractive`へ渡す。ブラウザからの要求は登録IDと宣言済み引数のみ。

`ready`はこのlaunchが提供する`topic`または`service`。名前・型で準備判定。型付きの出力が既にある名前空間、同じプラグインの同じ名前空間の重複起動は拒否。準備時間切れ・異常終了は失敗状態とログで表示。ROS出力の継続確認は0.5秒間隔、実行後の出力失効は2秒の猶予。

ローカルlaunchは`package`／`file`の代わりに`path: "plugins/launch/example.launch.py"`で登録可能。登録ディレクトリ外へのパス参照は禁止。

`interfaces`は入力・出力の仕様記載。トピック名・メッセージ型・対応パラメータを宣言可能。自動リマップや任意メッセージの描画は含まない。結果の描画には既存のROS Scene Layersなどを使用。

### 物理方式の追加

`category: "physics"`とPythonの`adapter: "module:class"`を登録。`models`は対応するモデルID。アダプタの役割は次の共通接続。

- `prepare(settings, robot)`：学習モデルなどの準備。`fixed_joint_names`を含む辞書を返却。
- `create(scene, settings, node, robot, prepared)`：計算実装との接続オブジェクトを返却。
- 接続オブジェクト：`update()`、`apply()`、`status`。所有資源がある場合は`close()`。目標姿勢の受付は`set_targets()`。
- `capabilities.can_set_goal_pose`：ブラウザからの名目目標姿勢の受付。
- `capabilities.pose_sources`：対応する姿勢入力元。
- `display`：表示名、状態名、数値項目。表示用の整形だけで、動作判定・制御への使用なし。

`gng_avoidance.py`の`gng_adapter`と`oscbf_avoidance/__init__.py`の`oscbf_adapter`が実装例。GNGは学習条件から固定関節を準備し、既存のGNG/VLUTノードとPython回避処理を接続。制御アルゴリズムを新規実装する場合はROSノードへ置き、アダプタには状態・指令・結果の橋渡しを配置。

### 所有権と終了

新規launchは独立したプロセスグループで管理。明示停止、接続先変更、ブラウザ再読み込み・切断、ブリッジ終了時に所有グループを停止。SIGINT→SIGTERM→SIGKILLで子ノードも回収。既存の外部プロセスは停止対象外。ブラウザの別タブへの切替だけでは汎用launchを停止しない。物理セッションの停止方針は従来どおり。

API：ブラウザサーバーの`GET /api/plugins`が公開カタログ。ROSブリッジのHTTPポート+1にあるWebSocket `/plugins`が`start`／`stop`を受付。起動構成・Pythonアダプタの指定はローカル登録定義だけで管理。

現在の移行対象は起動管理・方式選択・設定画面。GNGのPython回避処理、OSCBFのPythonフィルタ、ブラウザのIK・表示用計算は既存実装を維持。アルゴリズム全体のROSノード化は別の移行範囲。

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
候補枠と環境物体が重なる場合、通常クリックは候補詳細、Ctrl／Metaクリックは環境物体の複数選択です。
ROSノード起動停止・rosbag再生・サーバーファイル操作・点単位の編集・認識パラメータ変更はこの表示専用パネルに含みません。

- Topicsの選択: 通常トピックはROS購読の開始。ロボットは受信済みモデルと最新姿勢の描画開始。ゲートウェイの購読は接続中のクライアント間で共有。
- チェック解除: この画面の受信済みレイヤーと描画待ちデータを除去し、後着データの再表示を抑止。別画面の購読停止なし。ロボットのモデル・最新姿勢は再選択用に保持。
- 再接続: 同じ接続先では画面内の選択状態を保持し、選択済みトピックの購読を復元。接続先変更・ページ再読み込みでは初期化。
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

`/sim/joint_states` はロボット状態の送信経路から配信する現在の関節角です。点群取得時の関節角・配置は `/sim/points/info` のJSONに保持し、現在姿勢へ再配信しません。
ROSのheader stampは受信時のROS時計で、点群と取得時センサTFは共通stampです。ブラウザ取得時刻は `captured_at_ms` に別記録し、時計同期は行いません。
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

回避は「なし／GNG-VLUT／OSCBF」から選択。GNG-VLUTは下記のMID-360 bag接続、OSCBFは[専用フォルダの導入・検証手順](oscbf_avoidance/README.md)を参照。既定は回避なし。

一括起動後、ロボットの用途・指令・制御は「ROS2連携 → ロボットの用途と制御」、物体・関節固定・接触設定は「物理」で指定します。Python依存は `mujoco==3.3.7`、Dockerfileにも追加済みです。
未導入の場合はコンテナ内で `python3 -m pip install mujoco==3.3.7`。更新後は `bash start_ros.sh --restart` とブラウザ再読み込みです。

- ロボットと物体のON/OFFは、左ダブルクリックの「物理を有効にする」または「物理」タブの一覧のON／OFFボタンを使用。ロボットは既定ONで、物理タブでは物体未選択時にロボット全体が対象。物理開始後は基台固定の関節動力学を使用。標準・Longの同梱URDFから関節軸、慣性、可動範囲、トルク上限、グリッパー連動を生成。
- シミュレータ操作中の手動操作・軌道による角度変更は目標値として扱い、MuJoCoの結果を表示へ反映。ロボットの力学をOFFにすると描画のみとなり、ロボットの接触形状・接触力・関節指令は物理計算から除外。環境だけの物理実行やROS実測の直接表示との併用が可能。接触判定だけを返す検査機能は未実装。
- テーブル注視・ポーズ再生・VM AI動作・プリセットは、物理OFFならブリッジ不要。動作ボタンからの物理自動起動なし。ロボットの力学ONで明示的に物理を開始した場合のみ接続完了を待ち、物理時間で指令を進行。実行中の切断では再生を停止。
- 「関節固定・自己接触・回避」で選択した関節は開始時の角度を拘束し、駆動対象から除外。基台・URDF固定関節は固定。固定・連動・可動範囲はソルバによる拘束で微小な誤差あり。
- 自己接触は既定OFF。ONではURDF衝突メッシュの凸包同士を判定。初期姿勢の重なりや形状近似による押し戻しの可能性あり。
- 物体モードはON時の「固定／動的／姿勢指定／ヒンジ／スライド」の5種類。OFFは共通チェックで指定し、再度ONにすると同じページ内で前回の種類・質量・拘束を保持。固定はworld固定、姿勢指定は外部指定位置へ追従し、モータトルク制限の対象外。
- ヒンジ・スライドの軸と支点[m]は物体ローカル指定。開始位置0を含む範囲を入力して「拘束を適用」。画面単位はヒンジdeg、スライドm。開始配置からworldへ軸・支点を固定。物体同士をつなぐ拘束は未対応。
- 物体の質量[kg]・モード・拘束はシーンJSONへ保存。ロボットの固定選択・自己接触は画面内設定で、JSON保存対象外。
- 「物理を開始」で現在配置から生成。「停止」で配置を保持。Alt＋左ドラッグ後の「直接配置 → 確定」は実行中のMuJoCoへ反映し、再開始不要。動的物体は速度をゼロ、ヒンジ・スライドは新しい配置を拘束基準として変位をゼロへ変更。他の配置編集・削除・モデル切替・基台移動・ページ非表示では停止し、変更後は再開始。
- 「落下する箱を追加」でテーブル上35 cmに箱を追加して開始。物理なしの物体もLiDAR・RGB-Dの対象。

駆動はPD（P=40、D=3）と重力・コリオリ補償、URDFのeffortで合計モータトルクを制限。velocityは目標の変化速度と、位置誤差補正を含むPD目標速度に適用。物理計算の実速度には瞬間的な上振れがあり、実速度の上限保証はありません。
関節ダンピング0.1、armature 0.001を付加。実機の摩擦・減速機・ドライバを同定したモデルではありません。拘束反力・接触力はモータトルクとは別です。
ロボットの接触はURDF衝突メッシュの凸包および箱・球・円柱、環境物体は箱近似。環境の慣性は形状・質量から生成。摩擦は共通値 `0.8 0.005 0.0001`、床はworld Z=-0.14 m。

固定刻み2 msで5ステップずつ計算。目標角送信は最大約30 Hz、結果反映は描画周期。実時間への追従保証はありません。
通信は関節WebSocketと同じポートの `/physics`。直接配置は`place`要求と`placed`応答を使用し、応答前の古い物理フレームは配置番号で除外。旧ブリッジは`bash start_ros.sh --restart`とブラウザ再読み込みで更新。ブラウザごとの独立セッションで、構築・計算・解放は専用workerが担当。未処理の姿勢入力は最新1件だけ保持し、関節通信のイベントループでは物理計算を行いません。
物体・関節の結果を描画シーンへ適用し、既存LiDAR・RGB-Dで取得。走査中の移動歪み、MuJoCo時計とROSの同期、`/clock`は未実装です。

2026-10-07：標準・LongのURDF運動学一致、関節追従、トルク上限、連動、固定・ヒンジ・スライド、落下・接触を含む8テスト成功。
Longブラウザで首0.4 rad・左肩0.3 radへの追従と左肩固定（約0.2992 rad）を確認。既存サーバーの `/api/status` は404でしたが、物理通信・姿勢反映は成功。
従来の環境物体検証では箱の天板上高さ0.35 m→約0 m、LiDAR対象点の平均world Z 0.550→0.200 m、保存・再読込を確認。

### MID-360 bagによるGNG-VLUT回避

計算・再検査・判定範囲は既定方策`graph`の仕様。追加の回避優先方策は下記参照。

- 対象: Long、`gng_vlut_system/config/topo_dual_arm_max_long.yaml`の独立左右腕モデル。学習済みGNG/VLUT・独自ROSメッセージが必要。
- 操作: Longで開き、「ROS2連携 → ロボットの用途と制御」で用途を「MuJoCoシミュレーション」、指令元を「回避モジュール」、制御方式を「位置制御」に設定。「GNG-VLUT（ROS launch）」を選択し、「シミュレーション開始」。必要なROSノードは自動起動、停止・切断時に終了。
- 目標: 物理停止中に関節スライダー・手先ドラッグで任意の腕姿勢を作り、物理を開始すると、その姿勢を回避後の復帰目標に使用。GNG動作中の関節スライダー・ポーズ指定も目標値として保持し、描画姿勢はMuJoCo実測を維持。
- 姿勢例: ポーズ欄の「前方・肘曲げ」。Longの手先は基台基準で約`(0.409, ±0.203, 0.444) m`、肘は40°。首・腰・グリッパーの目標は学習条件の0。物理開始前の基準姿勢指定と、GNG動作中の復帰目標更新に対応。
- 入力: `/livox/lidar`、`/topo_dual_arm_max_long/roi_voxels`、記録自己マスク`/topo_dual_arm_max_long/self_voxel`。bagのGNG状態・関節姿勢・自己除去済みROIは回避入力に使用しない。
- 計算: 記録自己マスク一致部と自己境界候補を除いたROIを占有・危険ボクセルとVLUTへ入力。境界・孤立候補を含む制御用照合は別系統。現在のシミュレータ自己領域による環境点の除去なし。学習時の自己干渉フィルタを通過した関節角GNG接続（`edge_mode=0`）上で安全ノード列を探索。片腕ずつ経由姿勢をMuJoCoへ入力し、既存の補間で継続追従。計画更新は約10 Hz、モータ制御は500 Hz。URDFの指令速度・トルク上限での位置・速度PD。
- 自己除去: 直近32個の記録自己マスクから近傍時刻を選択、時刻差上限0.05 sec。格子・原点・フレームの一致確認。マスク外の中心間距離0.02 mを自己境界候補として分離。候補はGNG環境から除外、制御用`control_voxels`と要確認評価には保持。空間索引の再利用、再ボクセル化・膨張セル集合の全生成なし。
- ノイズ候補: 境界外の26近傍、または直前観測の近傍セルによる空間・時間の裏付け分類。孤立した新規物体の削除なし。元bag時刻の巻戻し・1秒を超える観測間隔・格子変更では時間的裏付けの持越しなし。
- 接触評価: 記録自己マスク一致部の除去後ROIとMuJoCo実測の`self_voxel`を照合。受信鮮度1 sec・時刻差0.2 secの近似評価。境界外の裏付けあり重なりは`contact`、自己境界・孤立候補のみは`uncertain`、重なりなしは`clear`、欠測・空入力・不一致は`unavailable`。自己境界も未分離総重なり数に保持。対象は全リンク。実物との厳密な接触・自己点群の完全除去の証明なし。
- 再検査: 経路ノードの最新VLUT状態・接続・左右組合せの照合。観測制約ON、回避優先方策、または自己境界候補のある観測では、制御用ボクセル・自己干渉に対する残区間検査。姿勢間隔0.025 rad、周期0.02 sec、新規指令は適用前に検査。連続区間安全の厳密な保証なし。
- 観測格子: 既定`fov_shadow`。`free`＝センサ視野内・非遮蔽の推定自由空間、`obstacle`＝記録自己除去後の制御用環境セル、`unknown`＝物体・記録ロボットの背後、視野外、近距離死角。孤立・自己境界候補も制御用障害物に保持。既存ROIと同じ解像度・原点・ID、Long基台の各軸−1.2〜1.2 m。GNGノード三値とは別分類。
- 視野・遮蔽: MID-360公称視野は水平360°、上下−7〜52°、近距離死角0.1 m（[仕様](https://www.livoxtech.com/jp/mid-360/specs)）。生点群・既存ROI・記録自己マスクの有限幅投影による影の上書き。格子と取付姿勢の座標キャッシュ、点群の再登録・反復膨張なし。freeは未検出物体の不在証明ではなく、新しい観測ごとのobstacle・unknownへの変更を許容。
- 観測制約: 既定ON。開始身体以外への新規進入はfreeが条件、開始身体内でも障害物の免除なし。点群・ROIの元時刻差上限0.15 sec、受信後0.3 secでfree失効。空ROIでも有効な測距・視野推定があれば利用、測距なし・TF欠落・範囲外・処理上限超過は保持。過去freeの蓄積・点単位の走査歪み補正なし。
- 切替・依存: GUI「遮蔽・視野外への進入制限」、CLI `--roi-visibility fov_shadow|ray|legacy`。`ray`は実光線の通過セルだけのfree、`strict`は`ray`の互換名、`legacy`は観測制約なしの比較用。C++部品`libgng_visibility_kernel.so`のビルド・installが必要、未配置・ABI不一致での暗黙な制約解除なし。
- 表示: 計画状態とMuJoCoの腕の実速度を分けて表示。復帰候補があっても実速度が小さい場合は「復帰待ち・姿勢保持」。安全な開始姿勢にいる場合は、回避入力があっても常時の腕動作は不要。入力不足は該当トピック名を表示。
- 出力: `/sim_topo_dual_arm_max_long/`以下の`joint_states`、`self_voxel`、`roi_voxels`、`control_voxels`、`occupied_voxels`、`danger_voxels`、左右`Tmap_*`・`gng_node_states_stamped`。環境分類は`supported_environment_voxels`／`uncertain_environment_voxels`、自己境界候補は`recorded_self_boundary_voxels`、重なりは`contact_voxels`／`uncertain_contact_voxels`。関節フィードバックは実測の位置・速度・トルク、上限50 Hz。ROS送信チェックボックスとは独立。
- 評価記録: CLI・物理応答の`avoidance.observation`。`state`は現在の判定、`history_state`は観測履歴。重なり数・未確定重なり数・ノイズ候補分離前の重なり数、最大重なり・接触サンプル数・有効観測時間`covered_sec`を保持。最後の`clear`だけでは実験成功の判定不可。記録自己マスク内に重なった実物・遮蔽領域の識別は対象外。
- 分類記録: `avoidance.roi_visibility`に方式・鮮度・セル数・棄却理由・更新／照会時間。購読時だけの`observed_free_voxels`は`fov_shadow`では推定freeも含む互換出力。障害物は`fov_shadow`で`control_voxels`、`ray`で`supported_environment_voxels`、範囲内の残りはunknown。`observation.clear`は重なりなしであり、自由空間の保証とは別。[自己除去](../../../../benchmarks/self_filter/results.json)、[視野推定の評価](../../../../benchmarks/fov_shadow/results.json)。
- 座標: 記録ROIの`topo_dual_arm_max_long/base_link`と仮想基台を重畳。ROIに取付角45度の再適用なし。視野・遮蔽・光線にはMID-360の生点群と元時刻のセンサTFを使用。ROI占有点群の再登録なし。
- 時刻: 新着ROIだけのROS現在時刻への対応、観測側のbag巻戻しへの対応。起動時の入力待機は時間制限なし。欠測・無効な観測は姿勢保持、近接・重なり時は検査済み退避候補の探索。有効観測での再検査、異常な関節状態・モデル不一致では停止。
- 制約: 学習時の首・腰・グリッパー固定条件を既存のMuJoCo関節拘束へ反映。GNG対象の腕関節は固定不可。同じ機体の回避セッションは1つ。実機指令の出力なし。
- 判定範囲: VLUTはノード姿勢での環境干渉、左右組合せ検査はノード姿勢での自己干渉。実測・任意目標と最寄りノードの接続は近似。学習時の接続検査と危険ボクセルによる余裕を利用した構成であり、動的障害物に対する区間全体の厳密な保証なし。

既存bagが上記3入力とセンサTFを再生中なら追加の再生は不要。限定再生はROSコンテナ内で`bash /ros2_ws/src/scripts/play_arm_avoidance_bag.sh --loop`。観測制約追加後も、この単視点bagでの往復タスク完遂・全身重なり解消は未達。観測不足による保持と回避成功の区別が必要。

#### 回避優先方策（実験）

- 選択: GNG設定の「回避方策」=`avoidance_first`、またはCLIの`--gng-strategy avoidance_first`。既定値: `graph`。既存ブリッジの実行中は、利用者によるバックエンド更新・再起動が必要。
- 優先順位: 退避 → 姿勢保持 → タスク復帰。現在姿勢または目標までの区間の離隔0.10 mを契機とするタスク中断。離隔0.14 m・安全継続0.5 secを条件とする復帰。距離は観測ボクセルとURDF外接球の近似値。
- 退避先: 各関節の両方向微動と合成降下方向の一括評価。目標姿勢への誘引なし、角度更新幅0.12 rad。自己干渉・関節範囲・区間離隔の確認済み候補のみの採用。最新観測による残区間検査の周期0.02 sec、検査姿勢間隔0.025 rad。
- 制限: 局所探索による行き止まり・通過不能時の姿勢保持。動的障害物の将来位置・制動距離・連続区間安全の保証なし。全身ボクセル重なりの解消は未達。実機指令なし。[自己境界分離前の比較](../../../../benchmarks/avoidance_first/results.json)。

#### 予測・不一致適応方策（実験）

- 選択: 回避方策`vrm_predictive`、CLI `--gng-strategy vrm_predictive`。既定は`graph`のまま、既存GNG/VLUTと実URDFの再利用。同梱3軸モデルの流用なし。
- 予測: 0.5 sec先までの「観測VLUT状態が安全以外へ変わる」リスク。履歴8観測・一次隣接、16入力・24中間素子のMLP。冷間時の接近傾向、期限経過後の教師によるオンライン学習、見逃し率と誤警報による閾値調整。校正済み衝突確率ではない選好値。
- 費用: 追加URDF距離照会96姿勢／腕、MLP推論1,024ノード、学習512例／更新の上限。現在判定は全VLUTノードで継続。既存KD-tree・ノード接続の再利用、ボクセルの反復登録・膨張なし。
- 実行: 現在離隔・復帰条件は`avoidance_first`と共通の0.10 m／0.14 m・0.5 sec。現在の条件を満たす場合だけ予測による先回り迂回。行き止まり時は予測だけを除外し再探索、現在の障害物・観測制約・自己干渉の解除なし。
- 制限: セッション内学習、再生区間変更時の学習初期化はセッション再開始。視野外への進入制約・未知の障害物運動・制動距離の保証は未解決。既存方式より良い回避の実証・実機検証は未達。[評価・再現設定](../../../../benchmarks/vrm_predictive/results.json)。

### CLIでのMuJoCo・GNG-VLUT実行

ワークスペースのルートで `bash scripts/run_dual_arm_simulation.sh --duration 30`。ホストからは起動済みのDockerサービス `gng_cpu`、コンテナ内からは同じROS環境入口を使用。ブラウザ・UIビルドは不要。既定はLong・GNG-VLUT・「前方・肘曲げ」、入力は既存のMID-360点群・記録ROI・記録自己マスク。

```bash
# 既存ROS入力で実行し、JSONLを保存
bash scripts/run_dual_arm_simulation.sh --duration 30 \
  --output log/artifacts/dual_arm_cli/run.jsonl

# 回避優先方策。既存ROS入力での実行
bash scripts/run_dual_arm_simulation.sh --gng-strategy avoidance_first --duration 30

# 予測・不一致適応方策。既存ROS入力での実行
bash scripts/run_dual_arm_simulation.sh --gng-strategy vrm_predictive --duration 30

# 独立ドメインでbagを再生し、Viewer受信用ブリッジも起動
bash scripts/run_dual_arm_simulation.sh --domain-id 197 --duration 0 \
  --viewer-bridge-port 8889 --viewer-observe-port 9002 --bag /rosbag/対象bag
```

Viewer: アプリサーバー8877を起動後、CLIの開始JSONにある`viewer_url`を開く。元のロボットをCLI実測JointStateへ追従表示し、ブラウザ内の物理計算・ROS状態送信はOFF。`state_source=simulation`付きのURLではモータ欄も同じJointStateの角速度・アクチュエータトルクをシミュレーション系列へ表示。欠測・失効は未表示、電流は換算係数なしでは未算出。同じ機体・同じROSドメインでのGNG回避セッションの重複は禁止。`--viewer-observe-port`指定時は同じROSドメインのTopics・点群・Tmap表示サーバーを所有起動し、URLへ接続先を設定。

設定: `--model long|standard`、`--avoidance gng|none|oscbf`、`--pose forward|home|zero`。姿勢プリセットは`app/robot-poses.json`の角度[deg]をブラウザと共用。`--pose-file`で初期姿勢、`--goal-file`で復帰目標をJSON辞書`{"L_joint7":0.15}`[rad]として部分指定。GNG・OSCBFは各方式の依存・モデルが必要。入力変更は`--points-topic`／`--roi-topic`／`--recorded-self-topic`。`--bag`指定時は必要3入力と記録TFを所有再生し、既存publisherがある場合は重複再生を拒否。記録TFは座標と親子関係を保持し、現在時刻で配信。

TF: MuJoCo実配置から`world → sim_<robot_name>/<URDFルート>`を配信。記録側は既存の基台TFを保持し、world未接続のルートだけを接続。記録基台と仮想基台は重畳配置、既存world配置が異なる場合は停止。静的TFは全件を保持し、後から接続したViewerにも配信。

出力: `start`・`state`・`summary`のJSONL。関節角[rad]・速度[rad/s]・トルク[N m]・回避状態・最大腕関節変化[deg]。`--report-sec`は出力周期のみ。既存出力ファイルへの上書きなし。終了コードは0=正常、2=起動・実行エラー、3=GNG入力未成立または`--min-motion-deg`未達。静止が正常な場面では移動量の下限指定は不要。

終了: 有限時間またはSIGINT/SIGTERMで、CLI所有のbag・GNG launch・受信用ブリッジを停止。`--duration 0`はCtrl+Cまで継続。既存のbag・ノードは停止しない。ブラウザを閉じるだけではCLIを停止しない。

### 追跡障害物の実験

起動済みの画面: `http://127.0.0.1:8877/?model=long&replay=latest`。元ワークスペースからの起動:

```bash
bash scripts/open_dual_arm_nvidia.sh --replay
```

通常画面の「実験」タブで設定 →「実験を実行」でライブ表示。既定は「連続・記録なし」。「中止」または接触・異常で終了、終了後は最終姿勢の保持。「時間指定・記録あり」では再生ボタン・時刻スライダーも利用可能。`replay=latest`は初回だけ最新記録への直接表示。リロード後は通常画面、記録の自動再生なし。既存の実験計算は継続、「実行中を表示」で表示再開。

- 設定: 実行方式、回避方式、球／二節腕、球の動き、近位部の迂回有無、通過開始リンク／追跡リンク、リンク内追跡点（点追跡のみ）、速度上昇時間または実験時間、seed、半径、速度比、加速・減速・旋回加速度。ランダムウォーク時のみ移動範囲・方向更新間隔。
- GUI初期値: 連続・記録なし、球の末端側横切り、開始リンク`L_link4`、速度上昇18 sec、速度比0.4→1.3、加速3 m/s²・減速8 m/s²・旋回10 m/s²。速度上昇後は最終速度比の維持。速度基準は無障害往復の手先最高速度、瞬間的な腕速度とは別の値。
- 往復タスク: `benchmarks/shuttle_bag/task.json`のA/Bによる、ロボット基準の左側↔前方の`L_tcp`位置・姿勢。片道約0.36 m、姿勢差約0.03°。GUIの4方式は既定`task_mode=cartesian`、7関節の同時制御。到達条件は位置15 mm・姿勢3°・手先並進速度0.12 m/s・角速度0.3 rad/s。元の全関節角と全関節の停止への復帰待ちなし。基準関節姿勢への復帰は零空間内の弱い選好、回避制約の優先。ロールアウトは既存FKの共用、指令速度0.25・0.5倍の2候補と最大2零空間方向の正負候補。全候補で既存の自己干渉・観測ガードを共用、反復ボクセル登録なし。実行中の試験への目標変更なし、次回開始時の反映。
- 連動回避: 左腕7軸。GNGロールアウトに、最大3可動リンクの接近に基づく連動退避・手先位置の変化を抑える補償の2候補を追加。固定点・無効軸の除外、既存FKの再利用。従来の最良候補より予測最小離隔を2 mm改善する場合、または従来候補が全て不成立の場合だけの採用。可動域・速度・加速度・自己干渉・観測ガードは共通。QP・OSCBFは従来の7変数最適化。腰・首・右腕・グリッパは固定。採用回数は`num_coordinated_selections`。回避に寄与しない関節の強制動作なし。
- 内部離隔: 開始姿勢・GNGロールアウト・実測監視で共通の5 mm基準。外接球は絞込み専用、近接したリンク対だけURDF衝突形状の凸包・基本形状で照合。衝突形状のない外装はvisual形状で補完。計画時は既存FKの共用、実測監視の打切りは3物理フレーム連続の不足。停止詳細は`failure_details.internal_clearance`のリンク対・距離。外部障害物・QP制約の距離近似は従来どおり。
- 制動: 予測・実指令で共通の速度・加速度・可動域制限。加速度制限による可動域制限の上書きなし。予測姿勢の可動域端への強制切詰めを廃止、範囲外候補は棄却。MuJoCo駆動とのモデル誤差・連続区間安全の保証は別の課題。
- タスク表示: 「基準線」で切替。基準線は濃い青の5px幅の破線、手先の実軌跡は黄色の細線。障害物の青い実軌跡は計算側の予測時間と同じ長さの直近区間のみ（未来予測線とは別）。予測時間のない方式・時間幅情報のない旧記録では障害物軌跡を非表示。A/Bの球・文字・姿勢軸の表示なし。現行の基準線は両端を結ぶ参考線、最適化軌道・直線追従の拘束ではない表示。状態欄は到達回数・位置／姿勢誤差。タスク情報のない旧記録への基準線の補完なし。
- 球の動き: `pursuit_mode=distal_sweep`は開始リンクの可動剛体と全末端側を通過可能範囲として、その中の剛体を均等抽選。進入点から狙点の先0.22 mまでの短い横切り経路を世界座標で保持。完了・時間切れで次の狙点へ移行、毎周期の手先への向き直りなし。最大8標本×9方向、既存FKの球中心の共用、追加ボクセル登録なし。接触無視・すり抜けの許可ではなく、実接触で終了。`link_point`は従来の点追跡。設定省略時のCLI・APIは点追跡を維持、二節腕も点追跡のみ。 画面はサーバー対応方式だけ選択可能。旧サーバーではリンク点追跡、横切りは「サーバー未対応」の表示。
- ランダムウォーク: 「球の動き」→「ランダムウォーク」、`pursuit_mode=random_walk`。球のみ、手先への追従なし。開始時の指定リンク位置を中心とする世界座標の箱内で、一定間隔のランダム目標点更新。既定は各軸±0.6 m・0.6 sec、球中心の下限Zは床側で調整。`random_walk_extent_m`: 半幅0.3〜1.5 m、`random_walk_change_sec`: 更新間隔0.1〜5 sec。同じseed・同じ条件での再現。更新ごと最大8候補、固定部への直達照合と既存ステップガードの共用、追加FK・ボクセル登録・可視グラフ探索なし。指定リンク以降が接触評価対象、近位部の迂回有無・速度上昇・加減速設定は共通。範囲と更新間隔はライブ表示・保存記録にも保持。CLI設定例: [sphere.json](../../../../benchmarks/random_walk/sphere.json)。
- 二節腕: 根元・中間とも球関節、既定リンク長0.35 m×2節。手首3D位置＋肘旋回の運動学、世界Z固定の曲げ平面なし。肩位置は固定、人体の動力学再現とは別のモデル。全端点の速度制限。入力加速度制限とガード後の実測加速度は別の値。
- 迂回: 既定ON。指定リンクより近位の剛体群・胴体・固定腕への禁止域と、床・台座への照合。固定結合カバーも同一剛体群。追跡・横切りで直進が遮られる場合だけ、近接禁止球3個×26方向の可視グラフ。再計画周期0.2 sec、実ステップには二節全体の経路ガード。対象の関節原点が近位部内の場合は対象剛体内の狙点への調整。`enable_proximal_guard=false`で迂回制約なしとの比較。
- 接触評価: MuJoCo実接触による終了。点追跡は指定リンクと固定結合部分への先端接触、球の横切り・ランダムウォークは開始リンク以降への接触が`target_touch`。二節腕の先端範囲は手首中心から障害物半径＋0.01 m。近位部・対象外・先端以外の接触は`attacker_*`・`is_attacker_loss=1`として回避失敗と分離。床・台座への接触も終了判定を維持、画面は簡潔な接触表示。勝敗文言なし。
- 可動域: 候補・指令・MuJoCo・評価で共通の有限制限。Long左腕の2・6軸は±115°、4軸は−115〜95°、7軸は±105°。URDFのcontinuous軸は暫定±180°、配線を含む実機可動域との照合は未完了。旧`enable_periodic_angles=true`指定でも制限解除なし。
- 表示: 「接触位置」は外部接触、「停止位置」は非接触打切り。接触リンク・超過関節の表示。旧記録と角度制限未反映の記録は一覧で区別。赤は外部接触。Spaceは再生／一時停止、Escは一時停止。
- 他タブ: 実験中も切替・設定参照・センサ操作が可能。実験表示中の同一ロボットへの姿勢書換とモデル切替だけ制限。終了後に他タブへ移動すると通常操作へ復帰。環境タブの机・物体設定は追跡実験の物理条件への自動反映なし。
- モータ表示: 実験の角度・実測角速度・駆動トルクを「ロボット」タブのゲージ・グラフへ共用。角加速度・ジャークは実験時刻による差分推定。実行中のモータ値は最新実測フレームから取得し、描画補間から独立。旧形式の角速度は角度差分の推定表示、未取得値の0補完なし。電流は係数設定時のみ推定。実行中の受信失効1 sec。実験の描画姿勢は通常物理・ROS受信より優先。
- 記録選択: プルダウンで選択 →「再生」で読込・再生。ローカルJSONの個別読込なし。タブ選択・記録選択だけでの再生なし。「記録を削除」は再生ボタンと同じ行、未読込の選択記録も削除可能。削除後の別記録の自動再生なし。
- 準備: 選択方式だけの初期化、ROS環境の引継ぎ。XML・メッシュ・MuJoCo版が一致する物理・内部離隔モデル、姿勢・有限可動域も一致する速度校正値、JAXコンパイル結果の再利用。保存先は`docker/runtime/pursuit-model`・`pursuit-clearance`・`pursuit-calibration`・`avoidance-jax`。変更時は再構築。準備段階・経過時間の表示。
- 実行: Long左腕7関節の実URDF・MuJoCo、制御20 Hz、同時1試験／サーバー。連続は時間打切りなし・実時間ペーシング、処理超過時の追付き連打なし。時間指定は1〜60 sec・実時間上限180 sec。ROS・実機への出力なし。ブラウザを閉じても計算は継続、再接続した画面から中止可能。
- 逐次表示: 実測20 Hz・取得間隔100 ms、受信済み区間の描画周期補間。初期バッファ120 ms、未受信姿勢の外挿なし。受信差は試験時刻、更新経過は実時間の別表示。500 msの受信停滞で表示保持、長い滞留は明示再同期。接触警告は最新受信に即時追従。実時間1倍・実機の通信遅延・安全性の保証なし。実行中の記録切替・時刻操作は無効。
- 連続表示の保持量: サーバー・ブラウザとも直近120フレーム（約6 sec）のみ。制御時間の標本は直近1,200件、平均は全期間・95百分位は直近標本。実験記録ファイルの作成なし。切断復帰は保持窓へ再同期、終了後の再生は無効。
- 保存: 時間指定モードのみ。`log/artifacts/pursuit_v2/manual_gui_<ID>/`に設定・実行ログ・指標・実測軌跡・モデル識別情報、表示用は`app/exports/`。既存結果への上書きなし。
- 削除: 一覧で選択 →「記録を削除」→ 確認。選択記録と一致する元実験ディレクトリ・表示JSONの完全削除、取り消し不可。実行中は削除不可。リロード・一覧再構築でも復帰なし、再掲載防止用の空マーカーのみ保持。旧方式の退避済みデータの自動一括消去なし。
- 中止: 連続モードは記録なし。時間指定モードは取得済み軌跡を保持、部分記録は同ディレクトリの`partial-recording.json`。完了試験の評価結果とは別の記録。
- QP不成立: 試験の即時打切りなし。腕は速度・加速度制限付きの減速保持、ボールは追跡継続、次の制御周期でQP再計算。未検査の目標追従への切替なし。接触・内部干渉・異常速度・可動域超過は従来どおり打切り。指標: `num_qp_no_solution`・`qp_no_solution_sec`・`max_num_consecutive_qp_no_solution`・`num_qp_recoveries`。

CLIでの再現:

```bash
# 高速球。対象点はL_link6のリンク内座標 [m]
bash scripts/run_dual_arm_simulation.sh --scenario chasing_sphere \
  --scenario-file benchmarks/pursuit_v2/fast_sphere.json \
  --target-link L_link6 --target-offset 0 0 0.02
```

設定例: `benchmarks/pursuit_v2/{fast_sphere,linked_arm,cbf_qp,oscbf}.json`。`--duration`で時間上書き、`--case-dir`で新規保存先。引数なしの旧追跡球設定は20 sec・速度比0.2→0.8を維持。`gng_dynamic_kernel`の初回ビルドは[コマンド](../../../../benchmarks/chasing_sphere/commands.json)。OSQPはDockerfile導入済み、OSCBFは[依存導入](oscbf_avoidance/README.md)が別途必要。

比較用JSONの`task_mode: "joint"`は従来の全関節角・関節速度による到達判定と関節誤差目的。旧CLI方式`avoidance_first`・`vrm_predictive`は`joint`のみ。新しい手先方式の保存項目は`task`・フレームの`task_goal`と誤差・`details.json`の`arrival_events`。連続実験の到達イベントは直近120件、記録ファイルの生成なし。

CLIの逐次JSONL出力は`--stream-frames`。GUIからは自動指定、既定CLI出力へのフレーム追加なし。サーバー更新後は画面の再読込が必要。

連続指定のJSON項目: `execution_mode: "continuous"`・`duration_sec: 0`・`speed_ramp_sec: 18`。停止: Ctrl+C。`--case-dir`・`--output`の併用不可。既存JSONの実行方式省略時は時間指定・記録ありの維持。

連続・球関節・完全削除の確認: [条件・集計](../../../../benchmarks/pursuit_continuous/verification.json)。181 sec相当の加速寿命確認と、実時間の手動中止・接触終了。回避成功率や実機安全性とは別の検証。

方式: GNG側は既存代表姿勢・接続を保持し、C++粗照合と候補FKを共用。点群の反復ボクセル登録なし。関節CBF-QPはOSQP、OSCBFは公式`OSCBFVelocityConfig`・CBFpy・qpaxと質量整合の作業空間目的関数。両CBFの距離制約と駆動条件は共通、全制約の残差検査後だけの採用。局所線形速度制約であり、公式論文全体の再現や連続安全性の保証ではない実験アダプタ。

可動域の正本: [LongのURDF](../../../../urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf)。1・3・5軸は角度上下限の記載なし。校正済み範囲は試験JSONの`joint_limits_deg`へ関節別の`min_position_deg`・`max_position_deg`で指定可能。有限値が必須、URDFにある有限範囲の拡張は不可。適用値・出典は`details.json`の`joint_position_limits`へ記録。暫定値はビューアでも「実機未照合」と表示。

旧構成の回帰確認（肘平面固定時）: 旧3.97 sec停止条件・手動腕型401・腕型719・前腕球701、各18 sec。[条件・全結果](../../../../benchmarks/pursuit_fixes/results.json)。保護リンク接触0件、可動域超過による打切り0件、全4条件の実行時間75.63 sec。保存軌跡20 Hzでの実測角の微小超過は最大0.026°、MuJoCoソフト拘束の誤差。

| 条件 | 評価時間 [sec] | 目標到達 [回] | 障害物の追跡停止 [sec] |
| --- | ---: | ---: | ---: |
| 旧停止条件131 | 4.49 | 0 | 0.00 |
| 手動腕型401 | 6.24 | 2 | 0.03 |
| 腕型719 | 18.00 | 0 | 14.23 |
| 前腕球701 | 18.00 | 16 | 6.94 |

旧構成の残存失敗: 131は外接球による自己干渉判定、401は`L_link6`への実接触。当時の禁止部接触は`invalid_obstacle_trajectory`、現行は上記の接触評価へ変更。完走だけでは回避成功の判定不可。

現行の確認条件・集計: [追跡条件と準備時間](../../../../benchmarks/pursuit_target/verification.json)。OSCBFの初回フレームは旧構成約7.1 sec、再利用時約2.6 sec（Docker内CLI、ROS環境初期化を除外）。回避成功率とは別の評価。二節腕の到達域外・経路制約による停滞は残存、画面で移動困難と到達域外を区別。全条件での追跡継続・連続安全性の保証なし。

旧結果: [3方式の比較](../../../../benchmarks/pursuit_v2/results.json)は肩保護漏れ・一部角度制限未反映の旧条件、現行性能の根拠から除外。[旧球条件](../../../../benchmarks/chasing_sphere/results.json)の3.97 sec停止も±π計画域を超えた旧打切り。元記録は保持。

QP継続の回帰確認: [4条件・各4 sec](../../../../benchmarks/pursuit_smooth/cases.json)。前腕701はCBF-QP・OSCBFとも不成立0.30 secから復帰、接触0件・到達4回／3回。手先401は1.17 sec／3.09 secで実接触。QP不成立の打切り解消と回避成功は別の評価。全4条件35.72 sec、事前概算60 sec。

制限: 高速の往復回避は未達。QP不成立時の減速保持は衝突回避の保証ではない動作。理想形状観測・有限周期・トルク制限付きPDと速度モデルの差、内部離隔の凸包近似。自己接触のMuJoCo物理応答はOFF、内部離隔監視による打切りは有効。実機試験前に配線を含む可動域の確定が必要。通常のGNG/VLUT既定方式の変更なし。

手先タスクの比較: [20条件・集計](../../../../benchmarks/cartesian_shuttle/verification.json)。障害物が遠い8 sec条件で4方式とも接触なし、全関節の復帰前の到達を確認。到達条件変更による回数差は性能改善の根拠とは別。接近条件では接触が残存、一部は従来の関節方式より早い接触。手先タスク導入による回避性能改善・安全性は未達。

### ロボットの用途・指令・制御

「ROS2連携 → ロボットの用途と制御」をGUI・CLI共通の設定入口として使用します。初期値はシミュレーション／ブラウザ操作／位置制御。物理の自動開始なし。設定変更で実行中の物理を停止し、開始操作で反映します。

| 設定 | 選択肢・用途 |
| --- | --- |
| 用途 | MuJoCoシミュレーション／ROS実測の表示／姿勢プレビュー（力学なし） |
| 指令元 | ブラウザ操作／ROS関節指令／ROSリーダー姿勢／回避モジュール／CLI・API |
| 制御方式 | 位置／位置・速度追従／速度／トルク |

ROS実測の表示はJointStateで描画だけを更新し、ロボットの物理を停止します。シミュレーションの表示・モータ監視にはMuJoCoの実測値を使用。現在姿勢の入力と制御指令の併用による同一ロボットの上書きは禁止します。軌道の表示再生は別の「軌道の表示プレビュー（力学なし）」で、物理指令としての再生ではありません。

| 制御方式 | 即時指令の項目 | 駆動 |
| --- | --- | --- |
| `position` | `position` | 従来の速度上限付き位置PD |
| `position_velocity` | `position`・`velocity`、任意の`effort` | 位置・速度PD＋補助トルク |
| `velocity` | `velocity` | 速度追従 |
| `torque` | `effort` | トルク直接入力。位置・速度PDなし |

指令項目は関節名とSI値の辞書。回転関節はrad・rad/s・N m、直動関節はm・m/s・N。位置・速度追従の位置と速度は同じ関節集合が必要です。未知関節・固定関節・非有限値・方式に対応しない項目・URDF上限外は受理しません。全方式でアクチュエータの駆動力上限を維持。重力・コリオリ補償はトルク方式で既定OFF、その他は既定ONで明示切替可能です。

ROS関節指令は `/sim/joint_command` の `trajectory_msgs/msg/JointTrajectory`。関節名付きの**単一点即時指令**として使用し、複数点・非ゼロの`time_from_start`・加速度指令は受理しません。選択した制御方式に対応する項目だけを指定します。購読・適用はMuJoCo側で実行し、ブラウザ描画周期には依存しません。入力が失効すると位置保持／速度ゼロ／トルクゼロへ切替。新しい有効な指令で再開します。

```bash
# GUI: シミュレーション／ROS関節指令／速度制御を選択し、開始
ros2 topic pub -r 50 /sim/joint_command trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [neck_pan_joint], points: [{velocities: [0.15]}]}'
```

リーダー姿勢のJointState入力は相対目標の生成アダプタとして保持し、一般的なROS関節指令と区別します。リーダー姿勢・現在の回避モジュールは位置制御専用です。GNGのself_voxel更新には従来どおりMuJoCo実測姿勢を使用します。

CLIはホストのNode.js 22以降から、起動中のブラウザを操作します。このディレクトリはSimulatorルートが基準です。

```bash
node tools/robot_control.mjs status
node tools/robot_control.mjs configure --usage simulation --source api --mode position
node tools/robot_control.mjs start
node tools/robot_control.mjs command --json '{"mode":"position","position":{"neck_pan_joint":0.2}}'
node tools/robot_control.mjs stop
```

複数画面がある場合は`status`のIDを`--session ID`で指定します。`command`はMuJoCoの反映通知後に成功を返します。目標変更に対応する回避モジュールの名目目標は`goal --json '{"L_joint4":-0.7}'`で指定し、回避器の出力を直接上書きしません。常駐実行は既存の`start_ros.sh --simulate`も引き続き使用できます。

HTTP APIは画面サーバーの`GET /api/robot`で状態一覧、`POST /api/robot`で操作を受理します。POSTには`X-ToPo-Robot: 1`と`Content-Type: application/json`が必要です。

```json
{"session_id":"statusで取得したID","action":{"type":"configure","config":{"usage":"simulation","command_source":"api","control_mode":"velocity"}}}
```

操作は`configure`・`start`・`stop`・`command`・`goal`。状態は`state_source`で`mujoco`・`ros`・`preview`を区別し、`has_state`で新しい実測標本の有無を確認します。未接続・複数画面の未指定・期限切れ・実行中の指令元と異なる指令はエラー。ブラウザを開き直しても前の操作は再実行しません。APIは実機出力の有効化には対応しません。

更新反映には画面サーバーとROS/物理ブリッジを再起動し、ブラウザを再読み込みしてください。

### リーダー・フォロワーの接続選択

このワークスペースでは `ros2 launch gng_vlut_system robot_follow.launch.py` を使用できます。Longが既定、USBドライバは別起動です。「ROS2連携 → リーダー・フォロワー接続」のリーダー・フォロワーは横並びのプルダウン。未接続でも標準構成を選択でき、接続失敗・切断・接続時の一覧更新でも未適用の選択を保持します。接続後に「接続を適用」で反映し、接続先の登録済み構成へ照合。未登録の組合せは適用不可。共通構成のリーダーの「指定なし」はシミュレータの手動操作。追従方法が複数ある場合は「ROS現在姿勢入力 → 描画追従／ROSリーダー姿勢 → 力学追従」を選択します。用途・指令元・制御方式は共通構成へ同期し、共通管理中の個別変更は無効です。接続関係・入力鮮度・実機状態を確認できます。管理中の姿勢入力・送受信設定は共通構成へ従い、シミュレータの状態送信は1ブラウザに限定します。モデルが共通launchと一致しない場合は接続を止めます。

選択だけでは接続を変更せず、物理・実機追従も開始しません。物理は「シミュレータの物理を開始」、実機はlaunchの出力許可と「実機フォロワーの操作」が必要です。構成変更・入力失効・切断後の実機追従は明示再開始。共通管理中の入力失効は既定0.3秒です。通常の単独シミュレータは従来の操作を維持します。
設定一覧・起動引数・実機の小動作制限は[gng_vlut_systemの共通追従構成](../../../../gng_vlut_system/docs/dynamixel_sim_control.md#srfの共通追従構成)を参照してください。

### 姿勢・TFのWebSocket送受信

「ROS2連携 → ロボットの用途と制御」で通信上限を **1～200 Hz** に設定します。初期値100 Hz。
送信は `/sim/joint_states`。現在姿勢の受信は `/joint_states`、力学追従の入力は `/leader/joint_states` が初期値で、両方とも現行の受信型は `sensor_msgs/JointState`。
「ロボット状態をROSへ送信（関節角・配置・TF）」1つで関節角・配置・TFを一括送信します。MuJoCo動作中は計算結果の位置・速度・アクチュエータ駆動力をJointStateへ送信します。effortは関節トルク／力で、接触力は含みません。物理OFFでは位置のみで、velocity／effortは空配列です。送信上限は1〜200 Hz、物理更新・描画より高い設定では同じ状態を再送します。時刻はROSブリッジの時計を使用し、MuJoCo時間による `/clock` 配信には対応しません。通常の状態送信は実機指令と独立、共通launch使用時だけ管理されたsの状態をfの追従元へ転送します。
共通管理なしの送信と受信追従は択一。共通launch管理中は検査済みの経路で送受信を併用し、関節角・配置・TFを一括配信。関節軌道再生とJointState追従も択一です。
受信は既知の関節名だけを反映。直接表示ではグリッパーとmimicだけをURDF開閉端へ飽和し、端点付近の実測角でも開閉表示を継続します。他の可動域外関節は関節名を警告し、その関節だけ前回表示を保持して正常な関節の更新を継続します。ROSの実測値への変更なし。非有限値・不明な関節名は受信停止。リーダー制御の検査・停止条件は別経路で維持。部分関節の更新に対応します。
通信は専用Workerで実行します。古い受信姿勢は蓄積せず、描画フレームごとに最新の一件を反映します。設定Hzは上限で、到達を保証するものではありません。
ブラウザ送信は表示中モデルの現在角を送ります。描画より高い頻度では同じ角度が繰り返される場合があります。
点群とTFは別経路。TF・Poseによるベース位置の受信追従は未対応です。
モデル切替・停止ボタンで関節通信を停止します。JointState追従は別パネルのクリックやページ非表示だけでは解除しません。背景タブの描画周期はブラウザに依存します。

「ROS2連携 → ロボットの用途と制御」で用途を「ROS実測の表示」、またはシミュレーションの指令元を「ROSリーダー姿勢」に設定します。一般的なROS関節指令は上記の専用トピックを使用します。現在姿勢用と力学入力用のトピックを個別に保持し、選択した方式の入力欄だけを表示します。力学入力はリーダー姿勢による相対追従か、共通構成で指定した絶対角追従かを欄名に明示します。Hz・受信ON/OFFも同じ場所で設定します。直接表示は現在姿勢用トピックを使用し、MuJoCoを停止して手動関節更新を抑止します。通信停止時は最後の姿勢を保持し、シミュレータ操作へ明示的に戻すまで手動更新を抑止します。Viewerに表示したROSロボットは独立した表示個体で、この選択によって統合されません。

物理フォロワーは`/leader/joint_states`を受信し、ロボットを左ダブルクリックして「物理を有効にする」をONにし、「物理を開始」で追従を開始します。通常の単独入力と共通構成の相対追従では、腕の最初の有効なリーダー姿勢と物理実姿勢を基準に保存し、角度差をMuJoCoのPDモータ目標へ入力。共通構成で `follow_mode: absolute` の場合は受信角を目標角へ使用。速度・トルク指令を含む一般的な制御コマンドの受信とは別の機能。グリッパーは換算済み開度へ初回入力から追従。リーダーの入力専用開閉端設定は[gng_vlut_systemの手順](../../../../gng_vlut_system/docs/dynamixel_sim_control.md#実機リーダーフォロワー制御)を参照。描画姿勢へのリーダー角度の直接代入なし。URDFの可動域・指令速度・トルク上限と接触計算は既存の物理経路を使用します。入力に実時間のheader stampが必要、旧stamp再送による鮮度更新なし。

リーダー入力・物理実測・目標送信は描画周期から独立。入力失効1秒、通信OFF・切断・可動域超過で保持して追従停止。物理側にも独立した失効監視を配置し、ブラウザの更新停止でも旧目標を解除。通信復旧だけでは再開せず、「停止」→「物理を開始」で明示再開と基準姿勢の更新。ページ非表示では物理セッションを終了します。実機への指令なし。

実機Dynamixelの読取り・ID対応は[リーダー・フォロワー起動手順](../../../../gng_vlut_system/docs/dynamixel_sim_control.md#実機リーダーフォロワー制御)を参照。実機出力OFFでのリーダー配信が可能。新しいstamp付き通信と物理監視の反映には、下記のブリッジ再起動とページ再読み込みが必要です。
操作対象ロボットの表示と不透明度は「ロボット → 操作対象ロボット」で調整できます。元の色・金属感・粗さを保持し、床などの共有材質は変更しません。ロボットの非表示は仮想センサの可視メッシュにも影響します。

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

点群送信時は取得開始時の配置・関節角をメタデータに保持し、追加センサフレームのTFだけを点群と共通stampで配信します。
有効XYZの点群は配置の逆変換を適用したbase_footprint座標、画素対応点群はカメラ光学座標です。
配置・TF・関節角の定期送信は「ロボットの用途と制御」の1つの送信スイッチで開始・停止します。個別の送信選択はありません。同じスナップショットから、共通の1～200 Hz・ROS時刻で `/sim/base_pose`、`/sim/joint_states`、`/tf`・`/sim/tf` へ配信します。world・基台・URDFリンクTFはこの経路、追加センサTFは点群取得経路が担当します。
受信時のROS時計を使用するため、実センサとの取得時刻同期を保証する仕組みではありません。

TFは標準の `/tf` と互換用の `/sim/tf` に同じ内容を配信します。専用配信だけにする場合は一括起動の `bash start_ros.sh --tf-topic /sim/tf` を指定し、利用ノードで `/tf:=/sim/tf` をremapしてください。UI用HTTP・ブリッジHTTP・その次のWebSocketポートは重複不可です。
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

実機への指令はありません。GNG/VLUTによる自己除去・回避は、上記の物理タブのGNG-VLUTオプションで使用します。
移動配置を使った従来VM・AIタブとの統合は未検証です。この座標整合の検証対象は独立ROS2連携タブです。
回帰試験：ROS環境で `python3 -m unittest discover -s integrations/ros2 -p test_robot_exchange.py`。

2026-10-01までの検証は標準モデル・ROS Humble。3種の点群の実受信、848×480全画素の深度・XYZ対応と共通時刻、移動配置（XYZ=(0.3,-0.2,0.1) m、yaw=25 deg）での点群・TF整合を確認。ROS軌道→ブラウザ首Yaw 0.1 rad→ROS関節状態の往復、未知関節・過速度・範囲外の拒否、停止操作を確認し、Python回帰15件が成功しています。

LongのRGB-D実送信・軌道往復、持続送信レート、大規模車両メッシュの性能、実RealSense購読アプリとの互換性、RGB-DとGNG/VLUTの統合運転、実機動作は未検証です。

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
