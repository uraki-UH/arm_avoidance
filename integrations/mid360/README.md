# MID-360実機 → ROS 2

対象: Linuxホスト、Docker Compose、ROS 2 Humble。公式Livox SDK2・livox_ros_driver2を固定リビジョンでReleaseビルド。GNGコンテナとは独立したセンサ用サービス。

## ドライバの導入（実機接続不要）

実行場所: **ホストPCの`uraki_ws`ルート**。GNGコンテナ内のcolconやルートの`docker compose build gng_cpu`では、このドライバの導入は対象外。

```bash
docker compose build mid360
```

ルートComposeから専用設定を共用。`mid360`の明示指定でビルド・起動可能、`--profile`の追加不要。通常の`docker compose build`・`docker compose up`では対象外。通常サービスとまとめたビルドは`docker compose --profile lidar build`。

生成物: `mid360:local`。SDK2は`/usr/local/lib`、ROS 2ドライバは`/opt/livox_ws/install`へ導入。依存パッケージ・SDK・ドライバのコンパイル・設定テストまでDockerfile内で実行。ビルドは2並列、実機通信・IP設定・既存GNG環境の変更なし。初回は依存パッケージのダウンロードとビルドに時間が必要。

導入だけの確認（ネットワークなし、ドライバ起動なし）:

```bash
docker run --rm --network none mid360:local ros2 pkg prefix livox_ros_driver2
docker run --rm --network none mid360:local python3 /opt/mid360/test_config.py
```

期待結果: `/opt/livox_ws/install/livox_ros_driver2`、設定テストの`OK`。`Package not found`をGNGコンテナ内で確認しても、専用イメージの導入失敗を意味しない。

## 固定IP設定込みの自動起動

ホストPCの`uraki_ws`ルートで実行。イメージのビルドと[mid360.yaml](mid360.yaml)の`host_ip`・`lidar_ip`設定が前提。MID-360直結、ネットマスク`/24`を対象。

```bash
python3 integrations/mid360/start.py
```

動作: YAML読込み → 物理リンクのある有線LANを選択 → PCの固定IPv4設定 → `docker compose up mid360`。LAN名の手入力は通常不要。IP設定時のみ`sudo`認証が必要、Dockerは通常ユーザーで実行。ホストにPython 3・PyYAML・NetworkManager・iproute2・Docker Composeが必要。`No module named yaml`の場合は`sudo apt install python3-yaml`。

- 有線LANが1個だけ接続中: 自動選択。複数候補・未接続の場合は変更前に停止。
- 複数LANからの明示選択: `python3 integrations/mid360/start.py --interface <DEVICE名>`。
- 既存の別IPv4・グローバルIPv6・デフォルト経路・別LANとのサブネット競合: 自動変更せず停止。
- 固定IPが設定済み: 既存設定を使用。未設定時は専用プロファイル`mid360-direct`の新規作成・有効化。既存の同名プロファイルが不整合なら上書きせず停止。
- 終了: Ctrl+Cでドライバ停止。PCのIP設定・専用プロファイルは維持。IP解除は`sudo nmcli connection down mid360-direct`。PC再起動後も同じスクリプトで起動可能。

変更前の確認だけなら:

```bash
python3 integrations/mid360/start.py --check
```

制限: LANの選択はリンク状態に基づくもので、LiDARの個体認証・SN取得・センサ側IP変更・相手機器のIP重複検出は対象外。接続先は下記手順で事前確認。通常の`docker compose up mid360`だけではホストのIP変更なし。Dockerへのホストネットワーク管理権限の追加なし。

## 接続先の確認・手動設定

以下はホストPCのBashで実施。MID-360を適合電源・Ethernetで接続し、初回確認はPCとの直結を推奨。Docker内の設定だけではPCのIPv4割当は未変更。

### 1. シリアル番号と接続先の識別

| 識別情報 | 確認場所・用途 |
| --- | --- |
| LiDARのシリアル番号（SN） | 本体の製品ラベル・SN表示。機体の識別と工場出荷時IPの確認 |
| LiDARのブロードキャスト番号 | 本体背面のQRコード、またはLivox Viewer 2のDevice Manager。ラベルのSN表記と区別 |
| PCのシリアル番号 | PC本体のラベル、BIOS/UEFI、下記DMIコマンド。機材管理用、IP設定には不要 |
| LANインターフェース名 | `nmcli device status`のDEVICE欄。コマンドの通信先指定 |
| LANのMACアドレス | `ip link`の`link/ether`欄。PC本体・LiDARのSNとは別の識別情報 |

PCのシリアル番号のCLI確認（コンテナ外）:

```bash
sudo dmidecode -s system-serial-number
```

DMI非対応・メーカー未登録では空欄や仮の文字列になる場合あり。その場合は本体ラベル・BIOS/UEFIで確認。シリアル番号の公開ログへの貼付は不要。

MID-360の工場出荷時IPは`192.168.1.1XX`（XXはSNの末尾2桁）、ネットマスクは`255.255.255.0`。SN末尾41なら`192.168.1.141`。変更済み機体では実際のIPを優先。ARPから完全なSNの復元は不可。Viewer接続後の識別情報との照合、または本体ラベルで確認。[Livox公式マニュアル](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)

### 2. 接続LANの選択（機種固有名の固定なし）

```bash
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
ip -brief link
read -r -p 'MID-360を接続したDEVICE名: ' mid360_iface
ip link show dev "${mid360_iface:?DEVICE名が未入力}"
nmcli -f GENERAL.DEVICE,GENERAL.STATE,GENERAL.CONNECTION,WIRED-PROPERTIES.CARRIER,IP4.ADDRESS device show "$mid360_iface"
```

選択対象: MID-360のケーブルを接続した`ethernet`。`enx…`等はLAN名であり、PCのSNではない。複数LANがある場合は対象ケーブルの抜差し前後の`CARRIER`・`LOWER_UP`を比較し、Wi-Fi・Dockerブリッジを除外。既存通信への影響を確認したうえで抜差し。最初のethernetを自動採用しない。

以後は同じ端末の`$mid360_iface`を使用。新しい端末では再選択。`CARRIER: on`は物理リンクあり、`connecting (getting IP configuration)`はIP取得待ちであり、センサの識別・点群受信成功ではない。

### 3. パケットからLiDARのIPを確認

PCのIPv4未設定でも受信パケットの確認は可能。`tcpdump`未導入の場合のみ`sudo apt install tcpdump`。

```bash
sudo timeout 10s tcpdump -nn -e -i "${mid360_iface:?先にLANを選択}" 'arp or udp'
```

10秒で終了。`timeout`の終了コード124は時間制限による終了。パケットなしの場合は電源・配線・対象LANを確認し、IPを決め打ちしない。

提示ログの読取り例（2026-10-09、ユーザー取得）:

```text
ARP, Request who-has 192.168.1.5 tell 192.168.1.141
```

- `tell 192.168.1.141`: ARP要求を送信した機器のIP。MID-360直結の確認済みLANなら`lidar_ip`の値。
- `who-has 192.168.1.5`: 送信元が探している相手のIP。今回のPC側`host_ip`の設定候補。PCへの割当済みという意味ではない。
- `0.0.0.0.68 > 255.255.255.255.67`のDHCP要求: PCのMACと一致する場合、PC側の自動IP取得要求。LiDARのIP・SNではない。

ARP要求の受信はEthernet通信の証拠であり、点群受信の証拠ではない。スイッチ経由・複数機器では送信元MAC・本体情報も照合。SN末尾41は工場出荷時IPとの整合候補にとどまり、SNそのものの確認とは別。

### 4. PCの固定IPv4をCLIで設定

今回の直結構成の例: PC `192.168.1.5/24`、LiDAR `192.168.1.141`。別機体では前項の確認値に置換。同じIPの機器がないこと、Wi-Fi・他のLANとサブネットが競合しないことを確認。Wi-Fiや遠隔操作に使用中の接続は変更対象外。

```bash
ip -4 address
ip -4 route
nmcli -f NAME,UUID,TYPE,DEVICE connection show
```

既存接続を残し、LiDAR専用の接続プロファイルを新規作成。`mid360-direct`が存在しない初回のみ実行。存在する場合は`nmcli connection show mid360-direct`で設定を確認し、重複作成を回避。

```bash
sudo nmcli connection add type ethernet \
  con-name mid360-direct ifname "${mid360_iface:?先にLANを選択}" \
  ipv4.method manual ipv4.addresses 192.168.1.5/24 \
  ipv4.never-default yes ipv6.method disabled connection.autoconnect no
sudo nmcli connection up mid360-direct
```

適用対象: 選択LANのみ。同じLANで使用中の接続は切替えにより切断。ゲートウェイ・DNSの指定なし、`ipv4.never-default yes`でWi-Fi等のデフォルト経路を維持。プロファイルは保存されるが自動接続は無効。再起動後の利用時も`sudo nmcli connection up mid360-direct`で有効化。[NetworkManager公式設定仕様](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-nmcli.html)

確認:

```bash
ip -4 addr show dev "${mid360_iface:?先にLANを選択}"
ip -4 route get 192.168.1.141
ping -I "$mid360_iface" -c 3 -W 1 192.168.1.141
ip -4 neigh show dev "$mid360_iface"
```

期待結果: PCに`192.168.1.5/24`、センサへの経路に選択LANと`src 192.168.1.5`。ping応答・ARP解決だけではROS点群受信は未確認。ping無応答だけで故障と断定せず、パケットとドライバログも確認。

専用接続の解除は`sudo nmcli connection down mid360-direct`。元の接続へ戻す場合は、上で控えた元プロファイルのUUIDを`sudo nmcli connection up uuid <元のUUID>`へ指定。既存プロファイルの削除・NetworkManager全体の再起動は不要。

### 5. YAMLへの反映とドライバ起動

[mid360.yaml](mid360.yaml)の2項目を実値へ変更。今回のログに対応する例:

```yaml
host_ip: "192.168.1.5"
lidar_ip: "192.168.1.141"
```

`host_ip`はPCの選択LANへ実際に割り当てた値、`lidar_ip`はセンサ側の値。YAML編集だけではPC・センサ本体のIPは未変更。IP設定だけの変更ではイメージ再ビルド不要。

ワークスペースルートから起動前確認（設定・PCへのIP割当のみ、LiDAR通信・ノード起動なし）:

```bash
docker compose run --rm --no-deps mid360 \
  python3 /opt/mid360/mid360.launch.py --check-config
```

`host_ip`・`lidar_ip`未設定なら対象キー名を表示して終了。`host_ip`のローカルbind失敗時は有線LANのIPv4割当を確認。`OK`でもセンサ本体の認識・通信成功の意味ではない。

`Cannot assign requested address`（Errno 99）は、指定した`host_ip`が起動環境のローカルIPにない状態。本構成は`network_mode: host`のためPC側を確認。YAMLに値があっても、接続プロファイルがDHCPのまま・固定IPプロファイルが未有効化なら起動不可。前項の`nmcli connection up mid360-direct`と`ip -4 addr`で割当を確認後に再実行。Dockerイメージの再ビルドでは解消不可。

ワークスペースルートから起動:

```bash
docker compose up mid360
```

終了: 同じ端末でCtrl+C、または別端末から`docker compose stop mid360`。停止済み専用コンテナのみの削除は`docker compose rm -f mid360`。ルートの`docker compose down`はGNG等も停止するため不使用。ホストネットワーク使用、PCのNIC設定の自動変更なし。IP未設定・ホスト未割当なら起動時エラー。既存のLivoxドライバとの同時起動不可。自動再起動なし。

設定の反映: `mid360.yaml`は読み取り専用マウント、編集後の停止・再起動で反映。Dockerfile・launch・entrypoint変更時は`build mid360`後に再起動。起動ログの確認は`docker compose logs --tail=80 mid360`。

旧形式の`docker compose -f integrations/mid360/compose.yaml …`も利用可能。ただしルートComposeとは別プロジェクト扱いのため、旧形式で起動中の場合は旧形式で停止してから切替え。同時起動不可。

| 出力 | 型 | frame |
| --- | --- | --- |
| `/sensors/mid360/points` | sensor_msgs/msg/PointCloud2 | mid360_link |
| `/sensors/mid360/imu` | sensor_msgs/msg/Imu | livox_frame（使用中の公式ドライバ内の固定値） |

点群: 既定10 Hz、XYZ[m]・intensity等。色なし。IMU周期は点群のpublish_freqとは別。センサは1台を対象。

疎通確認はGNGコンテナ内でROS環境を読み込み:

```bash
docker compose exec gng_cpu bash
source /ros2_ws/install/setup.bash
ros2 topic info /sensors/mid360/points -v
ros2 topic hz /sensors/mid360/points
ros2 topic echo /sensors/mid360/points --once --field header
```

型の検出だけでなく、継続受信・frame・時刻を確認。受信側とドライバの `ROS_DOMAIN_ID` を統一（既定0）。別PCでDDS受信する場合は双方で同一Domain、`ROS_LOCALHOST_ONLY=0`、DDS通信を許可。LiDAR直結側ではUDP 56101/56201/56301/56401/56501の受信とセンサ向け通信を許可。ファイアウォール全体の無効化は不要。

## 取付TFとボクセル化

`mid360.yaml` の `pos` [m]・`rot_deg` [roll,pitch,yaw、deg] は親frameから測定原点への変換。単独利用時のみ `enable_mount_tf: true` で配信。Viewer連携時はfalseを維持。SDK側のextrinsicはゼロ固定のため二重変換なし。

Viewer連携時のTF接続: `topo_dual_arm_max_long/base_link → … → topo_dual_arm_max_long/chest_lidar_link → mid360_link`。Viewer側の機体YAMLで取付TFを配信するため、専用ドライバの `enable_mount_tf` はfalse。URDF側は機械取付位置・pitch=45°、最後のTFは計測軸との対応としてyaw=90°。計測+X=取付+Y、計測+Y=取付-X、+Zは共通。45°の二重適用なし。腰関節の受信角に追従。

座標軸の根拠: [Livox公式マニュアル](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)のCoordinates図（冊子12頁）でコネクタ側は計測-X、[CAD由来メッシュ](../../urdf/topo_dual_arm_max_long/meshes/chest_lidar.json)では取付-Y。90°はこのURDFとの接続値であり、MID360一般の出力補正ではない。点群は公式の計測座標のまま、変換はROS標準の`tf2_ros/static_transform_publisher`。独自の点群回転処理なし。

腰角0の公称位置はbase_link基準で約[0.069326, 0, 0.352347] m。計測原点の位置差分は未校正。IMUは公式仕様上、点群と同じ軸方向・別原点。使用中ドライバのIMUはframe固定・加速度がg単位のため、融合用途ではframe接続とSI単位への変換も別途必要。

GNG/VLUT用のViewer連携は `gng_vlut_system/config/topo_dual_arm_max_long.yaml` で選択:

```yaml
mid360:
  enable_input: true
  points_topic: "/sensors/mid360/points"
  enable_mount_tf: true
  parent_frame_id: "chest_lidar_link"
  frame_id: "mid360_link"
  pos: [0.0, 0.0, 0.0]
  rot_deg: [0.0, 0.0, 90.0]
```

`enable_input` の既定はfalse。trueでMID-360入力と環境ボクセル化を有効化し、PointCloud2のframeを使用。取付親frameにはrobot_nameを自動付与。ドライバの起動・IP設定は専用Compose側。通信設定とViewerの入力選択を分離。

TF設定変更の反映: 起動中の`gng_viewer_bridge.launch.py`を停止して再起動。上記の機体YAMLを明示指定する場合はビルド不要。Viewer連携で取付TFを配信しないMID360ドライバの再起動は不要。

GNGコンテナ内で起動:

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=topo_dual_arm_max_long.yaml
```

明示したlaunch引数 `environment_input_topic`・`enable_environment_voxelization` はYAMLより優先。実機では `use_sim_time:=false`（既定）を使用。

前提: longのURDF・左右GNG/VLUTデータ、実姿勢の関節情報（waist_jointを含む）、`topo_dual_arm_max_long/base_link → … → mid360_link` のTF。既にrobot_state_publisherが稼働中なら `enable_robot_state_publisher:=false` を追加して二重配信を回避。固定センサの場合は校正先の親frameを変更。

```bash
ros2 run tf2_ros tf2_echo topo_dual_arm_max_long/base_link mid360_link
ros2 topic hz /topo_dual_arm_max_long/self_filter_roi_voxels
```

点群とロボットの重なり、自己除去後の手・物体の残存、移動後のボクセル消去を確認。実機点群では壁時計を使用。シミュレーション時刻への混在や移動中のスキャン歪み補正は本構成の対象外。ここまでが認識入力の準備であり、実機アームへの回避指令出力は含まない。

## 検証

ホスト自動設定: 選択・競合回避・プロファイル再利用・確認専用モード・設定失敗時の起動中止をモックテストで検証。実PCの`start.py --check`でLAN選択・YAML読込みを確認。固定IP変更と実機ドライバ起動を通した検証は未実施。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 integrations/mid360/test_start.py
```

導入確認: 2026-10-09、Linux amd64・ROS 2 Humbleで専用イメージのReleaseビルド成功。ネットワークなしの一時コンテナでROSパッケージ・実行ファイルの検出、共有ライブラリ読込み、設定テスト5件、launch引数表示を確認。IP未設定の事前チェックは終了コード1と対象キー名を表示。検証コンテナは終了・削除済み、実機ドライバの起動なし。

```bash
docker compose --profile lidar config --quiet
docker run --rm --network none mid360:local python3 /opt/mid360/test_config.py
docker run --rm --network none mid360:local ros2 launch /opt/mid360/mid360.launch.py --show-args
```

取付TFの検証（GNGコンテナ、他の処理と異なるROS Domainで実施）:

```bash
docker compose exec gng_cpu bash -lc 'source /opt/ros/humble/setup.bash; ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 python3 /ros2_ws/src/integrations/mid360/test_mount_tf.py --urdf /ros2_ws/src/urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf'
```

確認済み: 機体YAMLのyaw=90°、腰角0°・90°でbase_linkからの位置・計測+X軸・水平な機体の重力方向。検証用TFノードは終了時に停止。ROS Domain 97は検証専用とし、他用途で使用中なら変更。

実機の読取り確認（2026-10-09）: 10秒間に点群100件・IMU 1,994件、点群frame=`mid360_link`。同じIMU平均への数値変換でbase_linkの+Zとの角度は補正前60.04°、yaw=90°補正後0.67°。位置校正・絶対方位・時刻同期・実点群での自己除去・再起動後の画面表示は未検証。既存のドライバ・Viewerプロセスは停止・変更なし。

公式仕様: [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)、[Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)。ROS 2では `xfer_format=0` のPointCloud2を使用。
