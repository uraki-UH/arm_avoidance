# 別PCでの双腕ブラウザ・ROS 2の起動（Ubuntu・NVIDIA不要）

対象: Ubuntu・Docker Compose・WebGL 2対応ブラウザ。ROS・Node.jsはコンテナ側、NVIDIA専用環境は不要。
共有先には非公開リポジトリのアクセス権が必要。共同開発の取得先は `main`、変更箇所と確認方法は[CONTRIBUTING](../CONTRIBUTING.md)。

## ソース取得・基本構築

```bash
git clone --branch main https://github.com/uraki-UH/arm_avoidance.git
cd arm_avoidance
bash scripts/dual_arm_cpu.sh build
bash scripts/dual_arm_cpu.sh check
```

リポジトリ全体を配置。Simulator単体では共有UI・独自メッセージが不足。
checkはCPU依存・点群処理・短時間MuJoCoの確認で、回避性能の判定ではない。
構成は[Compose](../docker/compose.simulator_cpu.yaml)の`x-cpu-workflow`を参照。起動窓口は[dual_arm_cpu.sh](../scripts/dual_arm_cpu.sh)。
旧固定版`v0.1.0-cpu-preview.1`はCPU基本構成のみ。以下の`demo`操作は未収録。

## 学習データ・bagなしの回避デモ

```bash
bash scripts/dual_arm_cpu.sh demo
```

Long左腕の往復と追跡球、既存のCBF-QP方式。設定は[同梱シナリオ](../benchmarks/cpu_demo/chase.json)。GNG/VLUT方式とは別。
結果は`log/artifacts/cpu_demo/`。ブラウザ起動後、実験タブの記録一覧で再生。デモ再実行の結果は別保存。

## ブラウザとROS 2の起動・終了

```bash
bash scripts/dual_arm_cpu.sh start
```

表示: `http://127.0.0.1:8877/?model=long`。終了: 起動端末のCtrl+C、片付けは`bash scripts/dual_arm_cpu.sh stop`。
開発用gng_cpuとは別構成。同じポートの同時使用は禁止。

## 画面なしのCPU実行・ROS確認

```bash
bash scripts/dual_arm_cpu.sh simulate --avoidance none --duration 3
bash scripts/dual_arm_cpu.sh shell
```

上記操作はホスト側で実行。`simulate`の相対パスはリポジトリルート基準、出力省略時は端末表示。
shellはROS環境読込み済み。ブラウザのセンサ送信は明示的に有効化。

## GNG／VLUT回避まで使う場合

`x-cpu-workflow.enable_gng: true`に設定し、同じ`build`／`check`を使用。機体・ビルド対象・確認時間も同じYAMLで設定。
腕のGNG/VLUT・Viewerが構築・確認の対象。AIS-GNGライブ学習は対象外。
Longの学習バイナリはGitに未収録。機体URDF・設定に対応する左右のgng.bin／vlut.binを別途配置。
設定・生成は[学習手順](../gng_vlut_system/README.md#双腕学習viewerの既定モデル)、実行は[共通ROS回避](../gng_vlut_system/docs/dual_arm_simulation.md#センシング--ros回避--実行先)を参照。

## 起動できない場合

| 症状 | 確認先 |
| --- | --- |
| Docker接続失敗 | ホストのDocker起動・利用権限 |
| WebGL初期化失敗 | ブラウザGPU設定。画面なしcheckで切り分け |
| ポート競合 | 8877・8879・8880の既存サービス |
| 学習データ不足 | YAMLのGNG有効設定とcheck |
| UIソース更新が未反映 | shell内のSimulatorルートでnpm ci && npm run build |

CPUアーキテクチャ差・実機回避は未検証。機能別依存は[ROS連携](../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md)を参照。

## 開発用gng_cpuとViewer

CPU専用構成とは別のCompose構成。同じポートで同時起動しないこと。

```bash
docker compose up -d gng_cpu
docker compose exec gng_cpu bash
```

```bash
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=topo_dual_arm_max_long.yaml
```

Frontendはホストから`docker compose --profile manual up -d frontend`。
GPUブラウザは`bash scripts/open_viewer_nvidia.sh`。[Viewer単体の構築](../ToPoFuzzy-Viewer/doc/ROS2_BRIDGE.md)。

## 機能別の入口

| 用途 | 手順・設定 |
| --- | --- |
| ブラウザ・MuJoCo・ROS送受信 | [Simulator](../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/README.md)・[ROS連携](../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/integrations/ros2/README.md) |
| センシング→ROS回避→実行先 | [双腕シミュレーション](../gng_vlut_system/docs/dual_arm_simulation.md) |
| bag・データセット | [bag再生](../scripts/play_bag.sh)・[点群確認](../docker/rosbag_pointcloud_validation.md) |
| MID-360実機 | [接続・取付TF](../hardware/mid360/README.md) |
| RealSense・Dynamixel入力 | [実機入力](hardware_guide.md) |
| リーダー・フォロワー／重力補償 | [制御手順](../gng_vlut_system/docs/dynamixel_sim_control.md)。実機駆動・重力支持は未検証、校正・腕の支持が必要 |
| GNG学習・VLUT・ロボット表示 | [GNG-VLUT](../gng_vlut_system/README.md) |
| 上面／ボクセル把持候補 | [把持機能](../grasping_system/README.md)。同じ出力先の候補生成を重複起動しないこと |
| メッシュから完全表面テンプレート | [テンプレート生成](../gng_vlut_system/docs/MESH_SURFACE_TEMPLATE.md) |

## FuzzBotの表示専用起動

```bash
cd /ros2_ws
colcon build --paths src/fuzzbot_gng/fuzzbot/fuzzbot_description --packages-select fuzzbot_description --symlink-install
source install/setup.bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py params_file:=fuzzbot.yaml
```

親の`COLCON_IGNORE`は維持。走行指令・実機接続・学習・自己認識ボクセルは未起動。
設定: [fuzzbot.yaml](../gng_vlut_system/config/fuzzbot.yaml)。表示にGNG・VLUTは不要。

### FuzzBotの移動状態グラフ

```bash
ros2 launch gng_vlut_system mobile_state_graph.launch.py
```

Viewer表示: `/fuzzbot/state_graph/markers`。設定: [fuzzbot.yaml](../gng_vlut_system/config/fuzzbot.yaml)の`motion_graph`。
状態は`(x,y,yaw,v,omega)`、有向エッジは一定加速度による遷移。生データは`/fuzzbot/state_graph/data`。
初回生成・保存後は整合性検査付き再利用。再生成は`enable_rebuild:=true`、生成だけは`build_only:=true`。
平坦床・横滑りなしの差動二輪モデル。全状態被覆・障害物照合・実機指令・現在状態の検索は対象外。
速度制限はURDF由来、加減速上限は仮値。

## 大容量PointCloud2再生用UDPバッファ

```bash
sudo install -m 0644 docker/ros2_fastdds_udp_buffers.conf /etc/sysctl.d/99-ros2-fastdds-udp-buffers.conf
sudo sysctl -p /etc/sysctl.d/99-ros2-fastdds-udp-buffers.conf
```
