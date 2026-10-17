# arm_avoidance

ROS 2 Humbleを使った双腕ロボットのGNG/VLUT・回避・把持・ブラウザ表示の開発用ワークスペース。
既定機体はToPoDualArm-Max Long。CPU計算用Dockerと、ホストのWebGL 2対応ブラウザが必要。

## 取得・実行

[RUN_GUIDE](MD/RUN_GUIDE.md)のソース取得・構築手順を参照。構築後、ホストのリポジトリルートで:

```bash
bash scripts/dual_arm_cpu.sh check
bash scripts/dual_arm_cpu.sh demo
bash scripts/dual_arm_cpu.sh start
```

表示: `http://127.0.0.1:8877/?model=long`。停止: Ctrl+C、片付けは`bash scripts/dual_arm_cpu.sh stop`。
`demo`は学習データ・bag不要のCBF-QP追跡球実験。結果は`log/artifacts/cpu_demo/`、可視化はブラウザの実験タブ。
GNG/VLUT方式には別途学習データが必要。最小デモの成功は回避性能・実機安全性の保証ではない。

## 共同開発

変更箇所・部品間の契約・PR前の確認は[CONTRIBUTING](CONTRIBUTING.md)。起動手順の正本は[RUN_GUIDE](MD/RUN_GUIDE.md)。

```text
arm_avoidance/
├── MSG/                         ROSメッセージ・サービス定義
├── ais_gng_cpu/                 センサ点群のAiS-GNG学習
├── pointcloud_sampling/         点群の共通登録・サンプリング
├── gng_vlut_system/             ロボット姿勢GNG・VLUT・ROS計画制御
│   ├── include/, src/           C++アルゴリズム・ROSノード
│   ├── scripts/                経路探索・入力処理の共通実装
│   └── launch/, config/        起動構成・機体パラメータ
├── grasping_system/             把持候補・保持物体・把持体積
├── libs/                        共通ボクセル・表示・モータ計算
├── ToPoFuzzy-Viewer/            ROS結果の表示
│   ├── frontend/               ブラウザUI・共通描画
│   └── backend/src/            ROS集約・WebSocket配信
├── ToPoDualArmMax_SourceDelivery_20260928/
│   └── ToPoDualArmMax-Simulator/
│       ├── app/, src/          ロボット・センサ表示、Viewer埋込み
│       └── integrations/ros2/  MuJoCo・回避方式・ROS接続アダプタ
├── urdf/                       ロボット形状・関節定義
├── benchmarks/cpu_demo/        共有可能な最小実験設定
├── docker/                     Docker・Compose構成
├── scripts/dual_arm_cpu.*      ホスト／コンテナ共通の操作入口
├── .github/workflows/cpu.yml   PR・mainのCPU動作確認
└── MD/RUN_GUIDE.md             取得・構築・起動手順
```

構成は既存のROSパッケージ境界を維持。Git管理外の実験データ・個人用プロジェクトは共有範囲外。
