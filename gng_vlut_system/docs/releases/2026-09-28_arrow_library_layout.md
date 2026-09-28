# 2026-09-28 - 共通矢印ライブラリの配置整理

## 1. 要約

ルート直下の`arrow_visualization/`を`libs/arrow_visualization/`へ移動。
GNG・把持表示・Viewerで共用するヘッダー専用ROSライブラリとして配置。

- パッケージ名・include名・ヘッダー内容を維持。移動前後の5ファイルのSHA-256一致。
- ViewerのDockerfile・単独配布スクリプト、CPU用Composeのホスト参照先を更新。
- 共通矢印仕様のソース配置を更新。旧位置の互換symlinkなし。

## 2. 条件・検証

- colconで新配置から`arrow_visualization`を検出。
- 移動したパッケージの旧CMakeパスを`--cmake-clean-cache`で再構成し、単体ビルド成功。
- Release全体ビルド28パッケージ成功。起動したビルドプロセスは全終了。
- CPU用Composeの`docker compose config --quiet`成功。
- Viewer配布スクリプトの`bash -n`成功。Dockerイメージ・配布アーカイブの再生成は未実施。
- 稼働コンテナの再作成・ROS起動・既存プロセスの停止操作なし。
- 単独CPU Composeを使う環境では次回作成時から新ホストパスを使用。

コンテナ`gng_cpu_container`内の`/ros2_ws`でROS Humble・install環境を読込み後に実行。

```bash
colcon build --packages-select arrow_visualization --symlink-install --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
