# 2026-09-23 - GNG観測APIのヘッダー分離

## 1. 要約

基本APIから観測用型への依存を分離。観測機能を使うコードだけで`fuzzrobo/libgng/observation_api.h`を指定。

- 通常CPU版と実験版`gng_spatial_tree`の観測APIを専用ヘッダーへ移動。
- API実装、ROSコンポーネントのCPU側、観測・重点入力テストのincludeを更新。
- 観測APIテストは新ヘッダー単独、重点入力テストは基本・観測両ヘッダーの併用。
- [観測APIの利用説明](../../../ais_gng_cpu/docs/observation_support.md#ライブラリapi)を更新。

- `fuzzrobo/libgng/observation_api.h`。基本型の参照用に`api.h`を内包。

- 基本APIしか使わない翻訳単位への、観測角度範囲・画素ビューの推移的なinclude。

**削除**

- `api.h`内の`gng_observation_input`、`gng_observation_frame`、観測用関数3件の宣言。専用ヘッダーへ移動済みで、機能の削除なし。

## 2. 条件・検証

- 観測APIを利用する外部ソースでは、新ヘッダーの明示的なincludeが必要。
- 構造体のフィールド・配列・既定値、関数シグネチャ、Cリンケージ、実行処理の変更なし。
- GPU版の公開ヘッダーは変更なし。ROSコンポーネントの新includeはCPU条件内のみ。
- 実行時間の改善施策ではなく、公開ヘッダーの依存整理。

- 両CPUヘッダー群について、基本ヘッダー単独・観測ヘッダー単独・両方のinclude順2通りの計8コンパイル確認が成功。
- コンパイラの依存一覧で、`api.h`から観測ヘッダーへの依存がないことを確認。
- 通常CPUライブラリのReleaseビルド・隔離先へのインストール・CTest 10件が成功。
- 実験版のgrid・spatial・bsp3d各方式の観測／重点入力APIをビルドし、CTest 6件が成功。
- ROS側の`ais_gng_cpu`と`ais_gng_component_cpu`のReleaseビルドが成功。
- ROS画素参照のCTest 1件（GTest 6件）が成功。
- 検証プロセスは全終了。今回作成した一時ビルド・インストール・テスト出力のみ削除し、既存コンテナとViewer・bag再生のPIDが開始前と一致することを確認。

実施環境は既存`gng_cpu`コンテナ。作業用ディレクトリは`mktemp -d /tmp/gng-observation-api-XXXXXX`で作成した`/tmp/gng-observation-api-cPvRHN`。
以下は`docker compose exec -T gng_cpu bash -lc '…'`内での実施コマンド。通常のインストール先への書込みなし。

```bash
source /ros2_ws/install/setup.bash
timeout -s INT -k 10 90 cmake -S /ros2_ws/src/ais_gng_cpu/src/gng_cpu \
  -B /tmp/gng-observation-api-cPvRHN/core -DGNG_BUILD_BENCHMARKS=ON \
  -DGNG_ENABLE_AUTHENTICATION=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/tmp/gng-observation-api-cPvRHN/install
timeout -s INT -k 10 240 cmake --build /tmp/gng-observation-api-cPvRHN/core -j 4
timeout -s INT -k 10 60 cmake --install /tmp/gng-observation-api-cPvRHN/core
export LD_LIBRARY_PATH=/tmp/gng-observation-api-cPvRHN/core:$LD_LIBRARY_PATH
timeout -s INT -k 10 120 ctest --test-dir /tmp/gng-observation-api-cPvRHN/core --output-on-failure --timeout 30

timeout -s INT -k 10 90 cmake -S /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree \
  -B /tmp/gng-observation-api-cPvRHN/experimental -DCMAKE_BUILD_TYPE=Release
timeout -s INT -k 10 240 cmake --build /tmp/gng-observation-api-cPvRHN/experimental -j 2 \
  --target gng_observation_api_test_grid gng_priority_input_api_test_grid \
  gng_observation_api_test_spatial gng_priority_input_api_test_spatial \
  gng_observation_api_test_bsp3d gng_priority_input_api_test_bsp3d
export LD_LIBRARY_PATH=/tmp/gng-observation-api-cPvRHN/experimental:$LD_LIBRARY_PATH
timeout -s INT -k 10 120 ctest --test-dir /tmp/gng-observation-api-cPvRHN/experimental \
  --output-on-failure --timeout 30 -R 'gng_(observation|priority_input)_api_test'

timeout -s INT -k 10 120 cmake -S /ros2_ws/src/ais_gng_cpu/src/ais_gng \
  -B /tmp/gng-observation-api-cPvRHN/ros -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -Dgng_cpu_DIR=/tmp/gng-observation-api-cPvRHN/install/share/gng_cpu/cmake
timeout -s INT -k 10 300 cmake --build /tmp/gng-observation-api-cPvRHN/ros -j 3 \
  --target ais_gng_cpu test_observation_pixels
export LD_LIBRARY_PATH=/tmp/gng-observation-api-cPvRHN/ros:/tmp/gng-observation-api-cPvRHN/install/lib:$LD_LIBRARY_PATH
timeout -s INT -k 10 60 ctest --test-dir /tmp/gng-observation-api-cPvRHN/ros \
  --output-on-failure --timeout 30 -R '^test_observation_pixels$'
timeout -s INT -k 10 300 cmake --build /tmp/gng-observation-api-cPvRHN/ros -j 3 --target ais_gng_component_cpu
```

**制約**

- 新ヘッダーは既存の`install(DIRECTORY include/ ...)`対象。通常環境へ反映するビルドでは、利用側より先に`gng_cpu`のビルド・インストールが必要。
- ROSノード・再生・デーモンの新規起動なし。実点群・Viewerの動作確認とGPUビルドは未実施。
