# 2026-08-26 - AiS-GNG source CPU backend

## Summary

AiS-GNGのCPUバックエンドを同梱済みバイナリからワークスペース内の`gng_cpu`ソースビルドへ切り替えた。Release最適化、署名・YubiKey認証、フレームログを独立して設定できるようにした。

## Changed

- `ais_gng_component_cpu`は`ais_gng/libgng/lib_x86_64/libgng_cpu.so`ではなく、`gng_cpu::gng_cpu`へリンクする。
- CPU版のAPI headerと共有ライブラリは`gng_cpu`パッケージから取得する。
- GPUバックエンドは従来どおりarchitecture別の同梱済み`libgng_gpu.so`を使用する。
- ReleaseビルドでもGNGのフレーム要約ログを出力する。

## Added

- `ais_gng`から`gng_cpu`へのpackage依存を追加した。
- `GNG_ENABLE_AUTHENTICATION` CMake optionを追加した。既定値は`OFF`。
- `GNG_ENABLE_FRAME_LOG` CMake optionを追加した。既定値は`ON`。

## Fixed

- ソースビルドしたRelease版が対応する署名ファイルを持たず、起動時に`Failed to check file`で停止する問題を解消した。
- `_RELEASE`によって認証とフレームログが同時に切り替わっていた状態を分離した。

## Removed

- CPUバックエンドの実行時における、同梱済み署名付き`libgng_cpu.so`への依存を削除した。同梱ファイル自体は削除していない。

## Behavior Impact

- `gng_cpu`ソースの変更が、`colcon build`後のAiS-GNG CPU実行へ反映される。
- 通常のソースビルドではバイナリ署名確認とYubiKey認証を行わない。
- Releaseビルドでも各入力フレームについて`I/V/A/Nodes/Clusters`、工程別時間、クラスタ数を標準出力へ出す。
- `performance.log_interval_ms`によるROS 2周期ログは独立しており、`0`の場合は従来どおり無効となる。
- 既に起動しているAiS-GNGプロセスは旧共有ライブラリを保持するため、再ビルド後に再起動が必要となる。
- CPUソース版の学習係数処理を同梱済みCPUバイナリと互換化した。各ノードは生成時の`eta_s1` / `eta_s2`を保持し、各フレームで`node.eta_decay_rate`を掛けて減衰する。
- `node.eta_decay_rate: 1.0`では学習係数を減衰させない。ソース追加時に入っていた`eta / (node_age + 1)`方式は、既存設定でもノード追従がほぼ停止するため削除した。

## Topics / Params / Messages

- topic名、ROS parameter、message定義の変更はない。
- 既存ROS parameter `node.eta_decay_rate`をCPUソース版でも受理するよう復元した。
- 追加CMake option: `GNG_ENABLE_AUTHENTICATION` (`OFF` / `ON`, default `OFF`)
- 追加CMake option: `GNG_ENABLE_FRAME_LOG` (`OFF` / `ON`, default `ON`)
- 認証を有効にする場合は、ビルド時に`-DGNG_ENABLE_AUTHENTICATION=ON`を指定し、生成された共有ライブラリに対応する署名ファイルとYubiKeyを用意する。

## Verification

- Docker内で`gng_cpu`と`ais_gng`を`CMAKE_BUILD_TYPE=Release`として再ビルドし、成功した。
- `ldd /ros2_ws/install/ais_gng/lib/libais_gng_component_cpu.so`で、`libgng_cpu.so`が`/ros2_ws/install/gng_cpu/lib/libgng_cpu.so`へ解決されることを確認した。
- 認証OFFのソース版を小規模設定で起動し、`Initialized successfully`を確認した。
- 空の`PointCloud2`を1回入力し、`I: 0, V: 0, A: 0, Nodes: 0, Clusters: 0`、工程別時間、クラスタ要約の出力を確認した。
- 同梱済みCPUバイナリのDWARF型情報と逆アセンブル結果から、ノードごとの`eta_s1` / `eta_s2`、`node.eta_decay_rate`、フレーム末尾の乗算減衰を確認し、CPUソース版へ復元した。
- `graspnet.yaml`で起動し、`param node.eta_decay_rate 1.00`として受理され、`Unknown param`にならないことを確認した。
- rosbag再生中の2秒計測で、修正前は共通5000ノードの平均移動量が約`0.011 mm`、修正後は共通2493ノードで約`1.97 mm`となり、古いノードの追従がほぼ停止する状態が解消したことを確認した。

## Risk / Notes

- フレームログは入力周期ごとに標準出力へ出るため、高Hz入力ではログI/O負荷が増える。不要な運用では`-DGNG_ENABLE_FRAME_LOG=OFF`でビルドする。
- `GNG_ENABLE_AUTHENTICATION=ON`で有効な署名ファイルがない場合は、起動時に`Failed to check file`となる。
- 実行中プロセスと同じinstall/build領域を再ビルドすると、共有ライブラリ更新と新規起動が競合する可能性がある。再ビルド中はAiS-GNGを新規起動しない。
