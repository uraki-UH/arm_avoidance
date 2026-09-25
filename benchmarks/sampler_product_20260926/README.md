# 2026-09-26 サンプラーの製品配布構成検証

## 要約

外部評価器を禁止したSDKでも、組込みの把持・境界とCPU GNGの出力を維持。
CPU CTestは開発版25件・製品版22件、ROS関連は開発版4件・製品版2件が成功。
製品では開発専用API・ヘッダー用の3テストとROS接続例2テストを対象外。
SDK監査で外部API3関数・規則型・拡張ヘッダー・公開テンプレートターゲットの不在を確認。
正規クライアントのコンパイル・実行が成功し、規則型・関数宣言の利用はコンパイル失敗。
宣言自作とマクロ再定義でもリンク失敗。開発用ヘッダーの残存先へのinstallを書込み前に拒否。

実bagは80フレーム×2条件×3構成×2試行＝960フレームで時間以外の出力が一致。
ノード・エッジ・クラスタ・候補添字／XYZ・ラベル・勝者イベント・差分・統計を比較。
登録処理を含む入力＋学習＋公開出力取得の平均時間は次表。最初の20フレームを時間集計から除外。

| 条件 | 変更前 ms | 開発版 ms | 製品版 ms |
| --- | ---: | ---: | ---: |
| 通常 | 39.35 | 39.23 | 38.85 |
| 把持0.3＋境界0.2、支持・共分散・観測有効 | 42.17 | 41.82 | 41.79 |

2試行・既存bag/Viewer稼働下の測定であり、性能改善の一般的保証ではない。
ROS実行は各構成16フレームで候補XYZ・順序が独立抽出と一致し、最後の4入力で期限失効。
初回の製品試験は起動用実行ファイルだけのビルドで、既存開発コンポーネントを誤読込。
読込先検査の2回の失敗で検出し、コンポーネント本体を追加ビルドしたうえで製品試験をやり直し。
`/proc/<pid>/maps`の検査を追加し、製品ライブラリと製品コンポーネント両方の読込を試験条件化。
全試験プロセス終了。既存bag PID 293745、Viewer launch PID 333336と3コンテナを維持。

## 条件・検証

`gng_cpu_container`内、ROS Humble、Release＋LTO。`at128.yaml`、voxel幅0.5 m、学習4,000回。
入力は`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の先頭80点群。
比較コピーだけ乱数seed=20260926・dt=0.1秒・フレームログOFF。通常installの乱数・時刻は変更なし。
開発版は`allow_external_sampler=ON`・`enable_voxel_*`全OFF、製品版は外部登録OFF・3機能全ON。
現行評価器は履歴とファジィを要求しないため、全ONでも新しい評価式を追加した試験ではない。
旧任意重みAPIは製品にないため全構成で不使用。GPU・別OS・改ざん耐性・新しい認識品質は未検証。

以下はコンテナ内`/ros2_ws/src`でROSと依存先をsource後に実行したコマンド。
生成先は`artifacts/sampler_product_20260926/`、開始時ソースは同ディレクトリの`before/source.tar.gz`。
バックアップ・bag・生成物はローカル専用。再測定には変更前ソースの別途確保が必要。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cmake -S ais_gng_cpu/src/gng_cpu -B artifacts/sampler_product_20260926/product \
  -DCMAKE_BUILD_TYPE=Release -DGNG_BUILD_BENCHMARKS=ON -Dallow_external_sampler=OFF \
  -Denable_voxel_framework=ON -Denable_voxel_fuzzy=ON -Denable_voxel_history=ON \
  -DCMAKE_INSTALL_PREFIX=/ros2_ws/src/artifacts/sampler_product_20260926/product_install
cmake --build artifacts/sampler_product_20260926/product -j4
LD_LIBRARY_PATH=/ros2_ws/src/artifacts/sampler_product_20260926/product:$LD_LIBRARY_PATH \
  ctest --test-dir artifacts/sampler_product_20260926/product --output-on-failure
cmake --install artifacts/sampler_product_20260926/product
python3 benchmarks/sampler_product_20260926/verify.py audit
python3 benchmarks/sampler_product_20260926/verify.py prepare
python3 benchmarks/sampler_product_20260926/verify.py build
timeout -s INT -k 20 600 python3 benchmarks/sampler_product_20260926/verify.py measure
python3 benchmarks/sampler_product_20260926/verify.py summarize
cmake -S ais_gng_cpu/src/ais_gng -B artifacts/sampler_product_20260926/product_ros \
  -DCMAKE_BUILD_TYPE=Release \
  -Dgng_cpu_DIR=/ros2_ws/src/artifacts/sampler_product_20260926/product_install/share/gng_cpu/cmake
cmake --build artifacts/sampler_product_20260926/product_ros \
  --target ais_gng_cpu ais_gng_component_cpu test_grasp_attention test_nonplane_component_extractor -j4
ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 \
LD_LIBRARY_PATH=/ros2_ws/src/artifacts/sampler_product_20260926/product_install/lib:/ros2_ws/src/artifacts/sampler_product_20260926/product_ros:$LD_LIBRARY_PATH \
timeout -s INT -k 20 180 python3 benchmarks/common_sampling_20260925/smoke.py \
  --validation --frames 16 --trials 1 \
  --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 \
  --executable /ros2_ws/src/artifacts/sampler_product_20260926/product_ros/ais_gng_cpu \
  --expected-library /ros2_ws/src/artifacts/sampler_product_20260926/product_install/lib/libgng_cpu.so \
  --expected-library /ros2_ws/src/artifacts/sampler_product_20260926/product_ros/libais_gng_component_cpu.so \
  --output-dir artifacts/sampler_product_20260926/ros_product_final
```

開発ROS試験は上の`smoke.py`から製品用LD指定・executable・expected-libraryを除き、別output-dirで実行。
試験ノードは`smoke.py`の`finally`でプロセスグループ単位に停止、ROS daemon起動なし。
`prepare`は既存ソースコピーを上書きしない。各子コマンドにも時間上限あり。
詳細ログ・試行別値は生成先の`summary.json`、`*_test.log`、`sdk_*.log`、`ros_*`を参照。
[現行仕様・製品ビルド](../../ais_gng_cpu/docs/sampling.md#社内開発版と製品配布版)。
