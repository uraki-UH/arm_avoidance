# 共通サンプリングの検証（2026-09-25）

## 要約

既存入力セルを共有する規則APIへ把持・境界重点を接続し、通常・unknown・重点の学習配分を共通化。
ラベル名による抽選器の分岐は不要。点数・最近傍属性・要求時だけの同一セル内ノード数を評価器へ提供。
非平面・小規模平面・密度差の本番評価式は未追加。削除した非平面重点・専用トピックの復活なし。
実装の契約・拡張手順は[仕様](../../ais_gng_cpu/docs/sampling.md)を正本とする。

## 条件・検証

環境は`gng_cpu_container`のROS Humble、Releaseビルド。通常配布先の`gng_cpu`・`ais_gng`ビルド成功。
既存の符号比較・`_GNU_SOURCE`再定義等の警告は残存。CPU CTest 23対象、ROS単体の関連4対象が成功。
配分維持、重み分布、属性参照、任意ノード数集計、不正評価の退避、入力・規則失効、旧API混合を検証。
把持セル境界と境界距離の枝刈りは、従来の元点判定とも比較。

実bagは`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の先頭60点群。
`at128.yaml`と平面設定を使用し、入力10万点・uniform・voxel 0.5 m・総学習4,000回・最大2万ノード。
分類器・曲面OFF、入力座標のまま。把持配分0.3・境界配分0.2の混合と、両方OFFを比較。
把持範囲は[-20,-20,-2]〜[20,20,4] m、margin 0。未知領域学習は両条件で従来どおり。
旧ライブラリは作業開始時に保存した2ファイルを子プロセスの`LD_LIBRARY_PATH`だけで指定。
各60フレームの先頭20を除外し、残り40のprocessing平均を2試行平均。2試行目は実行順を反転。

| 条件 | 変更前 ms | 共通化後 ms |
| --- | ---: | ---: |
| 把持・境界OFF | 52.397 | 52.450 |
| 把持・境界ON | 68.783 | 68.144 |

測定値は入力処理・GNG・平面抽出・出力処理を含むノード内processingで、DDS待機・Viewer描画は対象外。
既存ユーザー処理と並行、乱数seed固定なし。小差の有意性や、大幅な高速化・同一グラフは主張しない。
480フレームすべてで非空Tmap。把持出力240フレームは独立したBBox抽出とXYZ・順序が完全一致。
最終ビルドの追加16フレームでも一致し、最後4フレームは古い候補の失効による空出力を確認。
全試行で`/downsampling/nonplane`のPublisherなし。今回のROS検証に非自明TF・実Viewer目視は含まない。

固定実入力10万点・13,654セルで、同じ単純なセル条件を重ねた準備時間も測定。
CPU 4へ固定、各12回の先頭3回を除外した中央値を3試行し、その中央値を採用。

| 規則数 | 準備 ms | 候補エントリー | 元点評価回数 |
| ---: | ---: | ---: | ---: |
| 1 | 0.339 | 8,278 | 0 |
| 4 | 0.781 | 33,112 | 0 |
| 16 | 2.700 | 132,448 | 0 |

対象はセル評価・候補生成・正規化・累積分布構築だけ。voxel構築・照合保存・抽選・学習・ROS配信は除外。
評価式は`num_points >= 3`、総配分0.5。任意ノード数集計・利用側属性表作成もこの測定に含まない。
無指定時は追加セル走査0。条件数・重なりが増えれば候補数と時間も増える。
実物体の追従性・認識精度や、新しい密度評価式による改善は未検証。

コンテナ内の再現コマンド（既存のビルド先とローカル計測データが前提）：

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src
timeout -s INT -k 20 480 colcon build --base-paths ais_gng_cpu/src --build-base /ros2_ws/build --install-base /ros2_ws/install --packages-select gng_cpu ais_gng --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cmake --build artifacts/nonplane_attention_20260925/gng_build -j 4
ctest --test-dir artifacts/nonplane_attention_20260925/gng_build --output-on-failure
ctest --test-dir /ros2_ws/build/ais_gng --output-on-failure -R 'test_(grasp_attention|boundary_attention|spatial_sampling|nonplane_component_extractor)$'
ROS_DOMAIN_ID=179 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 480 python3 benchmarks/common_sampling_20260925/smoke.py --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 --frames 60 --trials 2
ROS_DOMAIN_ID=179 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 240 python3 benchmarks/common_sampling_20260925/smoke.py --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 --frames 16 --trials 1 --validation
g++ -std=c++20 -O3 -DNDEBUG -DGNG_VERSION=0 -Iais_gng_cpu/src/gng_cpu/src -Iais_gng_cpu/src/gng_cpu/include benchmarks/common_sampling_20260925/scaling.cpp ais_gng_cpu/src/gng_cpu/src/cpu/voxel_grid.cpp ais_gng_cpu/src/gng_cpu/src/utils/node.cpp ais_gng_cpu/src/gng_cpu/src/utils/param.cpp ais_gng_cpu/src/gng_cpu/src/utils/vec3f.cpp ais_gng_cpu/src/gng_cpu/src/utils/utils.cpp -o artifacts/common_sampling_20260925/scaling
timeout 30 taskset -c 4 artifacts/common_sampling_20260925/scaling artifacts/nonplane_attention_20260925/selection.bin
```

CPU試験先は`GNG_BUILD_BENCHMARKS=ON`の既存分離ビルド。新規配置では同条件のCMake構成が必要。
比較用ライブラリ・固定入力・生成YAML・生ログはGit対象外。共通化後だけの機能確認は`--validation`を使用。
保存先は`artifacts/common_sampling_20260925/`。最終値は`comparison.json`、`validation.json`、`scaling_final.log`。
ビルドと単体試験は`colcon_build_final.log`、`core_test_final.log`、`ros_unit_test_final.log`。
各ノードの起動コマンド・PID・終了値は`comparison_final.log`と`validation_final.log`の`start:`／`stopped:`行。
ROSドメイン179で分離し、スクリプトの`finally`で自分の子プロセスグループだけを停止。全試験セッション終了。
最終比較8ノードと最終検証PID 330813は終了コード0。既存bag 293745、Viewer launch 305269、GNG launch 318858／node 318859を維持。
既存3コンテナのID・稼働状態を照合し、新規デーモン・試験プロセスの残存なし。稼働中GNGへの自動再起動なし。

2026-09-26追記: ヘッダー統合後も2パッケージのビルド、CPU 23対象・ROS関連4対象、上記`--validation`の16フレームが成功。
API公開シンボルを維持。処理内容は配置変更のみで、性能の変更前後比較は再実施していない。
追加ログは同保存先の`consolidation/`内の`build.log`、`core_test.log`、`ros_unit_test.log`、`validation.log`。
削除前ヘッダーは`consolidation/headers_before.tar.gz`、配布先の壊れたリンク2件は同フォルダへ退避。
試験ノードPID 336598は終了コード0。開始時のbag 293745、Viewer launch 333336と既存3コンテナを維持。
今回開始時はGNG未起動。全試験セッション終了、新規デーモン・試験プロセスの残存なし。
