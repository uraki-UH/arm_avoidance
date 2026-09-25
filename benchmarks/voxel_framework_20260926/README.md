# セル評価基盤のコンパイル時除外・実GNG比較（2026-09-26）

## 要約

共有セル評価基盤を追加し、不要な属性処理・ファジィ評価・履歴を型特殊化と`if constexpr`で除外。
既定は3機能ともOFF。既存の把持・境界とFVGは、全機能ビルドでも履歴を要求しない構成。
[仕様・利用方法](../../ais_gng_cpu/docs/sampling.md#セル評価フレームワークのコンパイル時拡張)を正本とする。

変更前・拡張OFF・全機能ONの実GNG比較、80入力×2条件×3構成×3試行＝1,440フレームで出力一致。
ノード座標・法線・rho・ラベル・エッジ順序・クラスタ・入力ラベルと、重点候補の番号・XYZ順序を照合。
追加機能条件では勝者イベント・差分・共分散／支持統計・観測方向も照合。浮動小数点はビット列比較。
試験用GNGライブラリの`.text`は3構成でSHA256一致。GNG／サンプラーのサイズも1,272／272 bytesのまま。
属性型と集計関数のないポリシーでもOFF構成はコンパイル成功。無効pipelineと直接計算の命令列も一致。
空pipelineは`sizeof=1`だが、空基底としての追加サイズ0。履歴表は履歴ONの型だけに存在。

## 条件・検証

`gng_cpu_container`、ROS Humble、GCC 11、Release。比較基準は`1f59b4526253cacc9b6fd33a685a3326ca428750`。
`/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3`の先頭80点群。
`at128.yaml`、入力上限200,000点・実入力159,744〜160,256点、voxel 0.5 m、総学習4,000回、最大20,000ノード。
通常条件の実ノード数13,541〜19,250。センサー姿勢は原点・単位回転。ROSの平面抽出・NN分類は比較対象外。
通常条件は追加重点なし。混合条件は把持0.3・境界0.2・旧添字API0.3と観測／共分散／支持／差分取得を有効化。
把持BBoxは[-20,-20,-2]〜[20,20,4] m、境界アンカーと半径は`fixture.cpp`の固定値。
変更前後の**比較用ソースコピーだけ**乱数seed=20260926、速度推定の時間刻み=0.1 secに固定。本番コードは変更なし。
CPU 4固定、各80フレームの先頭20を除外した平均、3試行。2試行目は構成の実行順を反転。

| 実GNGの平均処理時間 | 変更前 ms | 拡張OFF ms | 全機能ビルド ms |
| --- | ---: | ---: | ---: |
| 通常 | 40.588 | 39.214 | 39.305 |
| 既存重点＋統計等 | 42.085 | 41.826 | 41.585 |

時間は入力設定＋規則登録／GNG実行＋Tmap取得。bag読込・ハッシュ計算・候補展開・DDS・Viewer描画は除外。
本番の`processing`ログ全体とは異なる測定区間。既存ユーザー処理と並行しており、小差を高速化効果とは扱わない。
比較した既存ルールの結果変更や時間増加は検出なし。新しい評価式・実履歴利用の処理時間ゼロは主張しない。
プロセス最大RSSは3構成・全試行で761,200〜762,156 KiB。bag全80点群の保持を含み、微小な割当差の証明には不使用。

変更前CPU23件、変更後OFF／全ON各24件、通常配布先のROS関連4件が成功。
属性のみ・属性＋ファジィ・属性＋履歴でも新規APIテスト成功。5構成すべてで実GNGへ合成200点を4フレーム投入。
重点配分・候補集合・履歴有無、時刻逆行・epoch変更・未観測・重複キー・容量超過時の失効を確認。
不正CMake構成と、未ビルドのファジィ／履歴要求は期待どおりコンパイル拒否。
FVGは既存COLCON_IGNOREのため初回colcon対象外。設定を変えず独立CMakeでOFF／全ONのビルドを確認。
FVG実行ファイルの`NEEDED`に`libgng_cpu.so`なし。FVGの実ROS入力・ラベル改善は未検証、従来判定を保持。
既存の未使用変数・文字列const・Torch等のビルド警告は残存。

通常配布先はOFFで再ビルド済み。別途ROSドメイン179で実bag16フレームを入力し、非空Tmapと把持抽出の一致を確認。
最後4フレームは候補失効で空出力。削除済み`/downsampling/nonplane`のPublisherなし。
このROS確認は通常乱数を使用。全機能ビルドのROSノード・実Viewer描画、新規重点条件の認識品質は未検証。

コンテナ内の起動・再現コマンド（既存インストール、検証ソースコピーと開始時保存アーカイブが前提）：

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src
PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 600 python3 benchmarks/voxel_framework_20260926/verify.py build
PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 600 python3 benchmarks/voxel_framework_20260926/verify.py measure
PYTHONDONTWRITEBYTECODE=1 python3 benchmarks/voxel_framework_20260926/verify.py summarize
PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 360 python3 benchmarks/voxel_framework_20260926/verify.py matrix
ROS_DOMAIN_ID=179 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 240 python3 benchmarks/common_sampling_20260925/smoke.py --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 --frames 16 --trials 1 --validation --output-dir /ros2_ws/src/artifacts/voxel_framework_20260926/ros
```

初回は`verify.py prepare`で生成。`before/source.tar.gz`がない場合は上記基準commitの`git archive`でcore・ais_gng/includeを保存。
既存コピーを上書きしないため、prepareの再実行は拒否。生成ソースの変更はseed／dtのみで、初期保存物は保持。
保存先`artifacts/voxel_framework_20260926/`はGit対象外。`summary.json`、条件別JSON・ログ、`matrix.log`を参照。
`final_build.log`は通常配布先のビルド・回帰とFVG全ON、`fvg_build.log`はFVG OFF、`ros_validation.log`はROS確認。
`verify.py`は子プロセスの起動コマンドと正常終了を記録。全比較・ビルド・テストの実行セッション終了。
ROS試験の直接起動は`/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu --ros-args -r __ns:=/common_sampling_after_mixed_0_validation --params-file /ros2_ws/src/artifacts/voxel_framework_20260926/ros/after_mixed_0_validation.yaml`。
試験ノードPID343140は終了コード0。スクリプトのfinallyで所有グループだけを停止し、試験observerも破棄。
既存bag・Viewerと3コンテナは停止・再起動なし。新しいROSデーモン・常駐試験プロセスの残存なし。
