# 2026-09-23 - CPUの人・車推論結果の反映とラベル確定

## 1. 要約

人・車分類器からCPU GNGへのクラスタ年齢の受け渡しと、ラベル確定・保持・失効条件の修正。

- 分類器の返却値を生成フレーム番号から`map.frame_number - cluster.frame`へ修正。公開APIの既存契約に整合。
- 推論の保持期間内に確認回数を満たした場合のラベル確定。人・車共通の条件評価への集約。
- 期限切れ時の推論履歴の失効、人・車切替時の確認回数リセット、同一フレーム内の重複加算防止。

- CPU APIテスト：人・車それぞれの連続推論、重複入力、保持期間、失効、再確認、クラス切替、保持設定0の検証。
- 分類器テスト：生成フレーム3997・3998、現在4000で返却年齢3・2の確認。
- 有限ROSテスト`check_cluster_labels_ros.py`：点群とTFを入力元から別ドメインへ転送し、非空マップと指定クラスの確定ラベルを確認。入力元への点群・TF発行なし。

- 生成直後の人・車候補が、生成フレーム番号と経過フレーム数の誤比較によりフィードバックから除外される不具合。
- 推論継続中に確定せず、推論が古くなってから確定する保持条件の逆転。
- 期限切れの推論回数や別クラスの確認回数を使う誤確定。

## 2. 条件・検証

- 同じクラスの推論が既存の`confirmation_age`回数を満たすと、次のGNG更新で`label`へ反映。
- UNKNOWN判定・推論対象外の短い途絶は、最後の人・車推論から既存の`hysteresis_age`フレーム分を保持。期間内の再検出では確認回数を継続、期限切れ後は最初から確認。
- 内部保持期間の既存の`+1`は維持。推論の反映が次のGNG更新であるため、設定0でもその更新には有効。
- クラス切替後は新しいクラスの確認回数を再計数。モデル・対象選別・YAMLしきい値の変更なし。
- 通常インストール先の更新済みバイナリによる動作確認済み。起動済みの旧プロセスには自動反映されないため、旧版を使用中の場合は利用者によるGNGの再起動が必要。

トピック・ROSパラメータ名・メッセージ定義・公開APIの変更なし。`clusters[].frame`は生成フレーム番号、`label_inferred`は当該フレームの推論、`label`は確認・保持後の分類。ノードの幾何ラベルとは別。

- 修正前：人・車のAPIテストが「連続推論後のラベル未確定」で失敗。分類器テストも返却年齢3・2に対して3997・3998となり失敗。
- 修正後：CPU CTest 13件、学習イベント・マップ差分API 2件、実モデルを用いた分類器GTest 3件、launchテスト15件が成功。通常のログON設定でもCPU CTest 13件が成功。
- 実点群の初回確認：Macnicaの再生中区間を50秒購読し、非空マップ98件・UNKNOWN推論94件・人車推論0件。指定クラスの確定ラベルの検証は不成立。旧ROSコンポーネントの混在とmap TFの欠落警告があり、最終検証・精度評価から除外。
- 最終検証：通常インストール済みバイナリ、実モデル、既存`at128.yaml`で廊下bagを隔離再生。非空マップ120フレーム、人の推論219件・確定ラベル174件を確認。件数はフレームをまたぐ延べクラスタ数であり、人数ではない。車は推論5件・確定0件、確定処理自体はCPU APIテストで検証。
- 通常インストール先での所属情報のROS回帰検証も成功。5フレーム、最大所属数120、UNKNOWNを含む推論結果10件。
- 最終ROSテスト中の`/proc/<pid>/maps`で`/ros2_ws/build/ais_gng/libais_gng_component_cpu.so`と`/ros2_ws/install/gng_cpu/lib/libgng_cpu.so`の使用を確認。通常ビルド・インストール済み・一時検証用のCPUライブラリのSHA-256一致（`cdb5d7736c23ed07fdc86ca7202abf5c518ca0222067f0550cb0be8f1e8417a4`）。
- 別ターミナルの通常ビルドによる反映を確認したため、重複する一時ROSコンポーネントビルドは中断。通常ビルド先への別途上書きなし。
- 利用者側で再起動された通常ドメインのGNG PID 32948でも、更新済みCPU・ROSライブラリとマップ済みファイルのinode一致を確認。
- 検証用GNG・bag・転送・購読・ビルドプロセスの全終了と、検証用の追加ROSデーモンの残存なしを確認。既存プロセスへの停止・再起動操作なし。一時ビルド領域`/tmp/gng-classification-fix-vkWnEY`（16MB）は削除済み、記録コマンドから再生成可能。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cmake -S /ros2_ws/src/ais_gng_cpu/src/gng_cpu \
  -B /tmp/gng-classification-fix-vkWnEY/core \
  -DCMAKE_BUILD_TYPE=Release -DGNG_BUILD_BENCHMARKS=ON -DGNG_ENABLE_FRAME_LOG=OFF
cmake --build /tmp/gng-classification-fix-vkWnEY/core -j2
LD_LIBRARY_PATH=/tmp/gng-classification-fix-vkWnEY/core:$LD_LIBRARY_PATH \
ctest --test-dir /tmp/gng-classification-fix-vkWnEY/core --output-on-failure --timeout 30
cmake -S /ros2_ws/src/ais_gng_cpu/src/ais_gng \
  -B /tmp/gng-classification-fix-vkWnEY/ros -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build /tmp/gng-classification-fix-vkWnEY/ros --target test_cluster_classification -j1
ROS_DOMAIN_ID=229 OPENBLAS_NUM_THREADS=1 timeout --signal=INT --kill-after=5 30 \
  /tmp/gng-classification-fix-vkWnEY/ros/test_cluster_classification --gtest_color=no
```

最終実データ検証の起動コマンド。実際の実行時はPythonの`try/finally`でbagの子プロセスを管理し、成功・失敗時とも停止。スクリプト内のGNG起動引数は標準出力へ記録、点群・TF転送先はドメイン229のみ。

```bash
ROS_DOMAIN_ID=230 ros2 bag play \
  /rosbag/fuzzy/AT128/lidar_camera_rosbag2_hallway/home/fuzzrobo/デスクトップ/rosbag2_2026_05_22-09_32_47/ \
  --topics /lidar_points /tf_static --loop
ROS_DOMAIN_ID=229 OPENBLAS_NUM_THREADS=1 \
  python3 -B /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_cluster_labels_ros.py --source-domain 230
ROS_DOMAIN_ID=231 OPENBLAS_NUM_THREADS=1 timeout --signal=INT --kill-after=10 35 \
  python3 -B /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_cluster_members_ros.py
```

**制約**

- ラベル伝達・確定処理の検証と、モデルの実環境での認識精度は別。推論スコアは正解率ではなく、人・車の検出率改善の保証なし。
- 通常CPU版の状態機械の変更。ROS分類器の年齢返却はCPU/GPU共通。独立した実験版の状態機械、GPUコアバイナリの変更・実行検証なし。
