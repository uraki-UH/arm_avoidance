# 撤去前のPコア配置実験・起動コマンド記録

状態: 配置機能・設定・専用テストは撤去済み。以下は撤去前に実行したコマンドの記録で、現行ソースへの適用不可。
再現対象: [撤去前の対象ソース](../../artifacts/gng_affinity_20260930/source/)。部分スナップショットのため、当時の依存コード・環境を備えた別の作業領域が必要。
当時の仕様: [保存済みREADME](../../artifacts/gng_affinity_20260930/source/ais_gng_cpu/README.md#gng実行区間のpコア配置)。測定条件・制約: [結果](README.md)。
当時の環境: ホスト`/home/uraki/uraki_ws`、Docker内`/ros2_ws/src`。既存bag・TF・Viewerを維持した試験。
以下のパスは当時の記録。再現時は隔離した作業領域・出力先への読み替えが必要。Docker `gng_cpu_container` 内の共通環境読込:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
```

撤去前の実行済みビルド（初回・起動時パラメータ転送修正後）:

```bash
timeout -s INT -k 20 600 colcon build --packages-select ais_gng --executor sequential --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
CMAKE_BUILD_PARALLEL_LEVEL=2 timeout -s INT -k 20 600 colcon build --packages-select ais_gng --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cd /ros2_ws/build/ais_gng
timeout 30 ctest -R '^test_gng_cpu_affinity$' --output-on-failure
```

計測準備: [前回の手順](../gng_jitter_20260930/REPRODUCE.md)で独立計測ライブラリを`/tmp/gng_jitter_20260930/profile_build`へ復元・ビルド。
今回のscripts・cases・解析・原ログは`artifacts/gng_affinity_20260930/live/`に保存。空の一時領域への復元例:

```bash
mkdir -p /tmp/gng_affinity_20260930
cp -a /ros2_ws/src/artifacts/gng_affinity_20260930/live/. /tmp/gng_affinity_20260930/
```

実行済み比較バッチの起動:

```bash
ROS_DOMAIN_ID=0 timeout -s INT -k 25 580 python3 /ros2_ws/src/skills/run-benchmark-batch/scripts/run_batch.py /tmp/gng_affinity_20260930/cases.json --output /tmp/gng_affinity_20260930/batch --repeats 2 --timeout-sec 145 --max-total-sec 560 --estimate-sec 123
```

再実行時の出力先は未使用の`batch_rerun`などへ変更。既存結果の上書き不可。
各試行の完全な引数・環境は保存済み`batch/00*_*/command.json`、runner/probe引数は`batch/report.json`に収録。
全4試行共通のROS起動:

```bash
GNG_LIB_PROFILE_CSV=CASE_DIR/stages.csv LD_LIBRARY_PATH=/tmp/gng_jitter_20260930/profile_build:$LD_LIBRARY_PATH ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml input_topic:=/lidar_points
```

当時の無効条件だけ末尾に`enable_gng_cpu_affinity:=false`を追加。有効条件は試作YAMLのtrueを使用。現行に当該設定・起動引数なし。
probeは観測プロセスだけCPU23指定。GNGの外部固定操作なし。各120秒後に所有launchグループへSIGINT、必要時TERM/KILL、終了待ち。

実行済み通常ライブラリ起動確認:

```bash
ROS_DOMAIN_ID=0 timeout -s INT -k 20 65 python3 /tmp/gng_affinity_20260930/smoke.py
```

内部コマンドは診断用環境変数なしの`ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml input_topic:=/lidar_points`。
当時の確認は30秒後に所有グループを停止。Pコア配置・解除の機能は現行から撤去済み。

保存データの解析（ホストにも同じ`/tmp`構成で復元可能）:

```bash
python3 /tmp/gng_affinity_20260930/analyze.py /tmp/gng_affinity_20260930/batch
python3 /tmp/gng_affinity_20260930/analyze.py /tmp/gng_affinity_20260930/batch --exclude-shutdown-logged --output-dir /tmp/gng_affinity_20260930/before_shutdown
```

解析は保存時にgzip化したOSログを直接読取。結果JSON/CSVの更新先は復元先のみ。
コードsnapshot・今回対象のtracked差分・生データchecksumは`artifacts/gng_affinity_20260930/`。外部bag・Docker image・build/installバイナリは非同梱。
