# ToPoDualArmの再学習・自己干渉監査

- 対象: `urdf/dual_arm_urdf/dual_arm_robot.urdf`、左腕7自由度。右腕・腰・首・グリッパーの関節値は0固定。
- 出力: `gng_vlut_system/gng_results/ToPoDualArm10000_selfchecked_20261001/`。既存`ToPoDualArm10000/`は保持。
- 探索条件: 上限11,000ノード、探索2,200,000反復、補充600,000反復。TCP範囲は元YAMLを維持。
- 自己干渉: 全身の元メッシュ表面交差＋1 mm占有内包。既存の固定剛体・直接関節対と左右の指対だけを除外。辺は最大関節刻み0.025 radの離散検査。
- VLUT解像度: 0.02 m。自己干渉検査の1 mm占有とは別用途。
- 結果・失敗・制限: [リリースノート](../../gng_vlut_system/docs/releases/2026-10-01_topodualarm_gng_retrain.md)。
- 結果資料: [artifacts](../../artifacts/topodualarm_retrain_20261001/)。開始時YAMLは`input_params.yaml`、旧入力は`input_sha256.txt`、最終モデルは`final_output_sha256.txt`。修正前の新GNGは`gng_before_edge_repair.bin`として保持。
- 監査条件: 別プロセスで保存値を再読込し、現行の同じ幾何判定器で全姿勢・各層の全辺を照合。異なる幾何アルゴリズムによる二重証明ではない。
- 制限: 内包・BOX近似と離散辺検査。非学習関節の別姿勢、連続時間の非干渉、実機動作は保証対象外。

## 実行

コンテナ内、`source /ros2_ws/install/setup.bash`後のコマンド。初期検査・学習のmanifestで
`ROS_DOMAIN_ID=218`、`ROS_LOCALHOST_ONLY=1`、専用ROSログを指定。実機指令ノードの起動なし。
runnerの出力先は未使用のディレクトリが必須。開始時の既存プロセスは`processes_before.txt`へ記録。

```bash
cmake --build /ros2_ws/build/gng_vlut_system --target offline_urdf_trainer visualization_gng_trainer test_gng_collision_filter test_gng_nearest_training test_gng_nearest_index test_self_collision_policy test_geometric_solid_containment -j2
ctest --test-dir /ros2_ws/build/gng_vlut_system --output-on-failure -R '^test_(gng_collision_filter|gng_nearest_training|gng_nearest_index|self_collision_policy|geometric_solid_containment)$'
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider /ros2_ws/src/gng_vlut_system/test/test_dual_arm_collision_geometry.py

python3 /ros2_ws/src/skills/run-benchmark-batch/scripts/run_batch.py /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/initial.json --output /ros2_ws/src/artifacts/topodualarm_retrain_20261001/initial_fixed_run --repeats 1 --timeout-sec 240 --max-total-sec 260
python3 /ros2_ws/src/skills/run-benchmark-batch/scripts/run_batch.py /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/train.json --output /ros2_ws/src/artifacts/topodualarm_retrain_20261001/train_run --repeats 1 --timeout-sec 7200 --max-total-sec 7230

python3 /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/build_audit.py
timeout -s INT -k 10s 600s /ros2_ws/src/artifacts/topodualarm_retrain_20261001/audit /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/audit_old.json
timeout -s INT -k 10s 5400s /ros2_ws/src/artifacts/topodualarm_retrain_20261001/audit /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/audit_final.json
```

実測学習時間: 7,670.33秒。今回のrunner上限7,200秒は実行途中に延長。
所有runner PID 127428だけを一時停止し、所有学習PGID 127443の監視を
`python3 /ros2_ws/src/artifacts/topodualarm_retrain_20261001/extend_training.py`へ引継ぎ。
追加上限7,200秒、完了時にrunnerを再開して回収。記録は`extension.json`と`train_run/report.json`。
学習プロセスの再起動・設定変更なし。次回の同規模実行は、新しい出力先と
`--timeout-sec 14400 --max-total-sec 14430`を指定。既存モデルへの無断上書きは禁止。

学習本体の起動コマンド:

```bash
ros2 run gng_vlut_system offline_urdf_trainer --ros-args \
  --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  --params-file /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/training.yaml
```

## 保存後の診断・修復

`audit_new.json`・`audit_diagnostic.json`は修復前の監査設定、`audit_limits.json`は幾何判定なしの
関節限界切分け専用。後者を自己干渉の合格証拠に使わない。
単独辺のFK比較は`audit_edge_state.json`・`audit_edge_pure.json`。
各設定を上記`audit`へ渡す方法で実行。既存の監査JSONへの上書き拒否。
修復前の再現時は、設定の`gng_path`を退避bin、`output`を新しい保存先へ変更。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/filter_edge.py \
  --input /ros2_ws/src/artifacts/topodualarm_retrain_20261001/gng_before_edge_repair.bin \
  --output /tmp/topodualarm_edge_fixture.bin --first-id 3340 --second-id 3853 --mode fixture
python3 /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/build_audit.py \
  /ros2_ws/src/artifacts/topodualarm_retrain_20261001/probe_filter \
  /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/probe_filter.cpp
timeout -s INT -k 10s 60s /ros2_ws/src/artifacts/topodualarm_retrain_20261001/probe_filter \
  /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/audit_new.json \
  /tmp/topodualarm_edge_fixture.bin /tmp/topodualarm_filter_probe.json
```

`filter_edge.py --mode remove`は指定辺と欠落端点参照だけの除外。全ノードレコードをバイト単位で保持。
今回の指定は`3340 / 3853`、記録は`edge_repair.json`。旧新binの入替前に元ファイルを退避済み。
通常学習の再発対策は`GrowingNeuralGas.cpp`の双方向辺削除と欠落端点の読込ガード。
修正前2件失敗は`regression_before.log`、修正後は`regression_after.log`。

## 集約表示・ROS照合

```bash
export ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/topodualarm_retrain_20261001_logs
timeout -s INT -k 10s 900s ros2 run gng_vlut_system visualization_gng_trainer \
  --input /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000_selfchecked_20261001/gng.bin \
  --target-nodes 150 --iterations 200000 --seed 42 --joint-motion-weight 0 \
  --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml
PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 15s 180s python3 /ros2_ws/src/benchmarks/topodualarm_retrain_20261001/verify_ros.py
```

配信試験の全launch引数は`viewer_command.json`。
実機入力・環境ボクセル・取付TF・自己可視化を無効化し、`finally`で自身のlaunchグループだけを停止。
終了時のSIGINTによる3子プロセスの`exit -2`を記録。配信照合は成功、残留プロセスなし。
`ros_cleanup.json`で専用domainの空状態、`processes_before_viewer.txt`・`processes_after.txt`で既存PID維持を確認。
学習・監査・集約・配信・終了確認プロセスは全終了。専用一時ROSログは資料内の`ros_logs/`へ移動済み。

## 新モデルの利用

通常環境での起動。`ToPoDualArm.yaml`の既定IDと既存プロセスは未変更。

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  id:=ToPoDualArm10000_selfchecked_20261001
```
