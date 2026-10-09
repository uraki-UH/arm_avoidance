# 2026-10-01 - 頭部RealSenseからの環境GNGのベース座標出力

原因・実測:

- 稼働設定: `input.local_coordinates: true`、`input.base_frame_id: map`。ローカル座標優先によるTF変換の省略。
- 稼働出力: `/topological_map`・`/scan/transformed`とも`camera_depth_optical_frame`。`/ToPoDualArm/Tmap_static`は`ToPoDualArm/base_link`。[8秒読取り](../../../artifacts/gng_base_frame_20261001/live.json)。
- 実機姿勢: 首pan 0.650407 rad・tilt 1.113670 rad、腰0 rad。ベースからカメラへのTF取得成功。表示だけの補正では頭部移動時の蓄積GNGの座標基準は不変。
- 設定変更後の再発原因: `GNG_VERSION=0`の`GNG::setPointCloud`で、固定座標へ変換済みの入力とは別に、センサ姿勢差分を蓄積ノード全体へ適用。前回の学習後重心の概略検証では、この入力投入時の移動を未検出。

変更:

- 初回追加: `ais_gng/config/gng_cpu/graspnet_topodualarm.yaml`。当初は旧`graspnet.yaml`を保持。基準`ToPoDualArm/base_link`、ローカル座標OFF、取得時刻TF必須化ON。
- 続報: ユーザー依頼により通常の`graspnet.yaml`にも同じ座標変換3項目を適用。手編集のノード間隔0.015 m・長期記憶無効化−1など学習設定は保持。別YAMLへの起動切替は不要。
- `input.enable_strict_transform`: 既定OFF。ONの場合は全入力の取得時刻TF成立後に学習・配信。TF不成立時の恒等変換による誤配信を抑止。TF待機上限0.05 s。既存設定・観測支持モードの既定動作は保持。
- 座標切替: 蓄積済みGNGの座標混在防止のため新ノード起動が必要。稼働ノードへのパラメータ変更・停止・再起動は未実施。
- 本体修正: 固定座標時の蓄積ノード差分変換を禁止。新規点群のTF変換と従来のローカル座標時の差分処理は保持。追加設定なし。

起動（既存の環境GNG launch終了後、コンテナ内・workspace source済み）:

```bash
ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=graspnet.yaml
```

出力座標: RealSense生点群はカメラ基準、`/scan/transformed`と`/topological_map`は`ToPoDualArm/base_link`。カメラTFは実機関節状態由来。Gazebo側の首姿勢による実センサ変換なし。

検証コマンド:

- ビルド: `ais_gng` Release成功（CPU/GPUコンポーネント）。
- 続報の設定検証: 実`graspnet.yaml`を使用したlaunch展開を含む16 / 16件成功。基準フレーム・取得時刻TF必須化・長期記憶無効化・手編集のノード間隔保持を確認。
- 続報の試験コマンド: コンテナ内でROS・workspaceをsource後、`ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 PYTHONDONTWRITEBYTECODE=1 timeout 40s python3 -B -m pytest -q -p no:cacheprovider ais_gng_cpu/src/ais_gng/test/test_clustering_yaml_launch.py`。domain229の確認ノード・試験プロセスは終了済み。
- 隔離検証: 頭部yaw 0・0.8 radの2姿勢でベース点群重心誤差0.000001 m未満、GNG出力座標と位置の整合確認。TF欠測・取得時刻不整合での出力抑止成功。所有ノード停止済み。[結果と内部起動コマンド](../../../artifacts/gng_base_frame_20261001/transform_test/report.json)。
- 本体再現試験: yaw・pitch・rollと並進を変更、学習フレーム数を固定。旧版の既存ノード最大移動1.83848 mで失敗、修正版は6ノードすべて0 mで成功。入力点群の回転・並進も独立検証。既存パラメータ更新API試験も成功。
- 本体ビルド: Docker内`gng_cpu` Release/install成功、既存コンパイラ警告あり。新しい実行プロセスで修正版ライブラリを読込み。
- 本体修正後ROS試験: 頭部2姿勢の入力点群重心誤差最大0.000000775 m、GNGベース座標出力・TF欠測・取得時刻不整合の拒否に成功。予測16 s、実測14.42 s。[結果・ノード起動引数](../../../artifacts/gng_fixed_frame_20261001/ros_trial2/001_robot_base_frame/result/report.json)。
- 初回ROS試験失敗: `libnonplane_component_extractor.so: file too short`でノード起動不可。同時刻の共有ライブラリ更新を確認、生成完了後の再試験で成功。自試験による同ライブラリの変更なし。[失敗記録](../../../artifacts/gng_fixed_frame_20261001/ros_trial/report.json)。
- 反映・制限: YAMLのみの前回変更と異なり、今回は本体ビルド済み。変更前から稼働しているGNGは再起動が必要。実機首振り・Viewerの最終見た目は未検証。所有試験プロセス・ノードは終了済み、既存GNG・実機の停止操作なし。

本体修正の検証コマンド（コンテナ内、ROS・workspaceのsource済み）:

```bash
cd /ros2_ws
timeout 180s colcon build --packages-select gng_cpu --symlink-install --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
cd /ros2_ws/src
# API回帰試験。生成済み実行ファイルの起動、終了済み
timeout 15s /tmp/gng-fixed-frame-dG70PS/gng_fixed_frame_api_test
timeout 15s /tmp/gng-fixed-frame-dG70PS/gng_parameter_update_api_test
# 隔離ROS試験。再実行時は未使用の出力先への変更が必要
python3 -B skills/run-benchmark-batch/scripts/run_batch.py artifacts/gng_fixed_frame_20261001/cases.json --output artifacts/gng_fixed_frame_20261001/ros_trial2 --repeats 1 --timeout-sec 40 --max-total-sec 45 --estimate-sec 16
```

API試験の正本: `gng_cpu/test/gng_fixed_frame_api_test.cpp`。`GNG_BUILD_BENCHMARKS=ON`時のCTestにも登録。

```bash
# 読取りプローブ。終了済み
ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 python3 /ros2_ws/src/artifacts/gng_base_frame_20261001/probe.py
# 隔離ROSでのTF欠測・頭部姿勢変更・取得時刻の検証。終了済み
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_robot_base_frame.py \
  --output /ros2_ws/src/artifacts/gng_base_frame_20261001/transform_test
```
