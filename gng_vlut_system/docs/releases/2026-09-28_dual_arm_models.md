# 2026-09-28 - 双腕URDFの移動とmax / longのGNG・VLUT設定

## 1. 要約

`urdf/`配下の3機種に対応。既存の`ToPoDualArm.yaml`は移動先へ修正し、新2機種に個別YAMLを追加した。

| YAML（`gng_vlut_system/config/`） | URDF（`urdf/`） | 学習対象・保存先ID |
| --- | --- | --- |
| `ToPoDualArm.yaml` | `dual_arm_urdf/dual_arm_robot.urdf` | 既存の左腕7関節・`ToPoDualArm10000` |
| `topo_dual_arm_max.yaml` | `topo_dual_arm_max/topo_dual_arm_max.urdf` | 左右14関節・`topo_dual_arm_max` |
| `topo_dual_arm_max_long.yaml` | `topo_dual_arm_max_long/topo_dual_arm_max.urdf` | 左右14関節・`topo_dual_arm_max_long` |

- 新2機種は肩からTCPまでの左右各7関節を選択。腰・首・グリッパーは学習自由度に含めない。
- 学習結果は`gng_results/<保存先ID>/`へ分離。既存機種のGNG・VLUTの流用なし。
- 新2機種のTCP範囲はURDFルート基準で各軸±1.5 m。現行設定は上限10,000ノード、240万学習、VLUT 0.02 m。学習品質の最適化は未検証。
- グリッパーが旧モデルの直動式から回転式へ変わっているため、旧把持体積定義を流用せず、その表示は無効。
- 移動前を参照していた把持形状設定、補正ノード、ダミー関節launch、関連スクリプト・HTML・実行ガイドも修正。
- 新モデルの環境入力は外部TFを使用。TF未接続の入力をworld座標として扱う代替処理は無効。

Docker内での学習例：

```bash
ros2 launch gng_vlut_system offline_urdf_trainer_dual.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml
```

ロボット表示と、学習後の環境VLUT接続（それぞれ別ターミナル）：

```bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml
ros2 launch gng_vlut_system environment_to_vlut.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml
```

`gng_viewer_bridge.launch.py`は学習データがなくてもURDFを表示。GNG・VLUTの不足ファイルをログへ表示し、GNG配信ノードだけ省略。学習後の再起動でGNG配信も開始。

longはYAML名を`topo_dual_arm_max_long.yaml`へ変更。片腕学習時は学習コマンドに`gng_profile_names:=left_arm`または`right_arm`と別の`experiment_id:=...`を指定し、表示側にも同じIDを指定する。

## 2. 条件・検証

- 全3機種のYAML、参照メッシュ、関節鎖、リンク名、保存先の分離を確認。
- Releaseの`grasp_candidate_refiner_node`再ビルド成功。`cmake --install /ros2_ws/build/gng_vlut_system`で新YAMLとビルド結果を反映。
- 全3機種で小規模学習と`gng.bin` / `vlut.bin`保存が成功。上限8ノード、400学習、追加処理各20回。結果はmax 8、long 8、旧モデル7ノード。
- 試験の形状・キャッシュ解像度は0.02 m、衝突球化ボクセル幅は0.01 m。粗い球化でmaxに3組、longに衝突候補が出たが、両機種ともYAML既定の0.0025 mで初期姿勢を再検査し候補0組を確認。除外ペアの追加なし。
- Viewer・環境VLUT launchは新2機種の名前空間と子launch引数の展開を確認。GNGデータ有無による起動分岐も確認。
- ROS_DOMAIN_ID=96 / ROS_LOCALHOST_ONLY=1で上記maxのViewer起動コマンドを実行し、URDFと21関節の姿勢データ受信、自己認識ノード初期化を確認。試験ノード全終了。実ブラウザの描画と実センサ入力は未検証。
- 初回試験は自己認識初期化中の停止とSIGINT二重送信で終了エラー。試験側を初期化完了待ち・launchだけへのSIGINT送信へ修正し、再試験で全ノード正常終了を確認。
- 続報：maxの通常設定100万回学習とVLUT生成、1000ノードのViewer実配信が完了。[Gazeboデモと検証](2026-09-28_dual_arm_gazebo_demo.md)。longの通常設定学習と実機動作は未実施。小規模試験出力は`artifacts/dual_arm_models_20260928/`に分離。

試験は`gng_cpu_container`で以下を実行。`MODEL` / `CASE`の組は`topo_dual_arm_max/max`、`topo_dual_arm_max_long/long`、`ToPoDualArm/legacy`。全試験プロセス終了済み。

```bash
export ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1
MODEL=topo_dual_arm_max
CASE=max
ros2 run gng_vlut_system offline_urdf_trainer --ros-args \
  --params-file /ros2_ws/src/gng_vlut_system/config/$MODEL.yaml \
  -p gng.data_directory:=/ros2_ws/src/artifacts/dual_arm_models_20260928 \
  -p gng.experiment_id:=${CASE}_smoke -p gng_params.max_node_num:=8 \
  -p gng_params.max_iterations:=400 -p gng_params.lambda:=40 \
  -p gng_params.refine_iterations:=20 -p gng_params.coord_edge_iterations:=20 \
  -p gng.spatial_map_resolution:=0.02 -p gng.self_recognition_resolution:=0.02 \
  -p gng.arm_cache_resolution:=0.02 -p collision.voxel_ball.voxel_size:=0.01
```

実行時は上限180秒の`timeout --signal=INT --kill-after=10s`を付加。新2機種の既定解像度による初期検査は同じ実行ファイル・YAMLで、上書きを試験用保存先・固有ID・`initial_collision_only:=true`だけにし、上限120秒で実行。既存プロセス・コンテナの停止なし。
