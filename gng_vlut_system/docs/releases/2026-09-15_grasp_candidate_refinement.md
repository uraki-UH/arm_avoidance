# 2026-09-15 - 把持幅・TCP姿勢・関節姿勢の独立した追加評価

## 1. 要約

既存の概略把持候補を入力に、周辺の実点群から接触候補と開口幅を調べ、近傍のアームGNG姿勢を初期値にIK補正する `grasp_candidate_refiner_node` を追加した。既存候補と新しい結果を別トピックで比較できる。

- `gng_control_msgs` と `gng_vlut_system` のビルド登録のみ追加。既存候補生成・選定・経路計画・ToPoDualArm設定への変更なし。

- 独立launch、設定、ノード、幾何処理ヘッダー、専用メッセージ2種、C++テストと有限ROS検証スクリプト。
- 観測幅、両側接触を確認した把持幅、進入時開口幅、補正目標姿勢、IK関節姿勢、そのFK姿勢、棄却理由の出力。
- 接触点を結ぶ線と幅ラベルのMarkerArray。未確認の観測幅は灰色の `observed=... (unconfirmed)` 表示。

- 新規処理内で、候補の進入軸と実 `L_tcp` の軸方向の差、幅から対称直動指への換算、IK後の残差を含むグリッパ掃引検査に対応。
- 点群検索は既存の5 cmボクセル索引を再利用し、全探索姿勢の指・基部・進入掃引を包含するAABBから点を抽出。点の間引きなし。

## 2. 条件・検証

既存の `grasp_joint_candidates.launch.py` と並べて、追加ノードだけを起動する。

```bash
ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py
```

幅・接触評価だけを確認する場合：

```bash
ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py enable_ik:=false
```

元候補のID・順序を保持し、失敗や予算超過も候補ごとの `reason` として返す。既存出力の置換、経路の再配信、関節指令の発行はない。未起動時は追加の点群購読・計算なし。起動時は別プロセスとしてCPU・点群転送の負荷が増える。

| 方向 | 既定トピック | 型・用途 |
| --- | --- | --- |
| 入力 | `/grasp_pose_cands` | `GraspCandidateArray`：元のTCP候補 |
| 入力 | `/camera/camera/depth/color/points` | `PointCloud2`：実点群 |
| 入力 | `/ToPoDualArm/grasp_candidate_metrics` | `GraspCandidateMetricArray`：既存関節姿勢をIK初期値として利用 |
| 出力 | `/grasp_pose_refined` | 新規 `gng_control_msgs/msg/GraspRefinementArray` |
| 出力 | `/grasp_pose_refined/markers` | `visualization_msgs/msg/MarkerArray`：RViz等で追加表示 |

launch引数は `params_file`、`candidate_topic`、`point_cloud_topic`、`seed_topic`、`output_topic`、`enable_ik`。設定の正本は [grasp_candidate_refinement.yaml](../../config/grasp_candidate_refinement.yaml)。設定は起動時に読み込む。新しい結果を既存Viewerの候補ロボットへ取り込む処理は含まない。

主な設定：

| 用途 | 設定と既定値 |
| --- | --- |
| 更新・計算量 | `update_hz=2.0`、`max_candidates=20`、`max_cloud_points=1000000`、`max_local_points=30000` |
| 入力整合 | `max_input_age_sec=2.0`、`max_stamp_diff_sec=0.5` |
| 実機の内幅 | `min_width=0.005`、`max_width=0.074`、`closed_width=0.0` [m] |
| 指・基部形状 | `finger_span=0.061`、`finger_length=0.0883`、`finger_thickness=0.034`、基部の寸法・TCP内Z範囲 [m] |
| 開口・掃引 | `opening_margin=0.003`、`collision_margin=0.001`、`approach_length=0.060` [m] |
| 接触支持 | `min_contact_points=8`、`contact_band=0.002`、`min_contact_spread=0.002`、`max_contact_variation=0.08`、`max_contact_normal_deg=30` |
| 局所探索 | `yaw_offsets_deg=[-15,0,15]`、`insertion_depths=[0.025,0.040,0.055]` [m]、`max_center_shift=0.025` [m] |
| TCP軸補正 | `tcp_rotation_x_deg=180`。既存の+Z進入候補から、+Zが基部側の実TCPへの変換。実TCP姿勢を直接入力する場合は0 |
| IK | `max_ik_seeds=3`、`max_ik_iter=120`、`max_seed_dist=0.15` [m]、`max_joint_change_deg=25`、`max_position_error=0.002` [m]、`max_orientation_error_deg=3` |
| 関節鎖 | `root_link=L_shoulder_mount`、`root_frame=ToPoDualArm/L_shoulder_mount`、`tcp_frame=L_tcp`、ToPoDualArmのURDFと左指2関節 |

幅の単位はm。`observed_width` は局所領域内で見えた点の閉じ軸方向の幅で、物体全幅の保証はない。両側の支持点から接触条件を確認できた場合だけ `has_contact_pair=true` とし、`contact_width` と `opening_width` を設定する。未計算はNaN。開口幅は観測幅に左右それぞれ3 mmの余裕を追加し、74 mmへ切り詰めて成立扱いにする処理はない。

配列内の `source_candidate_id` は元候補ID、`seed_node_id` はIKに採用したアームGNGノードID。`source_pose` は元候補、`refined_pose` は補正した目標TCP姿勢、`joint_pose` は出力関節角のFK姿勢。`joint_state` は7軸と進入時の指開口関節値を内包する。`has_joint_solution` はIK成立のみを表し、`has_arm_path_check` は現実装では常にfalse。

`header` は元候補の座標系と使用点群時刻、`source_header` と `cloud_header` は各入力のヘッダー。TF欠損時の座標すり替えなし。候補・点群の時刻差、購読の停止、重複ID、不正姿勢・点群、空配信を検査し、旧結果を無効化する。結果とMarkerはreliable・transient_local・depth 1、点群購読はSensorDataQoS・depth 1。

`update_ms` は点群索引作成・局所評価・IKを含む更新処理の壁時計時間。ROS配信・Marker作成・購読転送・待ち時間を含まない。

Releaseビルド・インストール、新規8件と既存16件のC++テスト、隔離ROS結合検証に成功。ROS検証では40 mmの合成側面点群と実ToPoDualArm URDFを使用し、把持幅40 mm、開口46 mm、指関節±23 mm、IK位置残差1.567 mm、姿勢残差0.782度を確認した。両側欠損、観測障害物、TF欠損と復帰、入力失効、重複ID、不正クォータニオン、行末padding、初期関節候補なし、空配信、既存出力への非干渉も確認した。

指定bagの `/camera/camera/depth/color/points` から先頭20フレーム（179,915～181,371点）を読み、座標を維持して時刻だけ更新し、10 Hzで25秒間反復投入した。GNGは最大1,545ノード・入力10,000点・学習1,000回。既存の平面抽出・上方把持候補生成を経由し、新規評価だけIKなしで測定した。

| 項目 | 実測値 |
| --- | --- |
| 非空の追加評価 | 48更新、平均9.52候補、最大14候補 |
| 更新計算時間 | 平均11.569 ms、中央値10.967 ms、最大18.701 ms |
| 観測幅 | 延べ457件で出力 |
| 確認済み接触対 | 0件 |
| 棄却理由 | 支持不足306件、開口範囲外97件、対向法線条件不成立54件 |

この実点群では確定した把持幅・関節補正の成功を確認できていない。単一視点・局所探索・幅制限等を含む当該入力の結果であり、物体別の成功率検証ではない。初期試作の平均96.349 msから検索負荷を削減したが、GNG学習による候補集合が異なるため、同一候補による厳密な速度比には使わない。

検証コマンド（コンテナ `gng_cpu_container` 内）：

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
colcon build --packages-select gng_control_msgs --symlink-install --executor sequential
cmake -S /ros2_ws/src/gng_vlut_system -B /ros2_ws/build/gng_vlut_system
cmake --build /ros2_ws/build/gng_vlut_system --target grasp_candidate_refiner_node test_grasp_refinement test_grasp_candidate_reachability test_candidate_metric_availability -j2
cd /ros2_ws/build/gng_vlut_system
ctest --output-on-failure -R '^(test_grasp_refinement|test_grasp_candidate_reachability|test_candidate_metric_availability)$'
cmake --install .
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_refinement.py --output /ros2_ws/src/tmp/grasp_refinement_20260915/integration_verified.json
python3 /ros2_ws/src/tmp/grasp_refinement_20260915/real_pipeline.py
ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py --show-args
```

ROS検証で起動した子プロセスは以下。合成検証の一時YAMLパスは毎回生成し、終了時に削除。

```bash
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py params_file:=/tmp/grasp_refinement_5um6xfb3/params.yaml candidate_topic:=/grasp_refinement_test/source seed_topic:=/grasp_refinement_test/seeds point_cloud_topic:=/grasp_refinement_test/points output_topic:=/grasp_refinement_test/result
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu --ros-args --params-file /ros2_ws/src/tmp/grasp_refinement_20260915/gng_params.yaml -r topological_map:=/refinement_real/Tmap
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 /ros2_ws/install/grasping_system/lib/grasping_system/top_grasp_surface_estimator_node --ros-args --params-file /ros2_ws/src/tmp/grasp_refinement_20260915/top_params.yaml
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 /ros2_ws/build/gng_vlut_system/src/grasp_candidate_refiner_node --ros-args -p candidate_topic:=/refinement_real/source -p point_cloud_topic:=/refinement_real/points -p output_topic:=/refinement_real/result -p enable_ik:=false
```

起動したdriver・launch・子ノード・有限検証はすべて終了済み。開始前後のプロセス一覧で残留なし、既存プロセスの停止・再起動操作なし。結果と起動ログはワークスペースの `tmp/grasp_refinement_20260915/` 内の `integration_verified.*`、`regression.log`、`real_result.json`、`real_pipeline.log`、`processes_before.log`、`processes_after.log`。実点群検証スクリプトも同ディレクトリに保存。

**制約**

- 接触帯のPCAで局所法線・面内広がりを検査する方式。全物体への曲面モデル当てはめ、摩擦・力閉包・重心の検証はない。
- グリッパ検査は保守的な指・基部の直方体と観測点の検査。`has_observed_collision=false` は未観測領域の空間保証ではない。IK後の実TCP姿勢も検査対象。
- 補正した腕全体の自己衝突・環境衝突、元GNG姿勢から補正姿勢への接続、経路成立は未検証。既存seedの `feasible` を補正結果へ継承しない。
- 左腕・対称直動指の設定が既定。別ロボットではURDF・TCP軸・指形状・関節名の適合が必要。
- 元の候補・経路は変更していないため、追加ノードを停止すれば既存構成へ戻る。
