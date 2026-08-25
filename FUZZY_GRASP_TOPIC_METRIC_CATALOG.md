# 把持ルール用ROSトピック・評価指標候補集

## 目的

[把持ファジールール候補集](FUZZY_GRASP_RULE_CATALOG.md)のR001〜R145を実現するために必要なROS入力、派生評価指標、物体仮説および試行結果を整理する。

トピックを指標ごとに増やさず、次の4層へ分離する方針。

1. センサーおよび構造データの生入力
2. 候補・物体・軌道単位の派生評価指標
3. 根拠と信頼度を持つ物体仮説
4. 実行指令および試行結果

## ステータス表記

| 表記 | 意味 |
| --- | --- |
| 既存 | 現在のワークスペースにPublisherまたは利用経路が存在 |
| 条件付き | 実機ドライバやセンサー構成に依存 |
| 派生 | 既存入力から評価ノードで算出可能 |
| 新規候補 | 新しいtopicまたはmessage契約が必要 |

## 推奨データフロー

```text
点群・深度・GNG・ボクセル・平面クラスタ
                    │
ハンド包絡graph ───┼─→ grasp_candidate_feature_evaluator
                    │             │
把持姿勢・IK・経路 ┘             ├─→ /evaluation_metrics
                                  │     scope=candidate
関節・力覚・対象追跡 ─→ interaction_estimator
                                  ├─→ /grasp_trial_events
                                  └─→ /object_hypotheses
                                              │
                                              └─→ fuzzy_action_selector
```

`/evaluation_metrics`を数値評価の共通搬送路として維持し、個別metricごとのtopicは作らない。形状graph、姿勢集合、時系列イベントおよび根拠履歴は、構造を失うため`EvaluationMetrics`へflattenしない。

## 1. 現在利用可能な主なトピック

### 1.1 認識・形状

| ステータス | Topic | Message | 主な内容 | 対応ルール |
| --- | --- | --- | --- | --- |
| 既存 | GNG入力点群topic | `sensor_msgs/msg/PointCloud2` | 対象・環境の観測点群 | R001〜R012 |
| 既存 | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 深度と自由空間の観測 | R007〜R009、R047〜R048 |
| 既存 | `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | ボクセルの画像投影 | R007〜R009 |
| 既存 | `/ToPoDualArm/topological_map_static`または設定済みGNG topic | `ais_gng_msgs/msg/TopologicalMap` | GNGノード、edge、法線、ラベル | R001〜R012、R049〜R057 |
| 既存 | `/topo_voxel_ids` | `voxel_msgs/msg/Voxel` | 対象候補ボクセルID、ラベル、解像度 | R001〜R009、R021〜R048 |
| 既存 | `/topo_voxel_ids/edge_inferred` | `voxel_msgs/msg/Voxel` | edge補間由来の占有 | R003〜R012 |
| 既存 | `/topo_voxel_ids/triangle_inferred` | `voxel_msgs/msg/Voxel` | triangle補間由来の占有 | R003〜R012 |
| 既存 | `/topo_voxel_ids/isolated` | `voxel_msgs/msg/Voxel` | 孤立候補セル | R004、R011 |
| 既存 | `/topological_planar_clusters_incremental` | `ais_gng_msgs/msg/PlanarClusterArray` | 平面クラスタ、法線、広がり、平面度 | R010、R049〜R057 |

### 1.2 ハンド包絡形状

`{side}`は`L`または`R`。ハンドが増える場合は、左右固定ではなく`hand_id`からtopic namespaceを解決する。

| ステータス | Topic | Message | 主な内容 | 対応ルール |
| --- | --- | --- | --- | --- |
| 既存 | `/ToPoDualArm/{side}_grip_V_topological_map` | `ais_gng_msgs/msg/TopologicalMap` | 最大把持領域 | R013〜R034 |
| 既存 | `/ToPoDualArm/{side}_grip_minV_topological_map` | `ais_gng_msgs/msg/TopologicalMap` | 小さすぎる対象を除く最小把持領域 | R021〜R027 |
| 既存 | `/ToPoDualArm/{side}_grip_baseV_topological_map` | `ais_gng_msgs/msg/TopologicalMap` | ハンド基部の禁止領域 | R041〜R048 |
| 既存 | `/ToPoDualArm/{side}_grip_sweptV_topological_map` | `ais_gng_msgs/msg/TopologicalMap` | ハンド作動中の掃引禁止領域 | R038、R041〜R048 |
| 新規候補 | `/grasp_family_profiles` | `grasping_msgs/msg/GraspFamilyProfileArray` | family ID、包絡graph topic、許可延長方向、変形範囲、目標ハンド状態 | R013〜R040 |

### 1.3 把持候補

| ステータス | Topic | Message | 主な内容 | 対応ルール |
| --- | --- | --- | --- | --- |
| 既存 | `/grasp_pose_cands` | `geometry_msgs/msg/PoseArray` | 把持候補TCP姿勢 | R058〜R064 |
| 既存 | `/grasp_pose_cand_cells` | `voxel_msgs/msg/Voxel` | 代表化済みTCP候補セル | R058〜R064 |
| 既存 | `/grasp_pose_cand_scores` | `std_msgs/msg/Float32MultiArray` | 候補姿勢スコア | R136〜R145 |
| 既存 | `/grasp_pose_cands/summary` | `std_msgs/msg/String` | 照合数、占有率、法線・平面根拠などのJSON | R021〜R057 |
| 新規候補 | `/grasp_pose_regions` | `grasping_msgs/msg/GraspPoseRegionArray` | family、連結領域、代表姿勢、許容6DoF幅、候補ID対応 | R058〜R064 |

`/grasp_pose_cands/summary`は診断表示には使用できるが、文字列JSONのため安定した機械入力には向かない。候補単位の数値は`/evaluation_metrics`へ移し、姿勢領域の構造だけを`GraspPoseRegionArray`へ保持する構成を推奨。

### 1.4 IK・経路・関節

| ステータス | Topic | Message | 主な内容 | 対応ルール |
| --- | --- | --- | --- | --- |
| 既存 | `/ToPoDualArm/joint_states` | `sensor_msgs/msg/JointState` | 現在関節位置、速度、利用可能ならeffort | R065〜R075、R083〜R124 |
| 既存 | `/ToPoDualArm/grasp_candidate_metrics` | `gng_control_msgs/msg/GraspCandidateMetricArray` | 候補別IK、関節余裕、衝突余裕、経路品質 | R065〜R082 |
| 既存 | `/evaluation_metrics` | `gng_control_msgs/msg/EvaluationMetrics` | HTMLへ渡す動的な評価schemaと候補sample | R001〜R082、R136〜R145 |
| 既存 | `/ToPoDualArm/cand_topological_map` | `ais_gng_msgs/msg/TopologicalMap` | 候補経路graph | R076〜R082 |
| 既存 | `/ToPoDualArm/plan_topological_map` | `ais_gng_msgs/msg/TopologicalMap` | 選択済み経路graph | R076〜R082 |
| 既存 | `/ToPoDualArm/target_joint_states` | `sensor_msgs/msg/JointState` | 目標関節状態 | R073〜R082 |
| 既存 | `/ToPoDualArm/joint_commands` | `sensor_msgs/msg/JointState` | 実行用関節指令 | R083〜R124 |

### 1.5 力覚・接触・対象追跡

この領域はハードウェア依存。標準topic名を固定せず、起動引数で実機topicをremapする。

| ステータス | 推奨論理topic | Message | 主な内容 | 対応ルール |
| --- | --- | --- | --- | --- |
| 条件付き | `/ToPoDualArm/{side}/wrench` | `geometry_msgs/msg/WrenchStamped` | 手首力・トルク | R083〜R124 |
| 条件付き | `/ToPoDualArm/{side}/hand_joint_states` | `sensor_msgs/msg/JointState` | ハンド位置、速度、effort | R083〜R107 |
| 新規候補 | `/ToPoDualArm/{side}/tactile_state` | `grasping_msgs/msg/TactileState` | 接触領域、圧力、滑り | R090、R093、R100〜R107 |
| 新規候補 | `/tracked_object_components` | `grasping_msgs/msg/TrackedObjectComponentArray` | 物体成分ID、姿勢、速度、対応GNGノード・ボクセル | R088〜R124 |
| 新規候補 | `/grasp_trial_events` | `grasping_msgs/msg/GraspTrialEventArray` | 指令、観測、停止理由、試行結果 | R083〜R135 |
| 新規候補 | `/object_hypotheses` | `grasping_msgs/msg/ObjectHypothesisArray` | 物体物性・可動性仮説、信頼度、根拠参照 | R083〜R135 |

## 2. `/evaluation_metrics`へ追加する候補指標

### 2.1 共通仕様

- 候補単位の`scope_type`: `candidate`
- 物体単位の`scope_type`: `object_component`
- 姿勢領域単位の`scope_type`: `grasp_pose_region`
- 経路単位の`scope_type`: `path`
- 正規化済み評価値の基本範囲: `[0, 1]`
- 距離の単位: `m`
- 角度の単位: `rad`
- 力の単位: `N`
- トルクの単位: `N m`
- 質量の単位: `kg`
- 値を生成できない場合: `sample_metric_valid=false`
- 未観測値を`0`として送信しない
- 低いほど良い生値は、`*_risk`または`*_occupancy_ratio`として方向を名前で明示
- 高いほど良い評価値は、`*_fit`、`*_margin`、`*_confidence`として統一

### 2.2 観測品質指標

| metric ID | 範囲 | 算出元 | 意味 | 対応ルール |
| --- | --- | --- | --- | --- |
| `voxel_observation_density` | `[0,1]` | 対象ボクセル、点群支持数 | 対象局所領域の観測密度 | R001〜R002 |
| `gng_support_density` | `[0,1]` | GNGノード数、局所体積 | 対象形状を支持するGNG密度 | R001〜R002 |
| `temporal_presence_stability` | `[0,1]` | ボクセル・GNG時系列 | 複数フレームの在席安定度 | R003〜R006 |
| `shape_frame_consistency` | `[0,1]` | フレーム間局所形状差 | 観測形状の一貫性 | R005〜R006 |
| `unobserved_ratio` | `[0,1]` | 深度可視性 | 評価領域内の未観測割合 | R007〜R009 |
| `free_space_visibility` | `[0,1]` | 深度、CameraInfo、TF | 掃引領域が自由空間として確認された割合 | R008〜R009 |
| `normal_consistency` | `[0,1]` | GNGノード法線 | 局所法線の一貫性 | R010、R054 |
| `isolated_support_ratio` | `[0,1]` | 孤立セル、支持セル | 孤立した形状根拠の割合 | R004、R011 |
| `component_connectivity` | `[0,1]` | GNG edge、時系列 | 物体成分の連結安定度 | R012 |
| `observation_confidence` | `[0,1]` | 上記指標の統合 | 候補形状全体の観測信頼度 | R047〜R048、R136 |

### 2.3 Envelope適合指標

| metric ID | 範囲 | 算出元 | 意味 | 対応ルール |
| --- | --- | --- | --- | --- |
| `envelope_fit` | `[0,1]` | 対象占有、family包絡 | familyへの総合適合度 | R013〜R020、R052〜R053 |
| `family_ambiguity` | `[0,1]` | family別適合度分布 | family選択の曖昧さ | R014〜R016 |
| `deformation_req` | `[0,1]` | family基準形状、対象形状 | 必要なハンド変形量 | R019〜R020 |
| `deformation_margin` | `[0,1]` | ハンド可動・変形範囲 | 必要変形に対する余裕 | R020、R030〜R031 |
| `min_envelope_excess` | `[0,1]` | `min_envelope`外側の占有 | 対象が小さすぎない根拠 | R021〜R027 |
| `max_envelope_overflow` | `[0,1]` | `max_envelope`外側の占有 | 許可領域以外へのはみ出し | R023〜R027 |
| `envelope_fill` | `[0,1]` | 包絡内対象占有 | 包絡が対象で支持される割合 | R028〜R031 |
| `fit_margin` | `[0,1]` | min・max包絡境界距離 | サイズ適合境界からの余裕 | R025〜R027 |
| `centering_fit` | `[0,1]` | 包絡中心、対象局所中心 | 包絡中央への整列度 | R032〜R034、R089 |
| `allowed_continuation_fit` | `[0,1]` | 対象延長、許可方向 | 長尺部分の許可方向への適合度 | R035〜R039 |
| `forbidden_direction_overflow` | `[0,1]` | 対象延長、禁止方向 | 禁止方向への延長量 | R036、R038 |
| `center_of_mass_offset` | `m` | 重心推定、TCP | 把持局所部分からの重心距離 | R040、R122 |

### 2.4 掃引・衝突・分離指標

| metric ID | 範囲 | 算出元 | 意味 | 対応ルール |
| --- | --- | --- | --- | --- |
| `forbidden_occupancy_ratio` | `[0,1]` | 禁止graph、環境占有 | 禁止体積に入る環境占有割合 | R041、R044〜R046 |
| `swept_clearance` | `m` | 掃引graph、環境占有 | 作動掃引体積から環境までの余裕 | R042〜R048 |
| `base_clearance` | `m` | 基部graph、環境占有 | ハンド基部から環境までの余裕 | R042、R044 |
| `target_only_occupancy_ratio` | `[0,1]` | 対象ID付き占有 | 掃引・接触許可領域内で対象が占める割合 | R045〜R046 |
| `planar_support_ratio` | `[0,1]` | 平面クラスタ、GNG対応 | 接触予定領域の平面支持率 | R049〜R051 |
| `surface_curvature_fit` | `[0,1]` | GNG局所形状、family | familyに対する曲率適合度 | R052〜R053 |
| `surface_fragmentation` | `[0,1]` | 平面クラスタ分断数 | 接触予定面の分断度 | R055 |
| `object_separation` | `[0,1]` | GNG成分、環境占有 | 対象と周辺成分の分離度 | R056〜R057、R112〜R113 |
| `floor_separation` | `m` | 床クラスタ、対象成分 | 対象と床の局所距離 | R056 |
| `wall_separation` | `m` | 壁クラスタ、対象成分 | 対象と壁の局所距離 | R057 |

### 2.5 6DoF姿勢領域指標

| metric ID | 範囲 | 算出元 | 意味 | 対応ルール |
| --- | --- | --- | --- | --- |
| `pose_region_volume` | 正値 | 姿勢集合クラスタ | 有効6DoF姿勢領域の大きさ | R058〜R059 |
| `pose_region_boundary_margin` | `[0,1]` | 候補と領域境界 | 候補が姿勢領域内部にある程度 | R060〜R061 |
| `pose_region_candidate_num` | 0以上 | 姿勢集合クラスタ | 同一領域内の候補数 | R062 |
| `pose_region_num` | 0以上 | 姿勢集合クラスタ | 分離した有効姿勢領域数 | R063 |
| `pose_region_diversity` | `[0,1]` | family、位置、姿勢差 | IKへ送る候補群の多様性 | R063〜R064 |

`pose_region_volume`は位置3軸と回転3軸の単位が異なるため、絶対的な6次元体積ではなく、familyごとに正規化した離散セル数または許容幅の積として扱う。

### 2.6 IK・経路指標

次の指標は既存`GraspCandidateMetric`または`/evaluation_metrics`に存在する。

| metric ID | 状態 | 意味 | 対応ルール |
| --- | --- | --- | --- |
| `position_manipulability` | 既存 | 並進マニピュラビリティ | R069、R081 |
| `rotation_manipulability` | 既存 | 回転マニピュラビリティ | R069、R081 |
| `joint_limit_margin_min` | 既存 | 全関節中で最も小さい関節限界余裕 | R067〜R068 |
| `joint_limit_margin_mean` | 既存 | 関節限界余裕の平均 | R067〜R068 |
| `self_collision_margin` | 既存、現状NaNの場合あり | 自己衝突余裕 | R071 |
| `environment_collision_margin` | 既存、現状NaNの場合あり | 環境衝突余裕 | R072、R077 |
| `gripper_width` | 既存、平行グリッパー固有 | 必要開口幅 | family固有ルール |
| `grasp_region_score` | 既存、現状NaNの場合あり | 把持領域品質 | R058〜R064 |
| `estimated_energy` | 既存 | 関節角変化量に基づく推定エネルギー | R080 |
| `estimated_duration` | 既存 | 推定所要時間 | R080 |
| `path_position_manipulability` | 既存 | 経路上の並進マニピュラビリティ列 | R081 |
| `path_rotation_manipulability` | 既存 | 経路上の回転マニピュラビリティ列 | R081 |

追加候補は次のとおり。

| metric ID | 範囲 | 算出元 | 意味 | 対応ルール |
| --- | --- | --- | --- | --- |
| `ik_solution_num` | 0以上 | IK solver | 候補姿勢に対応するIK解数 | R065〜R066 |
| `joint_motion_cost` | 正値 | 現在角、IK解 | 現在状態から目標状態までの関節移動量 | R073〜R074 |
| `force_capability_margin` | `[0,1]` | Jacobian、関節トルク余裕 | 把持・持ち上げ方向の力発生余裕 | R070、R097、R115 |
| `post_grasp_motion_margin` | `[0,1]` | 把持後IK、局所探索 | 持ち上げ・搬送方向の可動余裕 | R074〜R075 |
| `approach_clearance` | `m` | 進入経路、環境占有 | 接近経路の空間余裕 | R077〜R079 |
| `retreat_clearance` | `m` | 離脱経路、環境占有 | 把持後離脱経路の空間余裕 | R082 |
| `min_path_collision_margin` | `m` | 経路全体 | 経路上で最も小さい衝突余裕 | R077〜R079 |
| `min_path_position_manipulability` | 正値 | 経路評価 | 経路上で最も小さい並進マニピュラビリティ | R081 |
| `min_path_rotation_manipulability` | 正値 | 経路評価 | 経路上で最も小さい回転マニピュラビリティ | R081 |
| `path_narrow_passage_ratio` | `[0,1]` | 経路、局所自由空間 | 狭い空間を通る経路区間の割合 | R078 |
| `has_reachable_path` | bool | 経路探索 | 現在状態から候補領域へ到達可能 | R076 |

`has_reachable_path=false`、IK解なし、衝突確定などはファジー入力ではなくハード制約として使用する。

### 2.7 把持試行の観測指標

| metric ID | 単位・範囲 | 算出元 | 意味 | 対応ルール |
| --- | --- | --- | --- | --- |
| `hand_position_error` | `rad`または`m` | 指令・実ハンド状態 | 予測した閉鎖位置との差 | R083〜R085 |
| `hand_effort` | driver依存 | ハンドJointState | ハンド駆動負荷 | R083〜R090 |
| `grasp_force` | `N` | 触覚・駆動負荷 | 対象へ加わる推定把持力 | R083〜R107 |
| `object_deformation` | `m` | 対象追跡、ハンド状態 | 把持力による対象変形量 | R085〜R087 |
| `object_centering_motion` | `m` | 対象追跡、包絡中心 | 閉鎖中の自己整列量 | R088〜R089 |
| `holding_force_stability` | `[0,1]` | 力覚時系列 | 閉鎖後保持力の安定度 | R090 |
| `slip_score` | `[0,1]` | 触覚、対象相対運動 | ハンド内での滑り程度 | R090、R093、R100〜R107 |
| `commanded_displacement` | `m` | 試行指令 | 探索動作の指令変位 | R091〜R100 |
| `end_effector_displacement` | `m` | FK、TF | 実際の手先変位 | R092〜R099 |
| `object_displacement` | `m` | 対象追跡 | 実際の対象変位 | R092〜R100 |
| `object_tracking_ratio` | `[0,1]` | 手先・対象変位 | 対象が手先へ追従する割合 | R092〜R100 |
| `wrist_force` | `N` | WrenchStamped | 手首合力 | R094〜R100、R108〜R111 |
| `wrist_torque` | `N m` | WrenchStamped | 手首合トルク | R094〜R100 |
| `object_acceleration` | `m/s^2` | 対象追跡 | 質量推定用の対象加速度 | R096〜R098、R109 |
| `force_motion_consistency` | `[0,1]` | 力、加速度、変位 | 質量推定モデルとの整合度 | R096〜R099 |
| `motion_direction_consistency` | `[0,1]` | 複数方向試行 | 方向ごとの可動性の一貫性 | R098〜R099、R108〜R111 |

## 3. 物体仮説として保持する指標

物体仮説は候補姿勢ではなく`object_component_id`へ紐付ける。確率の合計を1へ強制する競合仮説と、同時成立可能な属性を区別する。

### 3.1 競合する状態仮説

| hypothesis ID | 範囲 | 意味 | 対応ルール |
| --- | --- | --- | --- |
| `movable_prob` | `[0,1]` | 可動物体である確率 | R088、R092、R117〜R118 |
| `fixed_prob` | `[0,1]` | 環境へ固定されている確率 | R094、R098、R108、R114 |
| `obstructed_prob` | `[0,1]` | 周辺物体や構造へ拘束されている確率 | R094〜R095、R099、R111 |
| `grasp_failure_prob` | `[0,1]` | 対象を保持できていない確率 | R084、R093、R103 |
| `tracking_failure_prob` | `[0,1]` | 対象追跡の誤りである確率 | R006、R093〜R100 |

### 3.2 同時成立可能な物性・状態

| hypothesis ID | 単位・範囲 | 意味 | 対応ルール |
| --- | --- | --- | --- |
| `mass_estimate_kg` | `kg` | 推定質量 | R033、R040、R096〜R097、R109、R115〜R117 |
| `mass_min_kg` | `kg` | 質量推定範囲の下端 | 同上 |
| `mass_max_kg` | `kg` | 質量推定範囲の上端 | 同上 |
| `mass_confidence` | `[0,1]` | 質量推定の信頼度 | R096〜R097 |
| `friction_estimate` | `[0,1]` | 接触面の相対的な摩擦推定 | R100〜R107 |
| `friction_confidence` | `[0,1]` | 摩擦推定の信頼度 | R100〜R107 |
| `compliance_estimate` | `[0,1]` | 対象の変形しやすさ | R085〜R087、R120〜R121 |
| `compliance_confidence` | `[0,1]` | コンプライアンス推定の信頼度 | R085〜R087 |
| `fragility_prob` | `[0,1]` | 低い力でも損傷する可能性 | R119 |
| `self_alignment_score` | `[0,1]` | 閉鎖時に包絡中央へ馴染む程度 | R089 |
| `direction_constraint_score` | `[0,1]` | 特定方向へ拘束される程度 | R099、R110〜R111 |
| `grasp_success_confidence` | `[0,1]` | 現在把持が成立している信頼度 | R090〜R109、R117 |

### 3.3 仮説根拠として必要なメタデータ

各仮説値に次を関連付ける。

| field | 内容 |
| --- | --- |
| `object_component_id` | 仮説対象の物体成分ID |
| `hypothesis_revision` | 単調増加する仮説更新番号 |
| `source_event_ids` | 更新根拠となった試行イベントID |
| `source_stamps` | 根拠観測時刻 |
| `valid_until` | 再観測がない場合の有効期限 |
| `update_reason` | 人が確認できる更新理由 |
| `model_version` | 更新器およびパラメータの版 |

## 4. 新規message候補の最小構造

### 4.1 `GraspFamilyProfile`

```text
string hand_id
string family_id
string max_envelope_topic
string min_envelope_topic
string forbidden_volume_topic
string swept_volume_topic
geometry_msgs/Vector3[] allowed_continuation_dirs
sensor_msgs/JointState hand_goal_state
float32 deformation_margin
```

包絡本体は既存`TopologicalMap`を再利用し、profile messageへ巨大なgraphを重複格納しない。

### 4.2 `GraspPoseRegion`

```text
string region_id
string family_id
int32 object_component_id
geometry_msgs/Pose representative_pose
geometry_msgs/Pose[] sampled_poses
float32[] position_tolerance
float32[] rotation_tolerance
float32 region_score
```

`PoseArray`との順序対応だけに依存せず、`region_id`と候補IDを明示的に対応させる。

### 4.3 `GraspTrialEvent`

```text
string event_id
string trial_id
string action_type
int32 object_component_id
string family_id
string pose_region_id
float32 commanded_displacement
float32 end_effector_displacement
float32 object_displacement
float32 wrist_force
float32 wrist_torque
float32 grasp_force
float32 slip_score
string stop_reason
bool is_safe_stop
```

高周波の生センサー列は格納せず、bag上の元topicと集約区間を参照できるようにする。

### 4.4 `ObjectHypothesis`

```text
int32 object_component_id
uint32 hypothesis_revision
string[] hypothesis_ids
float32[] hypothesis_values
float32[] hypothesis_confidences
string[] source_event_ids
string update_reason
string model_version
```

質量範囲など単位付きの固定fieldと、拡張仮説用配列を併用するかは、実装開始時に確定する。HTMLへ表示する数値だけは`EvaluationMetrics`へ変換可能。

## 5. ルール群と必要入力の対応

| ルール範囲 | 必須入力 | 主な派生指標 | 追加topicの必要性 |
| --- | --- | --- | --- |
| R001〜R012 | 点群、深度、GNG、ボクセル履歴 | 観測密度、在席安定度、連結性、観測信頼度 | 既存topicで開始可能 |
| R013〜R020 | ハンド包絡graph、family定義 | family適合度、曖昧度、変形余裕 | `grasp_family_profiles`を推奨 |
| R021〜R034 | min・max包絡、対象ボクセル | 包絡占有、はみ出し、fit余裕、偏心 | 既存topicで開始可能 |
| R035〜R040 | 対象成分、許可延長方向、重心 | 延長方向適合度、重心offset | family profileと対象成分が必要 |
| R041〜R048 | 禁止・掃引graph、環境占有、深度 | 禁止占有率、掃引余裕、自由空間信頼度 | 既存topicで一部可能 |
| R049〜R057 | 平面クラスタ、GNG、対象成分 | 平面支持、曲率適合、対象分離 | 対象成分ID付与を推奨 |
| R058〜R064 | 6DoF候補集合、family | 姿勢領域、境界余裕、多様性 | `grasp_pose_regions`を推奨 |
| R065〜R082 | IK、関節状態、候補経路 | IK解数、関節余裕、経路余裕、実行品質 | 既存topicで大半が可能 |
| R083〜R090 | ハンド状態、力覚、対象追跡 | 閉鎖誤差、変形、自己整列、保持安定度 | 力覚・追跡topicが必要 |
| R091〜R100 | 試行指令、手先・対象変位、力覚 | 追従率、滑り、力と運動の整合度 | `grasp_trial_events`を推奨 |
| R101〜R107 | 触覚、把持力、対象相対運動 | 滑り、摩擦、接触形状不適合 | 触覚または代替推定が必要 |
| R108〜R124 | 複数方向試行、物体仮説 | 可動・固定・拘束・重量物仮説 | `object_hypotheses`が必要 |
| R125〜R135 | 試行履歴、ルール版、結果 | 成功率、失敗原因信頼度、更新量 | 永続ログが必要 |
| R136〜R145 | 全品質と仮説 | 総合選好、次行動 | 上記評価の集約で可能 |

## 6. 最小実装セット

最初から全指標を実装せず、次の順序を推奨する。

### Phase 1: 観測と包絡の候補評価

既存topicだけで開始可能。

```text
observation_confidence
envelope_fit
min_envelope_excess
max_envelope_overflow
envelope_fill
forbidden_occupancy_ratio
swept_clearance
planar_support_ratio
normal_consistency
```

対象ルールはR001〜R057とR136の一部。

### Phase 2: 姿勢領域と実行可能性

```text
pose_region_boundary_margin
pose_region_diversity
ik_solution_num
joint_limit_margin_min
force_capability_margin
post_grasp_motion_margin
min_path_collision_margin
estimated_energy
estimated_duration
```

対象ルールはR058〜R082、R137〜R145。

### Phase 3: 安全な把持試行と物体仮説

```text
grasp_force
slip_score
object_tracking_ratio
wrist_force
object_acceleration
movable_prob
fixed_prob
obstructed_prob
mass_estimate_kg
mass_confidence
compliance_estimate
```

対象ルールはR083〜R124。力覚、対象追跡、試行イベントおよび安全停止が揃ってから有効化する。

### Phase 4: 制約付きのルール洗練

```text
trial_success_rate
failure_cause_confidence
rule_support_num
rule_update_confidence
```

対象ルールはR125〜R135。ルール構造と安全制約は固定し、重みとMembership Function境界だけを履歴付きで更新する。

## 7. 実装上の注意

1. `/evaluation_metrics`の`sample_scope_ids`を候補ID、物体成分ID、姿勢領域IDと混同しないよう、`scope_type`ごとに名前空間を分離。
2. `PoseArray`とscore配列の順序対応だけに依存せず、永続的な候補IDを追加。
3. `/grasp_pose_cands/summary`のJSONは診断用とし、ファジー入力は型付きmetricへ移行。
4. GNGノードIDと配列添字を混同せず、平面クラスタの`node_indices`は入力map内の添字として解釈。
5. `JointState.effort`が空または推定値の場合、実力覚として扱わない。
6. 手首反力だけで重量物と固定物を確定せず、滑り、対象追従、複数方向試行およびロボット力余裕を併用。
7. 未観測、未実装、NaNを低評価の`0`へ変換しない。
8. `[H]`ルール用の安全値は学習対象外。
9. 高周波センサー値と集約metricの時刻区間を追跡可能にする。
10. 物体仮説更新は、更新前後の値、根拠イベント、更新理由、モデル版を保存。

