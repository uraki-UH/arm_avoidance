# gng_vlut_system Technical Specification

この文書は `gng_vlut_system` の把握に必要な変数、トピック、サービス、内部状態、データフローをまとめた技術仕様書です。

対象の中心は `grasp_goal_planning.launch.py` を起点とする把持候補選定・候補軌道生成・評価指標 publish の系統です。

## 1. システム概要

`grasp_goal_planning.launch.py` は次の 4 系統をまとめて起動します。

1. `topological_map_goal_selector.launch.py`
2. `robot_spawn.launch.py`
3. `topological_map_avoidance.launch.py`
4. `virtual_joint_state_driver.launch.py` ただし `enable_motion:=true` のときのみ

補助として、`publish_world_tf:=true` の場合に static TF を追加します。

## 2. 起動フロー

```mermaid
flowchart TD
    A[grasp_goal_planning.launch.py] --> B[topological_map_goal_selector.launch.py]
    A --> C[robot_spawn.launch.py]
    A --> D[topological_map_avoidance.launch.py]
    A --> E[virtual_joint_state_driver.launch.py]
    A --> F[static_transform_publisher]

    B --> G[selected_goal_candidate_ids]
    B --> H[selected_topological_map]
    B --> I[selected_topological_map_markers]
    B --> J[grasp_pose_candidates / grasp_pose_scores]

    D --> K[candidate_topological_map]
    D --> L[planned_topological_map]
    D --> M[grasp_candidate_metrics]
    D --> N[current_ee_pose]
    D --> O[target_joint_states]
    D --> P[control_claims]

    O --> E
    E --> Q[joint_states]
```

## 3. 変数一覧

### 3.1 `grasp_goal_planning.launch.py` の launch 引数

| 変数 | 型 | デフォルト | 用途 |
|---|---:|---|---|
| `params_file` | path | `config/ToPoDualArm2.yaml` | URDF と各種パラメータの参照元 |
| `topological_map_topic` | topic | `/ToPoDualArm/topological_map_static` | 目標候補選定の入力マップ |
| `output_topic` | topic | `/selected_topological_map` | 選定後マップ |
| `marker_topic` | topic | `/selected_topological_map_markers` | 選定後のマーカ |
| `goal_candidate_ids_topic` | topic | `/selected_goal_candidate_ids` | 把持候補として採択したノード ID 群 |
| `robot_name` | string | `ToPoDualArm` | namespace と topic 接頭辞 |
| `urdf_path` | path | 空 | ロボットモデル参照 |
| `enable_joint_state_publisher` | bool | `false` | `robot_spawn` での JointState Publisher 有無 |
| `enable_motion` | bool | `true` | 実際に target を駆動するか |
| `candidate_count` | int | `8` | 候補ノード数 |
| `non_collision_only` | bool | `true` | 衝突ノードを候補から除外 |
| `orientation_weight` | float | `0.0` | 向き一致度の重み |
| `target_pose_topic` | topic | 空 | 単一目標姿勢入力 |
| `target_point_topic` | topic | 空 | 単一目標位置入力 |
| `target_pose_array_topic` | topic | `/grasp_pose_candidates` | 目標姿勢配列入力 |
| `target_score_topic` | topic | `/grasp_pose_scores` | 目標姿勢のスコア配列 |
| `node_feature_topic` | topic | `/ToPoDualArm/topological_node_features` | ノード特徴量入力 |
| `manipulability_weight` | float | `0.25` | 可操作性ペナルティ重み |
| `joint_topic` | topic | `/ToPoDualArm/joint_states` | 現在姿勢入力 |
| `initial_joint_names_csv` | csv | 組み込み値 | 仮想関節初期化名 |
| `trajectory_topic` | topic | `/ToPoDualArm/planned_topological_map` | 確定経路の出力 |
| `candidate_trajectory_topic` | topic | `/ToPoDualArm/candidate_topological_map` | 候補経路の出力 |
| `candidate_metrics_topic` | topic | `/ToPoDualArm/grasp_candidate_metrics` | 候補評価指標の出力 |
| `publish_hz` | float | `20.0` | avoidance node の publish 周波数 |
| `avoid_collisions` | bool | `true` | 衝突ノードを避ける |
| `avoid_danger` | bool | `true` | danger ノードを避ける |
| `allow_danger_goal` | bool | `true` | 最終ゴールとして danger を許可 |
| `goal_rot_manip_weight` | float | `1.0` | ゴール姿勢の回転可操作性重み |
| `goal_joint_limit_weight` | float | `0.5` | ゴール姿勢の関節限界余裕重み |
| `strict_goal_collision_check` | bool | `false` | 目的地の衝突判定を厳格化 |
| `replan_on_path_collision` | bool | `false` | 進行中経路が危険なら再計画 |
| `allow_zero_initial_joint_state` | bool | `true` | 初期 joint_state が無いときゼロ初期値を使う |
| `publish_target_joint_states` | bool | `true` | target_joint_states の publish 有無 |
| `allow_safe_goal_fallback` | bool | `true` | 候補が無いとき安全ノードへ逃がすか |
| `virtual_joint_state_publish_hz` | float | `50.0` | 仮想関節追従の publish 周波数 |
| `virtual_joint_state_max_joint_velocity` | float | `0.6` | 仮想追従の最大関節速度 |
| `virtual_joint_state_position_tolerance` | float | `0.01` | 目標到達許容差 |
| `virtual_joint_state_use_wraparound` | bool | `true` | 角度 wrap-around 補正 |
| `publish_world_tf` | bool | `false` | world -> base_link static TF の publish |

### 3.2 `topological_map_goal_selector.launch.py` の引数

| 変数 | 型 | デフォルト | 用途 |
|---|---:|---|---|
| `topological_map_topic` | topic | `/ToPoDualArm/topological_map_static` | 元マップ入力 |
| `output_topic` | topic | `/selected_topological_map` | 選定後マップ |
| `marker_topic` | topic | `/selected_topological_map_markers` | 可視化用マーカ |
| `candidate_count` | int | `8` | 抽出ノード数 |
| `non_collision_only` | bool | `true` | 衝突ノードを除外 |
| `orientation_weight` | float | `0.25` | 姿勢整合重み |
| `target_pose_topic` | topic | 空 | 単一目標姿勢 |
| `target_point_topic` | topic | 空 | 単一目標位置 |
| `target_pose_array_topic` | topic | `/grasp_pose_candidates` | 候補姿勢群 |
| `target_score_topic` | topic | `/grasp_pose_scores` | 姿勢スコア群 |
| `goal_candidate_ids_topic` | topic | `/selected_goal_candidate_ids` | 選定ノード ID の出力 |
| `node_feature_topic` | topic | `/ToPoDualArm/topological_node_features` | manipulability 補正用特徴量 |
| `manipulability_weight` | float | `0.25` | 可操作性補正重み |
| `allow_untransformed_target` | bool | `true` | TF が無くても target を使うか |

### 3.3 `topological_map_avoidance.launch.py` の引数

| 変数 | 型 | デフォルト | 用途 |
|---|---:|---|---|
| `params_file` | path | `config/ToPoDualArm.yaml` | GNG/URDF 参照 |
| `urdf_path` | path | 空 | ロボットモデル |
| `gng_model_path` | path | 空 | GNG モデル直接指定 |
| `topological_map_topic` | topic | `/ToPoDualArm/topological_map_static` | 追跡対象マップ |
| `joint_topic` | topic | `/ToPoDualArm/joint_states` | 現在姿勢入力 |
| `trajectory_topic` | topic | `/ToPoDualArm/planned_topological_map` | 確定経路 |
| `candidate_trajectory_topic` | topic | `/ToPoDualArm/candidate_topological_map` | 候補経路 |
| `candidate_metrics_topic` | topic | `/ToPoDualArm/grasp_candidate_metrics` | 評価指標 |
| `trial_mode` | bool | `false` | trial 動作 |
| `trial_goal_interval_sec` | float | `4.0` | trial の目標切替周期 |
| `trial_safe_only` | bool | `true` | trial で safe のみ使う |
| `trial_return_home` | bool | `false` | home へ戻る試行 |
| `trial_auto_advance_goal` | bool | `false` | trial goal 自動進行 |
| `trial_seed` | int | `0` | trial の乱数シード |
| `avoid_danger` | bool | `true` | danger ノード回避 |
| `allow_danger_goal` | bool | `true` | danger を最終目標として許可 |
| `goal_rot_manip_weight` | float | `1.0` | ゴール評価の回転可操作性重み |
| `goal_joint_limit_weight` | float | `0.5` | ゴール評価の関節限界重み |
| `replan_on_path_collision` | bool | `true` | 進行中の危険経路で再計画 |
| `allow_zero_initial_joint_state` | bool | `true` | 初期 joint_state 無しのゼロ初期化 |
| `publish_target_joint_states` | bool | `true` | target_joint_states 出力 |
| `allow_safe_goal_fallback` | bool | `true` | safe ノード fallback の有無 |
| `goal_candidate_ids_topic` | topic | `/selected_goal_candidate_ids` | 候補 ID 入力 |
| `target_topic` | topic | 空 | target_joint_states 出力先 |
| `robot_base_frame` | frame | 空 | FK 基準フレーム |
| `control_claim_priority` | int | `10` | control claim 優先度 |
| `control_claim_mode` | int | `1` | control claim モード |
| `control_claim_enabled` | bool | `true` | claim publish 有効化 |
| `current_ee_pose_topic` | topic | `/ToPoDualArm/current_ee_pose` | 現在 EE pose 出力 |
| `metrics_max_joint_velocity` | float | `0.6` | 評価時間の見積もり用速度 |

### 3.4 `virtual_joint_state_driver.launch.py` の引数

| 変数 | 型 | デフォルト | 用途 |
|---|---:|---|---|
| `target_topic` | topic | `target_joint_states` | 目標関節角度入力 |
| `state_topic` | topic | `joint_states` | 現在状態入力 |
| `output_topic` | topic | `joint_states` | 出力 joint_states |
| `publish_hz` | float | `50.0` | 更新周期 |
| `max_joint_velocity` | float | `0.6` | 追従速度 |
| `position_tolerance` | float | `0.01` | 到達判定閾値 |
| `use_wraparound` | bool | `true` | wraparound 補正 |
| `ignore_state_after_first_target` | bool | `false` | 初回 target 後の state 上書き抑制 |
| `initial_joint_names_csv` | csv | 空 | 初期 joint 名列 |

## 4. 主要トピック

| トピック | 型 | 役割 |
|---|---|---|
| `/ToPoDualArm/topological_map_static` | `ais_gng_msgs/TopologicalMap` | 元の GNG マップ |
| `/selected_topological_map` | `ais_gng_msgs/TopologicalMap` | target に応じて選ばれたマップ |
| `/selected_goal_candidate_ids` | `std_msgs/Int32MultiArray` | goal 候補 ID の集合 |
| `/grasp_pose_candidates` | `geometry_msgs/PoseArray` | 候補姿勢群 |
| `/grasp_pose_scores` | `std_msgs/Float32MultiArray` | 候補姿勢スコア |
| `/ToPoDualArm/planned_topological_map` | `ais_gng_msgs/TopologicalMap` | 確定した経路 |
| `/ToPoDualArm/candidate_topological_map` | `ais_gng_msgs/TopologicalMap` | 候補経路。現在 EE pose を先頭ノードに含める |
| `/ToPoDualArm/grasp_candidate_metrics` | `gng_control_msgs/GraspCandidateMetricArray` | 候補評価指標 |
| `/ToPoDualArm/current_ee_pose` | `geometry_msgs/PoseStamped` | 現在 EE pose |
| `target_joint_states` | `sensor_msgs/JointState` | 目標関節値 |
| `joint_states` | `sensor_msgs/JointState` | 仮想 or 実機の現在関節値 |
| `/ToPoDualArm/control_claims` | `gng_control_msgs/JointControlClaim` | control claim |

## 5. サービス

| サービス | 用途 |
|---|---|
| `request_trajectory_update` | 軌道再計画を要求する |
| `request_trial_goal_advance` | trial モードで次の goal coordinate へ進める |

## 6. データフロー

```mermaid
flowchart TD
    A[TopologicalMap input] --> B[TopologicalMapGoalSelector]
    C[Target pose / point / pose array] --> B
    D[Node features] --> B
    B --> E[selected_goal_candidate_ids]
    B --> F[selected_topological_map]
    B --> G[selected_topological_map_markers]

    E --> H[TopologicalMapAvoidance]
    F --> H
    I[joint_states] --> H
    H --> J[target_joint_states]
    H --> K[current_ee_pose]
    H --> L[planned_topological_map]
    H --> M[candidate_topological_map]
    H --> N[grasp_candidate_metrics]
    J --> O[virtual_joint_state_driver]
    O --> I
```

## 7. no-motion モード

`enable_motion:=false` のときは次の扱いになります。

- `virtual_joint_state_driver.launch.py` を起動しない
- `publish_target_joint_states:=false`
- `control_claim_enabled:=false`
- `allow_safe_goal_fallback:=false`

このモードでは、候補ノード群・候補軌道・評価指標の生成は維持しつつ、実際の関節更新を止める。

## 8. viewer 側に委ねる見た目

候補ロボットプレビューの見た目は ToPoFuzzy Viewer 側で制御する。
ROS 側はプレビューの送信有無だけを制御し、見た目の指定は送らない。

### 8.1 URDF プレビューの初期ロード

- `stream.robot.description` のトップレベル `robot.urdf` にだけ URDF 本文を格納する。
- `robot.instances[]` はトップレベルの URDF を共有し、各候補の関節値・FK結果・可操作性だけを保持する。
- `stream.robot.pose` は先に受信した description の URDF を Viewer 側で維持するため、URDF 本文を再送しない。
- ToPoFuzzy Viewer は同一 URL のメッシュを1回だけ読み込み・解析し、候補間では geometry を共有する。候補固有の透過度や色が干渉しないよう、Object3D 階層と material は候補ごとに複製する。
- メッシュ到着後の外観反映は到着したオブジェクトだけに行い、タイマーによるロボット全体の反復走査は行わない。
- Viewer のメッシュ配信は package 解決結果とファイル内容をキャッシュする。ファイル内容は更新時刻とサイズが変わった場合に読み直す。
- メッシュHTTP応答は `Cache-Control: no-store` とし、ページリロード時はFrontendのgeometryとブラウザ内のSTL応答を破棄して再取得する。backend内部のファイルキャッシュは維持する。

## 9. AiS-GNG 入力点群の選択

### 9.1 パラメータ

| 変数 | 型 | デフォルト | 用途 |
|---|---:|---|---|
| `input.point_cloud_num` | int | `20000` | GNG に渡す1フレーム当たりの最大点数 |
| `input.sampling_mode` | string | `head` | 上限超過時の選択方式。`head` または `uniform` |

`graspnet.yaml` は `input.point_cloud_num=100000`、`input.sampling_mode=uniform` を使用する。

### 9.2 選択方式

- `head`: 従来どおり点列の先頭から最大 `input.point_cloud_num` 点を使用する。
- `uniform`: 点数上限を超えた場合、点列全体を同数の区間に分け、各区間の中央に相当する点を選ぶ。
- 入力点数が上限以下なら、どちらの方式でも全点をそのまま使用する。

`uniform` の選択元インデックスは、入力点数を `N`、選択点数を `M`、選択順を `i` として次式で求める。

```text
source_index(i) = floor(((2 * i + 1) * N) / (2 * M))
```

### 9.3 データフロー

```mermaid
flowchart TD
    A[PointCloud2] --> B{point count > input.point_cloud_num}
    B -- no --> C[全点を GNG へ入力]
    B -- yes / head --> D[先頭 M 点を GNG へ入力]
    B -- yes / uniform --> E[点列全域から M 点を等間隔抽出]
    E --> F[抽出点を contiguous PointCloud2 に再パック]
    F --> G[GNG voxel filter / learning]
    C --> G
    D --> G
    G --> H[TopologicalMap]
    E --> I[抽出IDから元点群IDへの対応表]
    I --> H
```

`uniform` では XYZ 以外の point field も点単位でコピーする。GNG 内部の `inpcl_ids` は publish 前に元点群のインデックスへ戻すため、semantic label の参照関係を維持する。`ds.transformed=false` の downsampling 出力は、抽出時には再パック後の点群を参照する。

## 10. リンク別可操作性の Viewer 表示

`robot_viewer_bridge_node` は、受信した `joint_states` を表示用 URDF と可操作性計算用の内部チェーンへ反映する。
マルチアームでは `waist_joint` のような共有関節が各腕の内部チェーンに存在するため、1 つの関節名を複数の内部 DOF 添字へ対応させる。

| 変数 | 型 | 用途 |
|---|---|---|
| `joint_name_to_active_index_` | `unordered_map<string, size_t>` | 受信 JointState の関節名から表示用値への対応 |
| `chain_joint_name_to_active_indices_` | `unordered_map<string, vector<size_t>>` | 関節名から全内部チェーン DOF 添字への対応 |
| `chain_joint_values` | `vector<double>` | 左右各腕と共有関節を含む内部 FK/Jacobian 入力 |
| `linkManipulabilities` | JSON array | リンク名、評価値、リンク相対楕円姿勢 |
| `manipCenter` | vec3 | リンク別データではリンク原点 `[0, 0, 0]` |
| `manipOrientation` | quaternion | 対象リンク相対の並進可操作性楕円姿勢 |
| `rotationalManipCenter` | vec3 | リンク別データではリンク原点 `[0, 0, 0]` |
| `rotationalManipOrientation` | quaternion | 対象リンク相対の回転可操作性楕円姿勢 |

```mermaid
flowchart TD
    A[joint_states] --> B[joint_name_to_active_index_]
    B --> C[chain_joint_name_to_active_indices_]
    C --> D[左右の共有関節を含む chain_joint_values]
    D --> E[MultiArm FK]
    D --> F[link Jacobian]
    E --> G[link transform]
    F --> H[manipulability ellipsoid]
    G --> I[world orientation から link-local orientation へ変換]
    H --> I
    I --> J[linkManipulabilities JSON]
    J --> K[Viewer の対象 URDF link]
    K --> L[link 原点へ楕円を描画]
```

Viewer のリンク別楕円は対象 URDF リンクの子として描画する。
そのため中心は FK 由来の別座標を再配置せず、選択リンクの原点と常に一致する。
手動関節表示ではリンクへの追従を維持するが、Jacobian と楕円スケールはブラウザ側で再計算せず、最後に受信した ROS 評価値を使用する。

## 11. Viewer の把持推定評価指標選択

`ToPo-FUZZY_Manipulation_v1.html` は、ROS topic または rosbag bundle JSON から受信した
`EvaluationMetrics` schema を把持推定ファジィルールエディタへ反映する。

| 項目 | 仕様 |
|---|---|
| 指標名 | `metric_id` を内部識別子、`label` を表示名として使用 |
| 入力特徴量名 | `evaluation.<metric_id>` |
| 初期状態 | schema の `enabled` を使用 |
| 対応型 | float、int、bool、float array |
| 非対応型 | string。名前は表示するがチェックボックスは無効 |
| float array | 有効要素の平均をスカラー値として使用 |
| 正規化 | 同じ指標の受信sample範囲を0〜1へ正規化 |
| 除外項目 | `selected`。ROS側の選択結果であり、HTML側の評価入力には含めない |
| sample対象 | 現在姿勢から候補経路を生成できた`feasible=true`の候補だけ |
| 未使用型配列 | `sample_int_values`、`sample_bool_values`、`sample_string_values`は空配列 |
| sample型 | `sample_metric_ids[i]`とschemaの`metric_ids/value_types`から解決。sampleごとの型配列は持たない |
| スカラー値 | `sample_metric_scalar_values[i]`が`sample_metric_ids[i]`のスカラー値 |
| 配列境界 | `sample_metric_array_offsets[i:i+2]`が評価指標slot `i`の配列範囲 |
| 配列値 | `sample_metric_array_values`へ全配列型評価値を連結 |
| 除外済み旧指標 | `combined_score`は構成要素と重複するため内部状態・候補metric・評価出力から削除 |

`feasible=false`の候補は`/evaluation_metrics`へsampleを生成せず、`feasible`自体も評価指標にしない。
診断用の`/grasp_candidate_metrics`には到達不能候補を含む全候補を維持する。

ライブROSでは「ROS2 ブリッジの起動」ボタンでrosbridgeへ接続し、`/evaluation_metrics`を購読する。
受信した指標定義は「3. 把持推定評価指標」へ追加し、有効な指標は既定のMembership Functionとともに
「4. 言語ラベル / Membership Function」の入力候補へ追加する。切断後にボタンを再度押した場合は
WebSocketへ再接続して購読を張り直す。rosbridge接続だけでは評価値は生成されないため、
`/evaluation_metrics`のPublisherが別途動作している必要がある。
rosbridgeコンテナはgng_cpuと同じ`ros_ws_volume`と`ros_ws_build_volume`を
`/ros2_ws/install`と`/ros2_ws/build`へmountし、
workspaceの`setup.bash`をsourceしてから起動する。これにより`gng_control_msgs`などの
workspace内カスタムメッセージをWebSocketへ変換できる。

チェックONでは既定の `Low`、`Medium`、`High` Membership Functionを生成し、
MF入力候補とルール条件候補へ追加する。チェックOFFでは特徴量の定義と編集値を保持したまま
`enabled=false` とし、MF入力候補から除外してファジィ推論でもその条件をスキップする。

```mermaid
flowchart TD
    A[ROS EvaluationMetrics] --> C[schema metric definitions]
    B[rosbag bundle JSON] --> C
    C --> D[把持推定評価指標チェックボックス]
    D -- ON --> E[evaluation.metric_id をMF入力へ追加]
    D -- OFF --> F[feature enabled=false]
    E --> G[ルール条件とMembership Function編集]
    F --> H[該当ルール条件を推論時にスキップ]
```

## 12. 変更時に更新すべき項目

1. launch 引数の追加・削除
2. topic 名の変更
3. service 名の変更
4. message field の追加・削除
5. no-motion / fallback / trial 分岐の追加
6. publish 周期や既定値の変更
7. 評価指標の意味づけや可視化ルールの変更
