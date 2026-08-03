# 軌道・把持候補選定におけるファジィ評価パラメータ仕様

ロボットの軌道生成（Trajectory Generation）および把持位置（Grasp Pose）の候補を絞り込むためのファジィ評価に使用可能な情報の一覧。

---

## 1. 概要
軌道生成および把持候補の評価指標は、ROS 2メッセージ `gng_control_msgs/msg/GraspCandidateMetric` および `gng_control_msgs/msg/GraspCandidateMetricArray` を通じて収集・配信されています。
これらのトピック（例: `/ToPoDualArm/grasp_candidate_metrics`）を `gng_bundle_exporter` パッケージの `gng-bundle-export` ツールを使用してJSON化することが可能です。

---

## 2. ファジィ評価に使用可能な情報のリスト

### 2.1 候補の基本情報と妥当性
| 変数名 | 型 | 値の想定範囲 | 単位 | 説明 |
| :--- | :--- | :--- | :--- | :--- |
| `candidate_id` | `int32` | $0 \le \text{id}$ | なし | 評価対象となる候補のシリアルインデックス。 |
| `goal_node_id` | `int32` | GNGノードID | なし | 目標姿勢となるGNGグラフ上のノードID。 |
| `start_node_id` | `int32` | GNGノードID | なし | 探索開始点となるGNGグラフ上のノードID。 |
| `selected` | `bool` | `true` / `false` | なし | この候補が現在の最適パスのゴールとして選択されたか。 |
| `feasible` | `bool` | `true` / `false` | なし | 開始ノードからこの目標ノードへの衝突回避経路が存在するか。 |

### 2.2 姿勢・関節状態
| 変数名 | 型 | 値の想定範囲 | 単位 | 説明 |
| :--- | :--- | :--- | :--- | :--- |
| `end_effector_pose` | `Pose` | 直交座標空間内 | `m` (位置) | 候補ノードにおけるエンドエフェクタの3D姿勢（位置 [x,y,z]、姿勢クォータニオン [x,y,z,w]）。 |
| `final_joint_state` | `JointState` | 関節可動範囲内 | `rad` | 候補ノード（目標姿勢）における全関節の目標角度。 |

### 2.3 マニピュラビリティ（操作性）指標
| 変数名 | 型 | 値の想定範囲 | 単位 | 説明 |
| :--- | :--- | :--- | :--- | :--- |
| `position_manipulability` | `float32` | $[0, \infty)$ | なし | 並進方向のマニピュラビリティ（ヤコビアンの操作楕円体体積比例値）。高いほど特異姿勢から遠く、手先位置の制御自由度が高い。 |
| `rotation_manipulability` | `float32` | $[0, \infty)$ | なし | 回転方向のマニピュラビリティ。高いほど手先の向きを自由に変更可能。 |

`manipulability_condition_number`、`min_singular_value`、`manipulability_singular_values` は
可操作性楕円体などの基礎データから派生できるため、`/evaluation_metrics` の固定schemaには含めない。

### 2.4 安全マージン
| 変数名 | 型 | 値の想定範囲 | 単位 | 説明 |
| :--- | :--- | :--- | :--- | :--- |
| `joint_limit_margin_min` | `float32` | $[0, 1]$ | なし | 候補姿勢における各関節の「限界までの余裕度」の最小値。0で限界到達、1で中心。 |
| `joint_limit_margin_mean` | `float32` | $[0, 1]$ | なし | 各関節の余裕度の平均値。 |
| `self_collision_margin` | `float32` | $[0, \infty)$ | `m` | ロボット本体（リンク間）の自己干渉マージン。※現状の実装では NaN。 |
| `environment_collision_margin` | `float32` | $[0, \infty)$ | `m` | 周囲の障害物環境との干渉マージン。※現状の実装では NaN。 |

### 2.5 把持特有の指標
| 変数名 | 型 | 値の想定範囲 | 単位 | 説明 |
| :--- | :--- | :--- | :--- | :--- |
| `gripper_width` | `float32` | $[0, \text{Max}]$ | `m` | 把持に必要なグリッパーの開口幅。※現状の実装では NaN。 |
| `grasp_region_score` | `float32` | $[0, 1]$ | なし | 把持点の安定性や面接触品質のスコア。※現状の実装では NaN。 |

### 2.6 経路品質（Path Quality）
| 変数名 | 型 | 値の想定範囲 | 単位 | 説明 |
| :--- | :--- | :--- | :--- | :--- |
| `path_node_ids` | `int32[]` | GNGノードID群 | なし | 開始点から目標点までの経路を構成するノードIDの配列。 |
| `path_position_manipulability` | `float32[]` | 各要素 $[0, \infty)$ | なし | 経路上各ノードでの並進マニピュラビリティの推移。 |
| `path_rotation_manipulability` | `float32[]` | 各要素 $[0, \infty)$ | なし | 経路上各ノードでの回転マニピュラビリティの推移。 |
| `estimated_energy` | `float32` | $[0, \infty)$ | `rad²` | 関節角度の変化量の二乗和（$\sum \|\Delta q\|^2$）。値が小さいほど滑らかでエネルギー消費が少ない。 |
| `estimated_duration` | `float32` | $[0, \infty)$ | `s` (秒) | 経路移動にかかる推定所要時間（$\sum \max(\|\Delta q\|) / \dot{q}_{\max}$）。 |

### 2.7 ノード付帯特徴量（拡張用）
`metric_names` および `metric_values` 配列にペアとして格納される、個別ノードが保持する統計評価値です。
- `dynamic_manipulability` (動的マニピュラビリティ)

---

## 3. ROS 2 BagからのJSON化手順

`gng_bundle_exporter` パッケージを使用して、ROS 2 bagからメトリクスデータをJSONとしてエクスポートすることができます。

### 3.1 設定ファイルの準備
`gng_bundle_exporter/config/` 配下に以下のようなYAML構成ファイルを指定します。
例として `gng_bundle_exporter/config/export_topics.example.yaml` が用意されています。

```yaml
topics:
  # 軌道生成・把持候補のメトリクス情報を抽出
  - alias: metrics
    topic: /ToPoDualArm/grasp_candidate_metrics  # bag内の該当トピック名
    kind: generic
    role: metrics

  # 必要に応じて候補軌道のグラフ構造も抽出可能
  - alias: candidate_graph
    topic: /ToPoDualArm/candidate_topological_map
    kind: topological_map
    role: candidate_graph
```

### 3.2 実行コマンド
パッケージディレクトリにてツールを実行します。

```bash
ros2 run gng_bundle_exporter gng-bundle-export export \
  --bag /path/to/your_rosbag_dir \
  --config /path/to/config.yaml \
  --output /path/to/output.json \
  --pretty
```

これにより、各候補の `candidate_id` に紐づく上記すべてのファジィ評価用変数がタイムスタンプ付きで整理されたJSONファイルが得られます。
