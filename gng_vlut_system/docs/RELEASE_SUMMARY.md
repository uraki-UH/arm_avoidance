# Release Summary - 2026-07-29 〜 2026-08-14

`docs/releases/` の個別リリースノート42件をまとめた版。個別ノートが「何を直したか」を残すのに対し、
この文書は **最終的に到達した仕様と挙動** を主題とする。途中の修正経緯は各節末尾の「経緯」へ畳む。

- 対象: `docs/releases/` 42ファイル（2026-07-29 / 08-03 / 08-07 / 08-12 / 08-14）
- 書き方の規約: [RELEASE_NOTE_TEMPLATE.md](RELEASE_NOTE_TEMPLATE.md) の「まとめ版の書き方」

---

## 到達点

エフェクティビティマップの **可視化経路** と、グリッパ形状を使った **把持候補の幾何照合** が、
どちらも設計から実配信まで通った。加えて Viewer が実機点群を安定表示できる負荷設定に落ち着いた。

| 領域 | 到達状態 |
|---|---|
| 可視化GNG | 3次元専用GNG + FK補間経路を事前計算し、150ノード規模で配信 |
| 軌道可視化 | 確定経路・候補経路を可視化ノード列へ線形時間変換して配信 |
| グリッパ体積 | `grip_V` / `grip_minV` / `grip_baseV` の3graphを実メッシュ形状で配信 |
| 把持候補照合 | 安定物体ボクセルと3graphの独立ゲートでTCP Pose候補を出力（増分更新対応） |
| 把持状態 | 外部 `GraspState` で把持物体VLUTを実行時合成 |
| Viewer | 100,000点 / 10 Hz でWebGLリセットなしの既定へ収束 |

---

## 作業種別ごとの最終状態

### 1. 可視化GNG

姿勢GNGの coord layer を3次元サンプルとして再学習した、可視化専用GNGを配信する。
可視化ノードは関節角を持たず、対応する元姿勢GNGノードID群を保持する。

- **所属**: 手先位置の純粋な最近傍割当。強制割当は行わない。各ノード中心は所属点 `weight_coords[layer]` の重心
- **エッジ**: 元GNGの angle-space edge を関節角補間しURDF FKで手先軌道へ変換したものを、事前計算して bin へ保存する。bridge実行時のFKと最近傍探索は行わない
- **状態集約**: best-wins（`safe > danger > colliding`）。「その場所に安全に手を届かせられるか」を答える基準で、プランナのゴール候補選定と一致する
- **候補目標**: 集約せず元座標のまま表示する（`TopologicalNode.is_goal`）
- **bin形式**: `VIZGNG2`、source signature schema **4**（元ノードID・3次元座標・関節角・angle-space edge を照合）
- **規模**: `ToPoDualArm10000` は 150ノード / 735エッジ、1連結成分、孤立0、空割当0

経緯:

- union-find による coord エッジ縮約の設計案は**未実装のまま置換**（`2026-08-03_graph_coarsening_visualization.md` は比較用に残置）
- edge の根拠を coord-space 縮約 → angle-space FK補間へ変更
- 可視化ノード数を 500 → 150 へ再学習（500ノード版は `vis_gng_500nodes_L0.bin` として保持）
- signature schema を 2 → 3 → 4 へ更新。旧binは不一致としてスキップされるため再生成が必要

### 2. 軌道の可視化配信

確定経路と候補経路を、元姿勢GNGノードIDから可視化ノード列へ bridge 内で変換して配信する。

- 変換は逆引き配列による線形時間。起動時 `O(n)`、単一経路変換 `O(L)`
- 先頭に現在EE poseの仮想ノード（**ID 65535**、`label=0`）を含み、最初のGNGノードへedgeを張る。FK結果がNaN/infなら追加しない
- 候補経路は各goal候補ごとに現在姿勢近傍の複数start候補を試し、そのgoalに対する最良pathを採る
- 既定色は静的GNGと区別するため safe を `#3b82f6`、edge を `#2563eb`（danger黄・collision赤・候補目標紫は不変）

経緯:

- 空path時に現在姿勢を持たない経路構築を使っていた分岐を統一
- Viewer側でedge終端から候補目標を推測する処理を廃止
- 起動順によって候補軌道が空のままになる問題を、静的graph完成後の再配信で解消

### 3. グリッパ体積graph

グリッパー形状を既存の `ais_gng_msgs/msg/TopologicalMap` として transient-local publish する。専用messageは追加しない。
ロボット固有launchは持たず、`gripper_volume_graph.launch.py grippers_file:=...` に構成を渡す。

| graph | 内容 | ToPoDualArm実測 |
|---|---|---|
| `grip_V` | 最大開口時の実メッシュに挟まれる自由空間 | 248 nodes / 585 edges |
| `grip_minV` | 全閉時の左右指に挟まれた内部の非占有空間 | 130 nodes / 279 edges |
| `grip_baseV` | 基部STLの占有形状（禁止領域） | 1271 nodes / 3156 edges |

- 探索は外接box内で閉方向の正負rayが対応する指STLへ交差する格子だけを採用し、指・基部STLの占有格子を除外する
- 実形状ではない外接boxを表示しないよう、`grip_V` と `grip_minV` の `clusters` は空
- graphはTCP frameに固定し、ロボット姿勢との合成は既存TFに任せる
- node/edge index は `uint16` のため、65,535 nodes に達する `resolution` は生成を拒否する

経緯:

- topic名を `gripper_volume` → `grip_V`、`grip_V_undersize` → `grip_minV` へ短縮（`V` は大文字、旧名の互換発見は持たない）
- `grip_V` を固定box 504 nodes / 1321 edges から実形状 248 nodes / 585 edges へ変更
- ロボット名固定の `ToPoDualArm_gripper_volume_graph.launch.py` を廃止し、汎用launchへ集約

### 4. 把持候補の幾何照合

`SAFE_TERRAIN` / `HUMAN` / `CAR` を除いた時間的に安定なGNGボクセルを物体候補とし、
グリッパ体積3graphの独立ゲートで把持候補TCP Poseを照合する。

- **ゲート**: `grip_V`＝必須占有、`grip_minV`＝最小体積外の支持、`grip_baseV`＝禁止領域衝突
- **物体候補の主topic**: 直接観測・GNG edge補間・3-edge cycleからの三角形面の**和集合**
- **孤立セル**: 26近傍を持たない直接観測セルは主topicから除外し、`<output_topic>/isolated` だけへ出す（Viewerでは赤で初期表示）
- **履歴**: 各セルが直近100同期更新の非除外label出現回数と点群input更新回数を独立に保持する。連続観測は要求しない
- **更新方式**: `voxel_msgs/Voxel` の全量snapshotで候補を再評価する
- **既定の処理上限**: 500 ms周期、最大500アンカー、12 yaw姿勢、上位50候補
- **セルサイズ**: launch・ノード・共通グリッド定義とも `0.01 m` に統一

経緯:

- 候補labelを限定する `included_labels` を削除し、`excluded_labels` による除外のみへ整理
- 判定器 `GraspPoseOccupancyEvaluator` を先に独立APIとして追加し、後からROS topic契約へ接続

### 5. 把持状態のランタイム反映

外部 `gng_control_msgs/msg/GraspState` を購読し、把持物体の占有形状を実行時のエフェクティビティマップへ合成する。

- 把持開始時に全有効GNGノードの物体占有ボクセルを計算し、把持物体VLUTを通常VLUTへ加算する
- 把持・解放の受信時は占有topicの次回更新を待たず即座に再評価する
- 同一の把持定義を再受信した場合はVLUTを再構築せず冪等に適用確認を返す
- 解放の `object_id` がactive objectと異なる場合は誤解放を防ぐため無視し、空なら現在activeな物体を解放する
- 適用済み状態は `grasp_state_applied`（reliable / transient-local）へ返す

責務分界: `grasping_system` は物体形状・相対姿勢・把持ライフサイクル・把持物体VLUTを持ち、**把持成否そのものは判定しない**。

### 6. Viewer の描画負荷と安定性

AMD GPUでのWebGLコンテキスト喪失を再発させない既定値に収束した。

| parameter | 既定値 |
|---|---|
| `pointcloud_max_points` | `100000`（`0`で無制限） |
| `pointcloud_max_hz` | `10.0`（`0`で無制限） |
| `websocket_max_backpressure_bytes` | `8388608` |

- PointCloud2 は SensorDataQoS depth 1 で購読し、表示待ちがある場合は古いフレームを保持しない
- 上限超過時は先頭切り出しではなく、点群全域から決定論的に均等抽出する
- WebGLは high-performance GPU を要求し、MSAAは無効
- RGB点群は vertex shader で sRGB → linear へ変換し、ACES tone mapping を無効化する
- 同一URLのメッシュ読み込み結果を共有し、URDF本文は description のトップレベルにだけ格納する（候補8件で転送量約94.4%減）
- Topics のチェックをオフにすると対応する Scene Layer も削除する
- clipping plane は Canvas / WebGL context を再生成せず既存 renderer へ同期する

経緯:

- 既定点数を 100000 → 300000 へ引き上げた後、AMD GPU の `ring gfx` timeout と GPU reset を `journalctl -k` で確認し **100000 へ戻した**。300000点以上が必要な場合は launch引数で明示指定する

### 7. 評価指標（EvaluationMetrics）

`/evaluation_metrics` は評価値専用とし、graph構造は格納しない。schema revision **5**。

- `selected` と `feasible` は publish しない。sample は `feasible=true` 候補だけ
- 基礎データから派生できる指標を削除（`manipulability_condition_number`、`min_singular_value`、`manipulability_singular_values`、`joint_limit_score`、`is_colliding` / `is_danger`、`collision_count` / `danger_count`）。関節限界余裕は `joint_limit_margin_min` / `joint_limit_margin_mean` を使う
- field名を `sample_metric_scalar_values` / `sample_metric_array_offsets` / `sample_metric_array_values` へ統一
- 購読は RELIABLE + TRANSIENT_LOCAL。Publisher より後に起動しても最新schemaを受信できる
- HTML側は受信schemaをそのまま動的に扱い、個別metric名を特別扱いしない。グリッパ体積graphは `evaluationMetrics.structuredInputs` として別枠で保持する
- 診断用 `/grasp_candidate_metrics`（`GraspCandidateMetricArray`）は互換性のため既存フィールドを維持する

経緯:

- `min_singular_value * joint_limit_score` だけを表していた旧統合スコアを削除（GNG binary **version 9**。version 4〜8 は旧fieldを読み捨てて継続利用可）

### 8. GNG学習とVLUT

- `ToPoDualArm10000`: 有効元ノード **10,801**、`gng.bin` 5,268,947 bytes、`vlut.bin` 57,516,224 bytes
- 学習制御を launch引数化（`max_node_num` / `max_iterations` / `refine_iterations`）。省略時はYAMLの値を使う
- VLUTは profile の EEF から親をたどり、同じ親から分岐する終端リンクとその子孫（グリッパ指）を自動的に対象へ含める
- 入力点数が上限を超える場合の抽出に `input.sampling_mode`（`head` / `uniform`）を追加。`graspnet.yaml` は `uniform`
- AiS-GNG の共分散処理は既定 **無効**（`node.covariance_enabled=false`）。`performance.log_interval_ms`（既定 `5000` ms）で周期と工程別処理時間をINFOログへ出す

### 9. 実機接続とリンク別可操作性

- Dynamixel ID **1-8=右腕 / 11-18=左腕 / 21=首pan / 22=首tilt**。角度変換は `(position_deg + joint_offsets_deg) * deg_to_rad * joint_scales`
- Viewer起動時は `direct_joint_tracking:=true` が既定で、mux目標角を速度補間せず直接反映する（`false` で従来の `max_joint_velocity` 補間へ戻せる）
- リンク別可操作性楕円は対象URDFリンク直下へ描画し、中心は選択リンクの原点。回転楕円は専用の `rotationalManipCenter` を使う
- 候補ロボットプレビューの `opacity` は ROS 側で持たず、Viewer側の既定値に従う

---

## 日付ブロック

| 日付 | 件数 | 主題 |
|---|---|---|
| 2026-07-29 | 8 | 文書運用の開始、評価指標の整理（combined score削除・候補filter）、可操作性のリンク相対化、RGB色空間、GNG均等抽出 |
| 2026-08-03 | 14 | 可視化GNGの導入と方式確定（3次元再学習 → FK補間経路）、軌道の可視化配信、10,000ノード学習、VLUTへの指リンク収録、Viewer初期表示の重複削減 |
| 2026-08-07 | 1 | 外部 `GraspState` による把持物体VLUTの実行時合成 |
| 2026-08-12 | 12 | グリッパ体積graph 3種の確立とtopic改名、HTML側の体積graph入力、可視化GNG 150ノード化、AiS-GNG共分散既定オフ |
| 2026-08-14 | 7 | 安定物体ボクセルとの把持候補照合（増分更新含む）、実機Dynamixel対応、Viewer点群負荷の確定 |

---

## 最終形リファレンス

### Topic

| topic | message | 備考 |
|---|---|---|
| `/ToPoDualArm/topological_map_vis_L0` | `TopologicalMap` | 静的可視化GNG |
| `/ToPoDualArm/plan_topological_map` / `_vis_L0` | `TopologicalMap` | 確定経路と可視化変換後 |
| `/ToPoDualArm/cand_topological_map` / `_vis_L0` | `TopologicalMap` | 候補経路と可視化変換後 |
| `/ToPoDualArm/{L,R}_grip_V_topological_map` | `TopologicalMap` | 最大把持領域 |
| `/ToPoDualArm/{L,R}_grip_minV_topological_map` | `TopologicalMap` | 最小把持領域 |
| `/ToPoDualArm/{L,R}_grip_baseV_topological_map` | `TopologicalMap` | 基部禁止領域 |
| `/ToPoDualArm/grasp_state` / `grasp_state_applied` | `GraspState` | 把持指示と適用済み状態 |
| `<output_topic>/isolated` | `Voxel` | 把持候補から除外した孤立セル |
| `/evaluation_metrics` | `EvaluationMetrics` | schema revision 5 |
| matcher出力 | `PoseArray` + `String` | 候補TCP PoseとJSON summary |

### バージョン・スキーマ

| 対象 | 最終値 |
|---|---|
| GNG binary version | 9 |
| 可視化GNG bin | `VIZGNG2` / signature schema 4 |
| EvaluationMetrics schema revision | 5 |
| 物体候補ボクセルのセルサイズ | `0.01 m` |

### 廃止

`combined_score` / `candidate_robot_preview_opacity` / `included_labels` /
`sample_metric_value_types` / `gripper_volume` 系topic名 / `grip_V_undersize` /
`ToPoDualArm_gripper_volume_graph.launch.py` / `/evaluation_metrics` の `selected`・`feasible`

---

## 未実装・既知の制約

- **把持成否判定ノードが未実装**。センサ・グリッパ開度・電流・力覚・接触からの判定は外部または手動publishに依存する。物体脱落検出、把持失敗判定、再把持、複数物体の同時active化も未実装
- 把持物体の形状入力は**直方体のみ**。mesh・点群・任意voxel形状のtopic入力は未実装
- `eef_link` は把持主体の識別に保持するが、**左右腕GNGの自動選択は未実装**。現行 `ToPoDualArm10000` は左腕GNG（`L_tcp`）
- 把持候補照合は**固定姿勢集合による直接照合**。局所法線からの3次元姿勢生成と物体ID分離は未実装
- edge補間は観測free空間との交差判定を持たない。`edge_max_length` は実データのGNG edge分布に合わせた調整が必要
- `TopologicalMap` は path ID を持たないgraph表現のため、候補ごとのpolyline分離や厳密な再生順序が必要なら別messageが要る
- GNG対象外のグリッパ開閉関節は既定値0で固定。開度ごとの占有形状には別仕様が必要
- bin は x86_64 little-endian の native binary 形式を前提とする
- `grip_baseV` の 5 mm安全余裕は初期値。実機の位置誤差と環境点群のvoxel寸法に合わせた調整が必要
- Frontend全体の lint は既存の `SharedControls.tsx` 2件・`ellipsoid.tsx` 1件で失敗する（本まとめ期間の変更とは無関係）
