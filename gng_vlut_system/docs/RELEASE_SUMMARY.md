# 開発結果の要約（2026-07-29〜2026-08-14）

## 1. 要約

対象期間のリリースノート42件を整理した記録。当時の到達点であり、現在のトピック名・設定の一覧ではない。

| 対象 | 到達点 |
| --- | --- |
| 可視化GNG | 3次元専用GNGとFK補間経路を事前計算。150ノード・735エッジ、1連結成分 |
| 軌道表示 | 確定・候補経路を可視化ノード列へ変換。現在EE位置を仮想ノードID 65535として接続 |
| グリッパ体積 | 最大把持領域`grip_V`、最小領域`grip_minV`、基部禁止領域`grip_baseV`を実メッシュから配信 |
| 把持候補 | 安定物体ボクセルと3領域を照合しTCP姿勢候補を出力。増分更新へ対応 |
| 把持状態 | 外部`GraspState`で把持物体VLUTを合成し、`grasp_state_applied`で適用状態を返却 |
| Viewer | 点群の均等抽出・送信抑制・WebGL安定化。URDF重複転送を候補8件で約94.4%削減 |
| 評価指標 | `EvaluationMetrics`を評価値専用へ整理。schemaに基づくHTML入力と遅れて接続した購読者への配信 |
| GNG・VLUT | 有効元ノード10,801のToPoDualArm10000を生成。指リンクをVLUTへ収録 |
| 実機接続 | Dynamixelの関節対応と直接追従を整備。リンク別可操作性の描画座標を修正 |

主な整理は、旧統合スコア・旧グリッパトピック名・ROS側の候補opacity設定の廃止。可視化のcoord辺縮約案は、angle-spaceのFK補間方式へ置換。

## 2. 条件・検証

**当時の設定・形式**

| 項目 | 値・内容 |
| --- | --- |
| Viewer点群 | 100,000点、10 Hz、送信待ち上限8,388,608 byte |
| 把持候補照合 | セル0.01 m、500 ms周期、最大500アンカー・12 yaw・上位50候補 |
| グリッパgraph | `grip_V` 248ノード／585辺、`grip_minV` 130／279、`grip_baseV` 1,271／3,156 |
| GNG保存形式 | binary version 9。version 4〜8は旧fieldを読み捨てて読込 |
| 可視化保存形式 | `VIZGNG2`、source signature schema 4。旧signatureは再生成が必要 |
| 評価schema | revision 5、RELIABLE・TRANSIENT_LOCAL |
| ロボットGNG | 左腕`L_tcp`。GNG 5,268,947 byte、VLUT 57,516,224 byte |
| Dynamixel ID | 右腕1〜8、左腕11〜18、首pan 21・tilt 22 |

**確認結果・制約**

- 可視化GNGは孤立・空割当とも0。可視化配信・候補照合・把持状態反映を個別ノートで検証。
- AMD GPUで30万点設定時にtimeout・resetを確認し、既定を10万点へ復帰。任意の機器・負荷での安定性保証ではない。
- 把持成否・脱落・再把持・複数active物体の判定は未実装。物体形状入力は直方体、左右腕GNGの自動選択も未実装。
- 把持姿勢は固定集合の照合。局所法線からの姿勢生成・物体ID分離・補間辺と観測free空間の交差判定は未実装。
- 候補経路graphにはpath IDがなく、候補別の厳密な再生順序を保持しない。
- 指の開閉関節は既定値0、基部の5 mm余裕は調整が必要。binはx86_64 little-endianのnative形式。
- Frontend全体のlintは当時の既存3件で失敗。実装の個別検証と区別。

詳細：[可視化GNG](releases/2026-08-12_topodualarm10000_visualization_gng_150_nodes.md)、[把持候補照合](releases/2026-08-14_stable_object_grasp_voxel_matching.md)、[把持状態](releases/2026-08-07_external_grasp_state_runtime.md)、[Viewer設定](releases/2026-08-14_topofuzzy_pointcloud_safe_default.md)。現行仕様は[TECHNICAL_SPEC.md](TECHNICAL_SPEC.md)。
