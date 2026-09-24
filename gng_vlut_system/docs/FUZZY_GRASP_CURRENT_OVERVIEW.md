# 把持位置・姿勢推定の調査要約（2026-09-13）

## 1. 要約

ROSの候補生成・経路計画と、HTMLのファジィ把持評価は別系統。調査したlaunchには、HTMLの評価順位をROSの目標選択へ返す接続はない。

| 対象 | 処理・確認結果 |
| --- | --- |
| ROS候補生成 | 環境GNG・平面クラスタからTCP姿勢を幾何学的に生成 |
| ROS目標選択・計画 | 到達領域、計画GNG、衝突・経路を評価。HTMLの採点とは独立 |
| HTML把持評価 | 最大6種類の幾何候補を、11特徴量・12ルールで採点。最良候補にIK |
| HTML前処理 | 接続用2ルールと意味ラベル用4ルール。把持用とは別 |
| 拡張ルールR001〜R145 | [統合設計書](designs/fuzzy_grasp_design.md#rule-catalog)の将来案。実装済みのルール一覧ではない |

**処理の流れ**

- ROS：点群 → 環境GNG・平面 → TCP候補 → 到達領域判定 → 計画GNGの目標選択 → 経路評価・表示。
- HTML：点群・GNG → 法線・領域・意味ラベル → 幾何候補 → 特徴量・ファジィ採点 → 最良候補のIK・表示。

**候補と評価の意味**

- 環境GNGは物体形状、ロボットGNGは関節配置と到達・接続関係を表現。両者のノードIDは別。
- ROSの候補は `/grasp_pose_cands`（`GraspCandidateArray`）。`shape_score`は面内充填率で、ファジィ総合点ではない。
- `INSIDE`はTCP位置が到達ボクセル内という判定。姿勢のIK成立・無衝突・把持成功の確認は別。
- HTMLは既成候補の採点方式。位置・姿勢の連続最適化や、全候補のIK成立確認ではない。
- 11特徴量は開口、接触奥行き、持ち手らしさ、GNG支持量、safe・unknown・wall・default比率、高さ、アプローチ選好、衝突の少なさ。
- 既定推論はAND=min、出力別max集約、代表値の加重平均。非発火時は固定加重和へ復帰。`Reject`の発火だけでは除外されず、開口・衝突の除外判定は別。
- 最終順位は意味ラベル適合の通過を優先し、その後に減点後スコア。ROS評価指標の追加には、受信・候補ID対応・参照ルールの設定が必要。

## 2. 条件・検証

| 項目 | 確認範囲 |
| --- | --- |
| 調査時点 | 2026-09-13、`grasp_new4`、HEAD `566fb67f`と当時の作業ツリー |
| ROS | ソースと稼働中 `/top_grasp_surface_estimator` のパラメータ |
| HTML | ソース内の初期値。ブラウザーで編集・読込した現在値は対象外 |
| 動作指令 | 調査した候補用launchは制御権要求・関節指令を無効化 |
| 実行確認 | 推論関数を抽出しNode.jsで数値評価。全ノード同時稼働・実機把持は確認対象外 |

**数値再現した問題**

| 項目 | 結果 |
| --- | --- |
| 肩型台形の端点 | `Low(0)=0`、`High(1)=0`、`High(0.99)=1` |
| 法線類似度 | 内積0／0.5／1のいずれも1 |
| 良好側を1、wall等を0とした入力 | ルール非発火で固定加重和の100点へ復帰 |

この調査は指摘・再現までで、コード修正は未実施。100点をファジィルールによる高評価とは解釈できない。

**条件・制約**

- `/evaluation_metrics`のscopeは計画GNGの`goal_node_id`。HTML既定候補には対応IDがなく、自動接続済みではない。
- 関節余裕・衝突余裕・グリッパ幅・把持領域・エネルギー・時間の未計算値はNaN。HTMLの未取得値0への変換や候補集合内の正規化に注意。
- 当時のフレームは`world`、TCPは`L_tcp`、到達ボクセル幅0.05 m。frame passthroughは座標変換の正しさを保証しない。
- 後続の[候補Tmap化](releases/2026-09-14_candidate_topological_map.md)と[付属抽出の方式統一](releases/2026-09-14_fixed_reference_attachment.md)は個別記録を参照。当時の設定を現在の既定値として流用しない。

実装：[HTMLの特徴量・既定ルール・推論](../../ToPo-FUZZY_Manipulation_v1.html)、[ROS候補生成](../../grasping_system/src/top_grasp_surface_estimator_node.cpp)、[目標選択](../src/nodes/planning/topological_map_goal_selector_node.cpp)、[評価値の仕様](../../grasping_system/docs/fuzzy_evaluation_metrics.md)。改善案は[統合設計書](designs/fuzzy_grasp_design.md#open-design)。
