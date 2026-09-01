# gng_vlut_system Task List

この文書は、実装方針が決まり「これから順番に実行する」と確定したタスクを並べる作業台帳です。
まだ方針が固まっていないものは [TASK_CANDIDATES.md](./TASK_CANDIDATES.md) に置き、ここには実行順が決まった具体タスクだけを積みます。

## 運用ルール

1. このファイルは手動編集が前提。ユーザーが直接タスクの追加・削除・並び替えを行う。
2. ディスク上の現在の内容を常に正とし、ユーザーが削除・変更した項目を過去のやり取りから復元しない([[preserve-user-edits]]に従う)。
3. 順番を変えたいときは、この文書を手で編集して並べ替える。
4. 完了したタスクはこのファイルから削除する。実装済みで安定した挙動は `TECHNICAL_SPEC.md` へ、変更点は `releases/` へ記録してから消す。
5. 進めているうちに方針が再び未確定になったタスクは、削除せず `TASK_CANDIDATES.md` へ差し戻す。
6. `TASK_CANDIDATES.md` の候補が実行確定したら、ここへ番号付きで移す。

## 前提

ゴミ処理場での廃棄物の移動・破砕が応用タスクの一つ。フェーズ1(POC)の対象品目はまず**コンクリートガラのみ**とし、廃プラ・木質廃材はStep3で追加する。大物の破砕処理はPOCスコープ外(別見積り)。

| 区分 | 時期 | 暫定目標 |
|---|---|---|
| POC | 〜8月(最大9月中旬) | 認識→把持→リリースが**一通り通る** |
| Step2 | 9〜11月 | コンクリートガラ単体で**安定して成功する** |
| Step3 | 12〜2月 | 他産廃へ**広がる**、実機HW整合 |

## タスク

### POC (〜8月 / 最大9月中旬)

1. **実機ラインの再有効化**
   `ToPoDualArm/topoarm_hardware/COLCON_IGNORE` により2026-06-19からビルド対象外。除外を解除し、実機との疎通を確認する。
   *暫定目標: 実機に対して関節指令が通り、safety_monitorが機能する*

2. **点群からの把持手先位置姿勢候補群の取得を精緻化**
   上面把持限定経路として、各平面クラスタのXY-OBBと、候補重心から隣接平面までの最小距離で出っ張りを候補化する初期実装まで完了。平面クラスタの空間共分散も出力済み。bagデータ・実センサで共分散から求める疑似クリアランス、候補安定性、後段の衝突・到達性を検証する。
   *暫定目標: 実センサ点群から妥当な候補群が安定して出る*

2.1. **最優先: 非平面成分を用いるオクルージョン下の物体テンプレート照合**
   `basket`の単一視点・床面なし・遮蔽あり環境で、平面クラスタだけでは物体固有の肯定証拠が不足した。既存の`nonplane_component_node`を、持ち手・フック・突起のような非平面構造を使う照合器入力へ拡張する。位置レジストレーションは導入せず、yaw探索と、平面対応に対する局所グラフ構造・法線分布・平面アンカー関係だけを評価する。

   **現状と判断根拠**

   1. `ais_gng`は`/topological_map`と`/plane_clusters`から、平面未所属nodeのGNG連結成分を`/nonplane_components`へ出力済み。
   2. 同じ`basket`テンプレートでは、持ち手候補が16node、25内部edge、平面クラスタ2への15アンカーedgeを持つ成分として得られた。
   3. 環境20フレームでは、最大成分が9〜27node、8〜28内部edge、平面アンカーedgeが0〜2本と変動した。成分数、node数、edge数、アンカー数を単フレームの必須条件または反証条件にしてはならない。
   4. 形状比と法線二次モーメント固有値は部分観測の弱い加点特徴として利用可能。ただし環境最大成分の形状比は0.110〜0.713と変動するため、狭い一致範囲を要求しない。
   5. 未観測の非平面特徴、アンカーedgeの欠落、テンプレートより小さい成分は遮蔽として中立扱い。高信頼度の観測成分がテンプレートで説明不能な場合だけ、後段の反証候補とする。

   **最初の実装到達点**

   テンプレート平面との対応が得られたyaw候補について、対応平面に接続する環境非平面成分をテンプレート非平面成分へ部分対応させる。法線分布と局所グラフ形状による加点を複数フレームで蓄積し、平面だけでは確定しない単一視点`basket`仮説を補強する。非平面成分が見えないフレームで仮説を破棄しない。

   **入力と出力の契約**

   1. `ais_gng_msgs`へ`NonplaneComponentArray`、`NonplaneComponent`、`NonplanePlaneAnchor`を追加。
   2. `NonplaneComponentArray`には`header`、`frame_number`、元`TopologicalMap`のnode添字を使う`components`を格納。
   3. `NonplaneComponent`には`id`、`node_indices`、`internal_edge_num`、`plane_anchors`を格納。表示用のコンパクト`TopologicalMap`は既存`/nonplane_components`として維持。
   4. `NonplanePlaneAnchor`には`component_id`、`source_node_index`、`plane_node_index`、`plane_cluster_id`を格納。nodeの`id`ではなく元map配列の添字を使い、ID再採番への依存を除外。
   5. `nonplane_component_node`は新規の`/nonplane_component_features`を同一フレーム番号でpublish。`/nonplane_components/markers`は可視化専用のままとする。
   6. `object_template_matcher_node`は`nonplane_component_features_topic`を購読し、`/topological_map`、`/plane_clusters`、特徴配列のframe番号が一致したときだけ評価。
   7. テンプレート側の非平面成分は、初期段階では保存形式を増やさず、`gng.edges`と`gng.plane_clusters[].idx`から読込時に導出。安定後にだけ`gng.nonplane_components`の保存を検討。

   **照合方式**

   1. 既存の平面クラスタ対応でテンプレート平面と環境平面を一対一対応し、既存yaw候補ごとに評価。
   2. テンプレート成分のアンカー先平面が対応済みの場合だけ、対応環境平面へ接続する環境成分を比較候補化。
   3. 成分ごとに、内部edge数、局所次数分布、位置共分散固有値比、法線二次モーメント固有値、`rho`統計、勝者誤差楕円の固有値比を算出。
   4. node数、内部edge数、絶対的な広がり、AABBは低重みの補助評価だけに使う。環境側が小さい場合は未観測扱いとし、減点しない。
   5. 平面アンカーedgeの存在は加点する。アンカーedgeが0本でも減点しない。アンカー群化、弧長、弦長、平面内の相対位置は初期到達点の後に追加。
   6. 各対応のscoreを`nonplane_component_score`として算出し、成分score和の飽和関数を`nonplane_evidence_score`とする。成分個数の一致率は診断値だけにする。
   7. 既存scoreへは`nonplane_weight`で加算。非平面証拠だけで`confirmed`にせず、対応平面または局所GNG edge構造の根拠を要求。
   8. `nonplane_component_score`、`nonplane_evidence_score`、`has_nonplane_evidence`、`nonplane_correspondences`を候補JSONへ出力し、調整と検証を可能化。

   **時間蓄積**

   1. テンプレートIDとyaw候補ごとに、直近`nonplane_evidence_window_sec`の加点を蓄積。
   2. 同一yaw近傍の候補を円周角度として統合し、平面1枚の対称性による40度と50度の往復を単一候補として扱う。
   3. `min_nonplane_evidence_frame_num`と`min_nonplane_evidence_score_th`を満たしたときだけ、非平面証拠ありと判定。
   4. 非平面特徴が欠落したフレームは蓄積値を即時消去せず、時間窓の自然失効まで中立扱い。

   **初期パラメータ**

   1. `enable_nonplane_component_evaluation: true`
   2. `nonplane_component_features_topic: /nonplane_component_features`
   3. `min_nonplane_component_nodes: 2`
   4. `nonplane_weight: 0.20`
   5. `nonplane_evidence_score_scale: 1.50`
   6. `nonplane_evidence_window_sec: 1.00`
   7. `min_nonplane_evidence_frame_num: 5`
   8. `min_nonplane_evidence_score_th: 0.55`
   9. 形状比、法線二次モーメント、`rho`、誤差楕円の各ファジー幅は`basket`の完全形、単一視点、背景平面追加の3条件で実測後に設定。

   **実装順**

   1. 新規メッセージ定義と`ais_gng_msgs`のbuild。
   2. `nonplane_component_node`から構造化アンカー付き特徴topicのpublish。
   3. 平面nodeをまたいで成分を併合しないこと、複数平面へのアンカー保持、元node添字の保存を確認する単体test追加。
   4. `object_template_matcher_node`でテンプレート非平面成分を読込時導出。
   5. yaw・平面対応で絞り込んだ成分候補と、加点専用の`nonplane_component_score`を実装。
   6. 候補JSONとvalidatorへ時間蓄積の証拠値を接続。
   7. ViewerまたはMarkerで、成分対応、アンカーedge、採用yaw帯、各ファジーscoreを可視化。

   **受け入れ条件と実行コマンド**

   1. `docker exec gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select ais_gng_msgs ais_gng gng_vlut_system --event-handlers console_direct+'` が成功。
   2. `docker exec gng_cpu_container bash -lc 'cd /ros2_ws && ./build/ais_gng/test_nonplane_component_extractor --gtest_color=no'` が成功。
   3. `ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=<入力yaml>` 実行中に、`/topological_map`、`/plane_clusters`、`/nonplane_component_features`が同じ`frame_number`で出力。
   4. `ros2 topic echo --once /nonplane_component_features`で、成分の元node添字と平面アンカーedgeを確認。
   5. `basket`完全形、単一視点遮蔽あり床面なし、背景平面追加の各条件で、候補JSONの非平面score・対応・yaw帯を記録。
   6. 単一視点遮蔽あり条件では、非平面特徴が欠落した1フレームだけで候補が`is_falsified: true`にならない。
   7. 背景平面追加条件では、背景平面だけで`confirmed`にならず、非平面証拠または複数平面証拠がある候補だけを時間確認へ進める。
   8. 既存の平面クラスタ照合を`enable_nonplane_component_evaluation: false`で従来どおり実行できる。

   *暫定目標: 遮蔽を含む単一視点環境で、平面クラスタを仮説起点、非平面構造を物体固有の時間蓄積型肯定証拠として使い、テンプレート候補とyaw帯を安定して出せる*

3. **軌道生成と目標姿勢の精緻化**
   既存手法の範囲でよい。新規手法の導入はしない。
   *暫定目標: 障害物回避を含む軌道が候補姿勢に対して生成できる*

4. **タスクシーケンス制御の実装**
   認識→把持候補選定→軌道生成→把持→搬送→リリースを、各launchの手動起動なしに一巡させる。
   *暫定目標: 単発の把持サイクルが自動で通る*

5. **実機での簡易検証**
   小型ハードで軽量・単純形状物を対象とする。成功率は問わない。
   *暫定目標: 実機で把持〜リリースが数回成功する*

### デモ可視化 (8月 / 先方提示用)

既存の番号を動かさないため英字で採番する。目的は**先方に見せたときの分かりやすさとインパクト**であり、デバッグ用途とは基準が異なる。

可視化専用3次元GNGによる集約、元angle-space edgeのFK補間、layer別配信、軌道変換、Viewer描画、双腕のcoord layer分離確認までは完了済み。現行仕様は[TECHNICAL_SPEC.md](./TECHNICAL_SPEC.md#13-可視化専用3次元gng)を参照する。

旧案のunion-find縮約とマージ列は可視化専用3次元GNGへ置き換えたため実装しない。マージ列を前提とした粒度スライダも現行タスクから除外する。

E. **(余力があれば) 障害物での塗り替え**
   障害物を置くとノードが赤くなり、**エッジが切れる**。「そこを通れなくなった」がグラフの切断として見える。オキュパンシーグリッドでは出せない絵であり、位相地図を使う理由がそのまま伝わる。

### Step2 (9〜11月 / コンクリートガラ)

6. **コンクリートガラを想定した物理シミュレーション環境の構築**
   不定形・重量物が山積みになった状態での検証環境。方式は[TASK_CANDIDATES.md](./TASK_CANDIDATES.md)参照。
   *暫定目標: 複数配置パターンを再現できる*

7. **把持成否の判定とリトライ**
   掴み損ねた場合に次候補へ移行する。
   *暫定目標: 失敗を検出して自動で復帰する*

8. **成功率の計測と評価の仕組み**
   *暫定目標: 試行回数・成功率・サイクルタイムが記録される*

   あわせて**段階別の処理時間**を計測できるようにする。論文で先行研究と比較する際、工程を分けていないと同一条件の比較にならないため。計測する段階:
   - ① 障害物→GNGノードの属性更新(VLUT引き + フラグ更新)
   - ② グラフ探索(経路計画)
   - ③ 平滑化
   - ④ 1周期の合計
   - あわせてノード数・有効ノード数を記録する(比較条件として必須)

   比較対象の実測値: Steffen et al. 2022 (arXiv:2207.03959) は GNG + Dijkstra で②が13ms、②+③で37ms(10,000ニューロン、6自由度 UR3e)。①は単独で報告されていない。同論文はAbstractで0.02秒と記載しているがTable Iの値と一致しないため、引用にはTable Iを使う。

9. **連続動作と安全な停止**
   *暫定目標: N個連続処理でき、回避不能時に安全停止する*

10. **積み残しの設計課題の解決**
    `TASK_CANDIDATES.md` の評価指標構成、候補姿勢とGNGノードの多対多化、関節wraparound対応。品目を増やす前に片付ける。

11. **ボクセル生成元の一本化**
    GNG側のボクセル化と `environment_to_vlut.launch.py` のボクセル化を二重に作らない。どちらを共通のボクセルとして使うかを決め、参照元を統一する。

### Step3 (13〜2月)

13. **廃プラ・木質廃材への対応**
    実サンプル入手が前提。
    *暫定目標: コンクリートガラと同等の成功率*

14. **品目ごとの把持戦略の切り替え**
    形状・重量・変形性の違いに対応する。

15. **実機HW整合とサイクルタイム最適化**
    *暫定目標: 実機で実用水準の速度で動作する*
