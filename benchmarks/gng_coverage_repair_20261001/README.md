# 既知手先位置の被覆を保つ双腕GNG補完（2026-10-01）

> 追試（2026-10-01）: max追加64姿勢のうち2姿勢で、元URDFにcollisionのない首・肩外装との衝突を実証。以下のFCL成功は既存形状・除外条件での結果であり、外装との非干渉保証なし。[反例・条件・再現記録](../gng_self_collision_audit_20261001/README.md)。

> 修正版（2026-10-01）: max / longとも全身自己干渉の再検査・安全辺の再構成・VLUT・Viewer配信照合まで完了。[修正版の結果・表示コマンド](../gng_self_collision_fix_20261001/README.md)。以下は旧形状・旧判定での記録。

結果: 両モデルの既知参照点を2 cm以内で100%被覆。未使用の片腕姿勢では3 cm以内が99.2〜99.5%、2 cm以内は71.5〜74.5%。全可動域の2 cm保証は未達。

元1万姿勢のID・関節角・全辺を保持し、既存reachability mapの証拠姿勢から補完候補を選択。元モデルよりノード数が増加。圧縮対象は追加候補であり、元1万姿勢の削除なし。標準GNG v9・VLUT v2による別保存モデル。学習パイプラインへの自動補完組込みなし。

## モデル規模と接続

| モデル | 元姿勢数 | 全追加候補数 | 採用追加数 | 最終姿勢数 | 追加辺数／層 | 関節角層の連結成分数 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| max | 10,000 | 13,881 | 9,242 | 19,242 | 18,359 | 1 |
| long | 10,000 | 21,519 | 14,302 | 24,302 | 28,445 | 1 |

追加候補数の削減: max 33.4%、long 33.5%。元の全関節姿勢と同時双腕姿勢の保持。追加姿勢の平均化なし。TCP座標層には元から孤立・複数成分が存在し、その全接続は今回の対象外。追加ノードは全層で元ノードの成分へ接続。

## 既知参照点の被覆

対象: 3 cmセルに保存された実証拠関節姿勢のFK位置。セル中心との比較ではない。各腕の最近傍TCPまでのユークリッド距離。最終保存TCP・保存qからの独立FKの両方で2 cm条件を検査。

| モデル・腕 | 参照点数 | 2 cm以内・補完前 (%) | 2 cm以内・補完後 (%) | 最大距離・補完前 (cm) | 最大距離・補完後 (cm) |
| --- | ---: | ---: | ---: | ---: | ---: |
| max left | 6,927 | 11.14 | 100.00 | 42.63 | 1.9989 |
| max right | 6,954 | 7.20 | 100.00 | 49.57 | 1.9996 |
| long left | 10,789 | 11.86 | 100.00 | 49.75 | 1.9994 |
| long right | 10,730 | 10.27 | 100.00 | 50.64 | 1.9998 |

## 独立姿勢での評価

条件: seed=20261001、左右各10,000回の関節範囲内一様抽選、他腕・腰・頭・グリッパーは0。全身自己衝突検査の合格分のみ集計。選択用候補との混用なし。静止側の0姿勢を分母へ加えず、可動側の手先位置のみが対象。姿勢抽選分布に対する割合であり、空間体積比ではない。

| モデル・腕 | 検査合格数 | 2 cm以内・前 (%) | 2 cm以内・後 (%) | 3 cm以内・後 (%) | 最大距離・前 (cm) | 最大距離・後 (cm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| max left | 9,599 | 12.01 | 74.55 | 99.54 | 43.24 | 3.40 |
| max right | 9,633 | 9.55 | 73.05 | 99.53 | 47.60 | 3.59 |
| long left | 9,746 | 10.84 | 72.76 | 99.22 | 50.51 | 4.95 |
| long right | 9,755 | 9.05 | 71.49 | 99.45 | 49.49 | 3.93 |

5 cm以内: 両モデル・両腕とも100%。左右独立の位置被覆であり、任意の同時双腕目標・TCP姿勢角・14次元関節空間全体の被覆保証は対象外。全域2 cmを目指す場合は、参照サンプリングの追加と、別系列の検証が必要。

[TCP分布比較図](../../artifacts/gng_coverage_repair_20261001/tcp_coverage.png)

## 処理と検証

1. 元1万姿勢を固定し、未被覆参照点の初期最遠順で実証拠姿勢を選択。追加ごとに両TCPの2 cm被覆を更新。最小ノード数の最適性保証なし。
2. 元・追加の全姿勢に関節限界とstrict FCL自己衝突検査。既存の隣接リンク・指などの除外規則を継承。環境障害物の検査なし。
3. 初回の近傍探索で、関節空間の近傍8候補から各新ノードを始点とする補間検査合格辺を最大2本追加。未接続成分は全ノードから64候補の橋渡し探索。各関節の補間刻み0.05 rad。既存辺の再検査とサンプル間の連続衝突保証なし。
4. 通常トレーナーの`vlut_only:=true`で標準VLUTを生成。元ID・q・辺レコード、全FK、全参照点、全ノードの18リンク占有参照を再読込検査。
5. 元5姿勢＋追加5姿勢×18リンク×2モデルのVLUT集合を、XMLからの独立全リンクFKで照合。360組すべて完全一致。格子境界1e-7 mの許容処理は実装済み、今回の差分許容使用0組。
6. ROS_DOMAIN_ID=91で通常Viewer launchを起動。全ID・関節角・左右TCP・辺数・座標系・robot descriptionの配信を照合。両モデル成功、試験launch終了。実ブラウザ描画・実機動作は未検証。

初回パイロット: 肩座標を全身へ誤適用する既存の変換不具合で失敗。TCP・関節限界は一致し、衝突用base_linkが右肩位置へ移動。修正後の元1万姿勢と追加姿勢は全件合格。[修正・回帰範囲](../../gng_vlut_system/docs/releases/2026-10-01_dual_arm_link_transforms.md)。回帰7/7件、関連CTest 3/3対象、通常build/install成功。新ノードの可操作性特徴は未計算の無効状態。

## Viewer起動

Docker内での通常モデル。元モデルファイルの置換なし。longはパス内の`max/model`を`long/model`へ変更。

```bash
source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/artifacts/gng_coverage_repair_20261001/max/model/preview.yaml \
  joint_control_backend:=viewer enable_dynamixel_input:=false
```

namespace: `coverage_max_2cm` / `coverage_long_2cm`。既定表示の辺はTCP座標層。`edge_mode:=0`はメインmapを関節角層の辺へ切替。モデルファイルは各`model/gng.bin`と`model/vlut.bin`。

## 再現コード・実行記録

| 処理 | コード | 実行argv・終了状態 |
| --- | --- | --- |
| 被覆候補選択 | [select_coverage.py](select_coverage.py) | [selection_batch](../../artifacts/gng_coverage_repair_20261001/selection_batch/report.json) |
| 衝突・接続・独立姿勢生成 | [repair_graph.cpp](repair_graph.cpp)、[build.py](build.py) | [graph_batch](../../artifacts/gng_coverage_repair_20261001/graph_batch/report.json) |
| 保存先・Viewer設定 | [prepare_model.py](prepare_model.py) | 各モデルの`model/preview.yaml` |
| 通常VLUT生成 | `offline_urdf_trainer_dual.launch.py` | [vlut_batch](../../artifacts/gng_coverage_repair_20261001/vlut_batch/report.json) |
| 全データ・被覆検査 | [verify_repair.py](verify_repair.py) | [verification_batch](../../artifacts/gng_coverage_repair_20261001/verification_batch/report.json) |
| 独立位置検査 | [verify_vlut_geometry.py](verify_vlut_geometry.py)、[局所占有出力](dump_local_voxels.cpp) | [geometry_batch](../../artifacts/gng_coverage_repair_20261001/geometry_batch/report.json)、[局所出力コマンド](../../artifacts/gng_coverage_repair_20261001/local_voxel_commands.json) |
| ROS配信検査 | [probe_viewer.py](probe_viewer.py) | [viewer_batch](../../artifacts/gng_coverage_repair_20261001/viewer_batch/report.json) |
| coreビルド・回帰 | core・testの通常ビルド | [全コマンドと終了状態](../../artifacts/gng_coverage_repair_20261001/core_fix_validation/summary.json) |
| 実行状態の確認 | [audit_runtime.py](audit_runtime.py) | [runtime_final](../../artifacts/gng_coverage_repair_20261001/runtime_final.json) |

補助C++の再ビルド例（Docker内）。通常package build/install済みの環境が前提。

```bash
source /ros2_ws/install/setup.bash
python3 /ros2_ws/src/benchmarks/gng_coverage_repair_20261001/build.py \
  --output /tmp/gng_coverage_repair_20261001/repair_graph
```

上表のJSONに実行した全引数・ログ・終了コード・後片付け結果を保存。各`*_cases.json`がbatch入力。再実行は別の出力先を指定し、既存モデルへの上書きを拒否。入力モデル・到達マップ・URDFのSHA256は各`model/verification.json`、形状と占有生成コードのSHA256は`model/vlut_geometry_verification.json`に保存。

実測時間: 候補選択2件20.5秒、グラフ生成2件647.7秒、VLUT生成2件141.0秒、保存・被覆検査2件116.5秒、独立位置検査2件44.8秒、Viewer検査2件68.3秒。通常package build/install892.4秒。並行実行を含むため単純合計は全体経過時間ではない。グラフ生成の開始前見積もり600秒に対して実測647.7秒。

実行状態: 所有試験プロセス残留0件、開始時の21プロセスと3コンテナを維持。作業中に別の`dual_arm_control.launch.py`と子プロセスの起動を観測し、停止・再起動なし。初回runtime監査ではこの並行作業を差分として検出し、所有関係を分けた再確認結果を上表へ保存。
