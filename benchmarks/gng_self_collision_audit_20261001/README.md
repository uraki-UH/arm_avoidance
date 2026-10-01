# 補完GNGの胴体外装との自己干渉監査（2026-10-01）

結果: 表示中の max 補完モデルから、既存検査で合格する外装衝突姿勢を実証。自己干渉検査の存在だけでは、表示形状との非干渉は未成立。

## 実測

対象: `artifacts/gng_coverage_repair_20261001/max/model/gng.bin` の追加ID 10000〜10063、64姿勢。保存14関節角を変更せず、左7関節・右7関節へ適用。腰・首・グリッパーは0。関節限界内、暗黙の関節角clampなし。比較条件はURDFの衝突形状だけ。

| 条件 | 検査姿勢数 | 通常strict FCLで衝突の姿勢数 | ゼロ姿勢で非接触の胴体・腕ペアとの接触姿勢数 |
| --- | ---: | ---: | ---: |
| 元URDF | 64 | 0 | 0 |
| 胴体外装5リンクを追加した診断用URDF | 64 | 2 | 2 |

| ノードID | 衝突リンク | 元モデルの保存フラグ | ゼロ姿勢での当該ペア接触 | 当該ペアの自動除外 |
| --- | --- | --- | --- | --- |
| 10025 | `R_link7` / `neck_tilt_cover_link` | 無衝突 | なし | なし |
| 10062 | `R_finger_right` / `L_shoulder_cover_link` | 無衝突 | なし | なし |

追加した形状は元URDFのvisualの原点・scale・geometryのコピー。対象: `waist_cover_link`、左右`shoulder_cover_link`、`neck_tilt_cover_link`、`realsense_mount_link`。既存collisionの置換・膨張なし。全メッシュの読込みとFCLオブジェクト生成を検査し、球への読込み失敗フォールバックなし。反例のR_link7・R_finger_rightは、visualとcollisionの三角形頂点配列が完全一致。通常checkerの集計に加え、除外を無視したbody対armの直接FCLでも同じ2姿勢を検出。

`torso_link`と左右`link1`は両条件の全64姿勢で直接FCL接触あり。ただしゼロ姿勢でも接触し、接続部として通常除外されるペア。上表の2件とは別集計。この事実のみを、64件すべての不正な衝突の証拠とは扱わない。

測定範囲: maxの選択した64姿勢。全19,242姿勢の衝突数・発生率、longの姿勢衝突数、辺の補間、実機の装着寸法は未測定。反例確認が目的のため、予備試験後の全件検査は未実施。

## 衝突姿勢の図

[ノード10062・視点A](../../artifacts/gng_self_collision_audit_20261001/node10062_mesh_view_a.png)・[視点B](../../artifacts/gng_self_collision_audit_20261001/node10062_mesh_view_b.png)。青: 左肩カバー、橙: 右指。元の全三角形を保持。関節角は保存GNGと監査CSVで一致、独立XML FKのTCPと保存TCPの最大差1.15e-8 m。図は位置関係の補助であり、交差の判定根拠は上記FCL結果。

[描画コード](render_collision_pair.py)、[関節角・全リンク変換・入力ハッシュ](../../artifacts/gng_self_collision_audit_20261001/node10062_mesh_geometry.json)。描画コマンド `timeout 120 python3 benchmarks/gng_self_collision_audit_20261001/render_collision_pair.py`、実測51.48秒、終了0。

## 原因と関連する実装上の問題

- 確認済みの原因: 上記外装のcollision要素欠落。Viewerは既定でvisualを表示し、`GeometricSelfCollisionChecker`はcollisionのみ読込み。首・左肩カバーとの見逃しを今回実証。
- 形状比較: max/longともbase_link、torso_linkのvisualとcollisionは、1 µm量子化の三角形集合が一致。双方向の最近傍頂点距離は最大0 m。胴体本体STLの違いを原因とする根拠なし。
- 除外規則: 親子・祖父母と孫・同じ親の兄弟リンクを自動除外。strict FCLでも有効。torsoと左右link1も除外対象。ただし今回の2件はこの除外による漏れではなく、外装形状欠落による漏れ。
- 通常トレーナーの別問題: `offline_urdf_trainer.cpp`の`setStrictMode(true)`がself_checker生成前のnull判定内にあり、mesh経路で無効。非strict判定にはmesh-meshの実装なし。今回の補完用ツールは生成後にstrictを明示しており、上記2件の直接原因とは別。
- 初期衝突の取扱い: 通常トレーナーは設定次第でゼロ姿勢の接触ペアを全姿勢の除外へ自動追加。strictの順序だけを変更した場合も除外結果に影響。既存指示・設定との整合を含めた修正が必要。

参照箇所: `gng_vlut_system/src/core/collision/geometric_self_collision_checker.cpp`、`gng_vlut_system/src/core/collision/self_collision_checker.cpp`、`gng_vlut_system/src/offline_tools/offline_urdf_trainer.cpp`。本監査で本体・正式URDF・保存グラフの修正なし。

以前の「全姿勢のFCL検査成功」は既存形状・除外設定の下での実測。外装との非干渉を意味しない。参照到達マップも同じ衝突モデルに依存するため、修正後の再検査はノード・辺だけでなく、到達マップと被覆評価にも必要。

## 再現と記録

- [監査コード](audit.cpp)、[ビルド](build.py)、[診断用URDF生成](prepare_diagnostic.py)。入力モデルへの書込みなし、既存出力先への上書き拒否。
- [衝突試験の全argv・ログ・終了状態](../../artifacts/gng_self_collision_audit_20261001/pilot_batch/report.json)。2条件各1回、終了0、所有プロセスグループの残留なし。開始見積り30秒に対して実測88.95秒。各条件のメッシュ初期化に約39〜43秒、64姿勢の検査部分は約3秒。
- [元URDFの全ペア集計](../../artifacts/gng_self_collision_audit_20261001/pilot_batch/001_max_original_urdf/result/audit_metrics.json)、[外装追加後の全ペア集計・反例の14関節角](../../artifacts/gng_self_collision_audit_20261001/pilot_batch/001_max_body_visual_collision/result/audit_metrics.json)。各resultにnode_audit.csv、body_pair_summary.csv、first_collision_samples.csv。
- [反例の腕側メッシュ照合](../../artifacts/gng_self_collision_audit_20261001/example_arm_mesh_compare.json)。R_link7・R_finger_rightとも三角形配列の完全一致、頂点距離差0 m。
- [形状比較](../../artifacts/gng_self_collision_audit_20261001/mesh_compare.json)、[形状比較の起動コマンド・終了状態](../../artifacts/gng_self_collision_audit_20261001/mesh_compare_commands.json)。初回の全リンク詳細比較は120秒timeout、胴体詳細比較＋全リンク寸法表へ範囲を絞った試験は正常終了。
- [入力ハッシュ](../../artifacts/gng_self_collision_audit_20261001/input_hashes.json)、[診断用URDF生成記録](../../artifacts/gng_self_collision_audit_20261001/diagnostic_urdf_manifest.json)。longの診断用コピーも生成済み、longのFCL姿勢試験は未実施。

実行コマンド（既存Docker内、常駐ROSの起動なし）。再実行は別のoutput先を指定。

```bash
docker exec gng_cpu_container python3 \
  /ros2_ws/src/benchmarks/gng_self_collision_audit_20261001/build.py \
  --output /tmp/gng_self_collision_audit_20261001/audit

docker exec gng_cpu_container python3 \
  /ros2_ws/src/artifacts/gng_self_collision_audit_20261001/run_batch.py \
  /ros2_ws/src/artifacts/gng_self_collision_audit_20261001/pilot_cases.json \
  --output /ros2_ws/src/artifacts/gng_self_collision_audit_20261001/pilot_batch \
  --repeats 1 --timeout-sec 120 --max-total-sec 250 --estimate-sec 15
```

終了確認: [runtime_final.json](../../artifacts/gng_self_collision_audit_20261001/runtime_final.json)で所有試験プロセス残留0件、開始時の21プロセスと3コンテナの維持。元モデル6ファイルと正式URDF2ファイルのSHA256一致。[比較・未変更確認](../../artifacts/gng_self_collision_audit_20261001/verification.json)。全試験・描画プロセス終了済み。
