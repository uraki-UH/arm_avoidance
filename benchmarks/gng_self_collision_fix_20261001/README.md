# 全身自己干渉の修正と保存GNGの再検証

状態: 通常学習・到達域生成の判定修正は実装済み。表面交差と完全内包を併用する最終方式の統合回帰は6対象49件成功。保存モデル全件監査と安全参照点の補完は両モデル完了。両モデルの辺再構成・被覆・VLUT更新・Viewer配信照合が完了。

対象: `topo_dual_arm_max` / `topo_dual_arm_max_long`。元データは `artifacts/gng_coverage_repair_20261001/{max,long}/model/`、今回の記録先は `artifacts/gng_self_collision_fix_20261001/`。発端の外装欠落による見逃しは[前段の監査](../gng_self_collision_audit_20261001/README.md)を参照。

## 判定仕様

| 項目 | 修正後 |
| --- | --- |
| 対象形状 | URDF内の全 `collision`。基部・胴体・外装・指を含む構成 |
| 表面の交差 | 元メッシュの三角形とFCLによる判定。形状ごとの外接範囲による候補絞込み |
| 完全内包 | 非退化な連結表面成分ごとに元メッシュ頂点を1点選択し、相手の占有データへ双方向に照会 |
| 内包用の占有データ | 表面セルと内部セルを含む、剛体ローカル座標のOctoMap。占有木どうしの直接衝突判定には不使用 |
| 凸primitiveの内包 | 球・BOX・カプセルの中心を代表内部点に使用。相手からの照会には各形状の解析的な内点判定 |
| 既定セル幅 | 0.001 m。通常2モデルの `gng.use_voxel_collision: true` 経路と到達域生成での使用 |
| 閉面確認 | 同位置頂点の統合後、辺の使用面数の偶数性と向きの収支0を確認。面積0の接続面は辺収支だけに保持し、内包検査用の表面成分の統合から除外 |
| 内部充填 | 余白付き格子の外周から6近傍で到達する空間を外部とし、表面と残る内部を占有 |
| BOX | 元の各半幅へ 0.002 m を加算する既存処理の継続 |
| CYLINDER / SPHERE | CYLINDERは同じ半径・軸長のカプセルによる保守近似、SPHEREは元半径 |
| 初期化失敗 | STL欠落・短い読込み・不正座標・開面・格子予算超過などをエラー化。検査を省略する代替形状なし |
| 姿勢変換欠落 | 対応するリンク変換がない場合のエラー化 |
| 判定経路の保護 | メッシュ登録後のstrict検査・FCLの無効化を拒否。未登録のFCL形状を後から有効化する切替えも拒否 |
| 除外 | 同一固定剛体内、可動関節を挟む形状付き基幹リンク対、明示されたペア |
| GNG辺 | 角度層と左右TCP層の全層。端点と各関節の線形補間刻み 0.025 rad による離散検査 |

方式の選択: 表面同士が交差しない完全内包は、表面判定だけでは見逃す場合あり。一方、1 mm占有木どうしの直接衝突判定は、首カバー等の狭い隙間で計算負荷の増大。最終方式は元メッシュの表面交差をFCLで判定し、完全内包を各表面成分の点照会で補完。追加除外による高速化なし。

高速化: 自己判定の表面照会は、幾何ポインタと両形状の絶対変換行列が全係数一致する場合に、同じペアの直近結果を再利用。角度や座標の丸めなし。一般FCL APIでは既定無効で、初期化後の幾何が不変な全身checkerで有効化。除外規則と包絡判定は再利用の前に適用。球形状登録時の回転未初期化も修正。位置・回転・除外変更、形状追加、番号再利用の回帰を含む構成。

予備比較: 同じ192姿勢ずつ、計384姿勢の監査CSV全18ファイルが高速化前後でbyte一致。両モデルのゼロ姿勢と既知の衝突2姿勢の結果も維持。非干渉姿勢の群別平均判定時間はmax約16.5〜20.2 msから8.94〜9.56 ms、long約23.3〜28.2 msから14.1〜21.2 ms。形状初期化を除く照会時間で、学習全体の短縮率ではない値。[比較記録](../../artifacts/gng_self_collision_fix_20261001/pilot_cached_comparison.json)。

内包の座標と成分: 元頂点をリンク・collision原点で世界座標へ変換し、相手のcollision原点の逆変換後に占有セルを参照。頂点の平均やセル中心への置換なし。面積ゼロの接続面による別固体の成分統合を回避し、第二成分だけが内側にある場合も検査対象。片方向だけの照会による包含方向の見落としを避ける双方向検査。

判定条件: `collision_method: mesh_surface_and_component_voxel_containment`。表面接触には元三角形、完全内包の補完には1 mmセルを使用。占有木どうしの接触と同じ結果という扱いなし。外部とつながる空洞は6近傍で外部として扱う構成で、有限解像度による隙間の表現限界あり。

除外の範囲: 可動関節の前後に形状のない固定リンクがある場合、その固定経路上の最初の形状付きリンクを基幹リンクとして選択。祖父母・兄弟を理由とする一律除外や、隣接リンクの固定外装全体への除外の拡張なし。ゼロ姿勢で接触しているという理由だけの全姿勢除外も廃止。明示除外は設定の `collision.self_collision_exclusion_pairs` と `collision.apply_self_collision_exclusion_pairs` から抽出したCSVを使用。現設定の左右それぞれの `finger_left` / `finger_right` を含む構成。監査側での独自ペア追加なし。

外装: max 11リンク、long 13リンクへvisualの原点・縮尺・geometryを複製。maxの左右link4/link6カバーは固定先collisionにも同じ三角形を含む構成。同一固定剛体内の除外による重複形状の取扱い。[追加箇所と入力ハッシュ](../../artifacts/gng_self_collision_fix_20261001/urdf_geometry_patch.json)、[固定先との形状照合](../../artifacts/gng_self_collision_fix_20261001/fixed_group_geometry_comparison.json)。

カメラ: `camera_link` の開いたSTL全305,907頂点を含むBOXへ置換。URDF上の寸法は約 25.0525 × 89.8563 × 25.0000 mm。判定時には上記BOXの各面2 mm膨張を追加適用。visual・慣性・接続位置の変更なし。実機寸法の校正は対象外。[全頂点包含と原点の記録](../../artifacts/gng_self_collision_fix_20261001/camera_collision_box.json)。

設定: 新項目 `collision.voxel_size: 0.001`。通常2モデルの設定と、trainer・到達域生成の互換項目の既定値も1 mm。旧項目 `collision.voxel_ball.voxel_size` は、新項目未指定時の値の引継ぎだけに使用。`max_spheres` 等の球分割設定は新しい学習判定に不使用。`gng.use_voxel_collision: false` は元メッシュの表面判定のみの経路。完全内包の補完を含む本検証とは条件差あり。

実装: [全身checker](../../gng_vlut_system/src/core/collision/geometric_self_collision_checker.cpp)、[体積ボクセル](../../gng_vlut_system/src/core/collision/fcl/solid_voxel_geometry.hpp)、[除外規則](../../gng_vlut_system/src/core/collision/self_collision_policy.hpp)、[関節補間](../../gng_vlut_system/src/core/collision/joint_segment_collision.hpp)。

## 確認済みの結果

| 対象 | 最終方式での状態 | 根拠 |
| --- | --- | --- |
| 除外規則・判定経路の切替え保護 | 8件成功 | `test_self_collision_policy` |
| 体積形状・成分・境界・不正入力・姿勢キャッシュ | 16件成功 | `test_solid_voxel_geometry` |
| checkerの完全内包・空洞・変換 | 7件成功 | `test_geometric_solid_containment` |
| GNG全層フィルタ・保存読込 | 3件成功 | `test_gng_collision_filter` |
| 全身変換・関節限界・形状登録 | 7件成功 | `test_reachability_collision` |
| 外装欠落・STL・カメラBOX | 8件成功 | `test_dual_arm_collision_geometry` |
| 最終方式でのゼロ姿勢・実姿勢 | 両モデルのゼロ姿勢は非干渉。既知の外装衝突2姿勢は棄却 | `pilot_hybrid_comparison.json` |
| 修正後の全保存姿勢・参照姿勢の監査 | maxの19,242姿勢中123棄却、longの24,302姿勢中374棄却 | `full_audit_summary.json` |
| 安全姿勢の補完 | max 20、long 78姿勢を追加選択 | 選択集合は19,139 / 24,006姿勢 |
| 全層辺の再構成 | 両モデル完了 | max 19,139姿勢・各層38,162辺、long 24,006姿勢・各層47,857辺。全層1成分・孤立0 |
| 安全参照点・独立参照点の被覆 | 両モデル完了 | 安全参照2 cm 100%、独立参照3 cmはmax 99.51% / long 99.20% |
| VLUTの再生成・Viewer配信 | 両モデル一致 | 全ノード・辺集合・関節角・TCPの一致。ブラウザ描画の目視検査は対象外 |

全件監査: 保存姿勢の合格はmax 19,119 / 19,242、long 23,928 / 24,302。参照姿勢は13,727 / 13,881、21,020 / 21,519、独立参照姿勢は19,167 / 19,232、19,127 / 19,501。関節限界違反は全群0件。辺の再構築前の姿勢単位の結果。[全件記録](../../artifacts/gng_self_collision_fix_20261001/full_audit_summary.json)。

棄却の内訳: 両モデルとも元学習ID 0〜9,999の1万姿勢は全件合格。max 123 / long 374件は旧補完ID 10,000以降の姿勢。元1万姿勢を保持したうえで、補完姿勢と全層の接続を修正。[内訳](../../artifacts/gng_self_collision_fix_20261001/rejected_node_groups.json)。

安全姿勢の選択: 元の安全姿勢19,119 / 23,928を保持し、安全参照の証拠姿勢20 / 78を追加。選択集合は19,139 / 24,006姿勢。参照点の2 cm被覆はmax 13,705 / 13,727から13,727 / 13,727、long 20,935 / 21,020から21,020 / 21,020。最大誤差は約19.996 / 19.998 mm。辺再構成前の選択集合での値であり、保存グラフ・連結成分の最終被覆は後段で別検証。[max選択](../../artifacts/gng_self_collision_fix_20261001/max/selection/selection.json)、[long選択](../../artifacts/gng_self_collision_fix_20261001/long/selection/selection.json)。

辺検査の並列化: workerごとにURDF・運動学・checkerを独立構築し、ノード順で結果を統合。関節近傍8候補から非干渉辺を最大2本選ぶ条件と、補間刻み0.025 radは不変。128姿勢の1 / 4 worker比較では、保存GNG・辺監査・非連結一覧・安定指標が完全一致。初期辺検査は162.41秒から45.32秒、初期化を含む全体は214.61秒から145.03秒。[比較記録](../../artifacts/gng_self_collision_fix_20261001/rebuild_workers_comparison.json)。

maxの再構成結果: 19,139姿勢、各層38,162辺、全3層で連結成分1・孤立0。追加20姿勢も元の成分へ接続。40,081候補のうち1,919辺を棄却、補間姿勢1,516,943点を検査。連結補完6辺、保存・再読込成功。runner全体3,429.87秒、初期辺検査3,041.42秒、worker追加初期化55.54秒、連結補完4.88秒。[確定指標](../../artifacts/gng_self_collision_fix_20261001/max/model/metrics.json)、[終了記録](../../artifacts/gng_self_collision_fix_20261001/rebuild_max_final_batch/report.json)。

maxの保存後被覆: 安全参照13,727点（左6,854・右6,873）は2 cm / 3 cmとも100%、最大距離1.999612 cm。独立の安全参照19,167点は2 cm以内14,116点（73.6474%）、3 cm以内19,073点（99.5096%）、最大距離3.59625 cm。全ノードと最大連結成分の結果は一致。[被覆・構造検証](../../artifacts/gng_self_collision_fix_20261001/max/verification/verification.json)、[尺度別集計](../../artifacts/gng_self_collision_fix_20261001/max/coverage_scales.json)。

maxのVLUT・Viewer: VLUTは10,435,043参照、参照ノード集合は保存全19,139と一致。Viewerの3グラフトピックは各19,139ノード・38,162辺でID集合・辺集合が一致、関節角とTCPの最大差0。フレームは `self_collision_max_2cm/base_link`。準備8.676秒、VLUT80.803秒、独立検証27.331秒、被覆集計9.725秒、Viewer54.189秒。[後処理集約](../../artifacts/gng_self_collision_fix_20261001/max_postprocess_summary.json)、[起動コマンド](../../artifacts/gng_self_collision_fix_20261001/max_postprocess_commands.jsonl)、[所有プロセス終了と既存環境の照合](../../artifacts/gng_self_collision_fix_20261001/max_postprocess_runtime_comparison.json)。

VLUT生成時の再保存: 通常trainerの既存経路によるGNGの再保存あり。maxでは全19,139ノードの生レコードが完全一致し、辺のID・age・activeを含む生レコードの多重集合も全層一致。差は辺の保存順序だけ。独立XML FKとの全ノードTCP最大差は左3.463e-8 m・右3.341e-8 m。再構成直後のコピーを `max/pre_vlut/gng.bin` に保持し、`rebuild_provenance.json` に前後それぞれのSHAと検証記録を保存。[再保存差分](../../artifacts/gng_self_collision_fix_20261001/max/vlut_resave_comparison.json)。

longの再構成結果: 24,006姿勢、各層47,857辺、全3層で連結成分1・孤立0。追加78姿勢も元の成分へ接続。50,054候補のうち2,197辺を棄却、補間姿勢1,651,455点を検査。連結補完7辺、保存・再読込成功。runner全体5,574.09秒、初期辺検査4,895.56秒、worker追加初期化59.08秒、連結補完8.90秒。[確定指標](../../artifacts/gng_self_collision_fix_20261001/long/model/metrics.json)、[終了記録](../../artifacts/gng_self_collision_fix_20261001/rebuild_long_final_batch/report.json)。

longの保存後被覆: 安全参照21,020点（左10,535・右10,485）は2 cm / 3 cmとも100%、最大距離1.999845 cm。独立の安全参照19,127点は2 cm以内13,678点（71.5115%）、3 cm以内18,974点（99.2001%）、最大距離4.95360 cm。全ノードと最大連結成分の結果は一致。[被覆・構造検証](../../artifacts/gng_self_collision_fix_20261001/long/verification/verification.json)、[尺度別集計](../../artifacts/gng_self_collision_fix_20261001/long/coverage_scales.json)。

longのVLUT・Viewer: VLUTは14,584,205参照、参照ノード集合は保存全24,006と一致。Viewerの3グラフトピックは各24,006ノード・47,857辺でID集合・辺集合が一致、関節角とTCPの最大差0。フレームは `self_collision_long_2cm/base_link`。準備9.273秒、VLUT85.009秒、独立検証33.131秒、被覆集計10.826秒、Viewer64.195秒。[後処理集約](../../artifacts/gng_self_collision_fix_20261001/long_postprocess_summary.json)、[起動コマンド](../../artifacts/gng_self_collision_fix_20261001/long_postprocess_commands.jsonl)、[所有プロセス終了と既存環境の照合](../../artifacts/gng_self_collision_fix_20261001/long_postprocess_runtime_comparison.json)。

longの再保存も全24,006ノードの生レコードと全層の辺の生レコード多重集合が一致し、差は辺順序のみ。独立XML FKとのTCP最大差は左4.205e-8 m・右4.209e-8 m。`long/pre_vlut/gng.bin` とstage別SHAを保持。[再保存差分](../../artifacts/gng_self_collision_fix_20261001/long/vlut_resave_comparison.json)。比較helper初回はlongのURDF basenameの指定誤りで失敗。実パスへの修正後に成功し、失敗・修正SHAの記録を後処理集約へ保持。VLUT生成や本体判定の失敗とは別事象。

辺の密度: maxの旧角度層471,346辺・左TCP層26,858辺・右TCP層24,456辺を破棄し、全層を同じ38,162本の検査済み近傍辺へ再構成。longも旧角度層571,543辺・左TCP層35,925辺・右TCP層34,835辺から、各47,857辺へ再構成。両モデルの角度層は疎な接続へ変更。連結性と被覆は検証対象だが、旧グラフに対する最短経路長や経路探索性能の同等性は未検証。

大モデルの処理量: 最初の2候補がすべて非干渉の場合の補間点数はmax 1,491,114、long 1,625,153。全8候補を最後まで検査する場合は7,303,772 / 7,869,951点。いずれも初期辺構成の見積で、衝突時の途中棄却と連結補完を含む実測値ではない条件。[max見積](../../artifacts/gng_self_collision_fix_20261001/max/initial_interpolation_estimate.json)、[long見積](../../artifacts/gng_self_collision_fix_20261001/long/initial_interpolation_estimate.json)。

最終方式の回帰: 6対象49件成功。[回帰ログ](../../artifacts/gng_self_collision_fix_20261001/test_final_batch/001_collision_regression/output.log)、[終了・cleanup記録](../../artifacts/gng_self_collision_fix_20261001/test_final_batch/report.json)。先行方式の結果とは別の集計。

姿勢キャッシュ追加時の先行回帰: 球を含む姿勢変更・回転変更の2試験で失敗。球形状の変換行列の初期化漏れが原因で、単位行列からの生成へ修正後、上記最終49件で成功。[修正前記録](../../artifacts/gng_self_collision_fix_20261001/test_cache_batch/report.json)、[最終集計](../../artifacts/gng_self_collision_fix_20261001/final_test_summary.json)。

追加回帰の内容: 表面判定だけで衝突なしとなる球・BOX・カプセルの完全内包、分離した第二メッシュ成分だけの内包、登録順を反転した双方向の一致、primitiveによるメッシュ内包、二壁・U字の開放空洞、回転・並進したcollision原点。`checkCollision()` と衝突対一覧の一致も照合。新規試験は [test_geometric_solid_containment.cpp](../../gng_vlut_system/test/test_geometric_solid_containment.cpp)。

先行の直接ボクセル判定: 2.5 mmセルでは、元メッシュ間に約2.2〜3.023 mmの隙間があるゼロ姿勢の対を接触と判定。1.25 mmではmaxは0対、longは `finger_left/right` と `link7` の左右4対が残存。1 mmでは両モデル0対となる結果。占有木どうしを直接比較した当時のゼロ姿勢結果であり、最終方式の検証とは別条件。ゼロ姿勢のための除外ペア追加なし。[元メッシュの距離・max](../../artifacts/gng_self_collision_fix_20261001/zero_mesh_max.json)、[long](../../artifacts/gng_self_collision_fix_20261001/zero_mesh_long.json)、[1.25 mm・max](../../artifacts/gng_self_collision_fix_20261001/zero_voxel_max_1p25mm.json)、[long](../../artifacts/gng_self_collision_fix_20261001/zero_voxel_long_1p25mm.json)、[1 mm・max](../../artifacts/gng_self_collision_fix_20261001/zero_voxel_max_1mm.json)、[long](../../artifacts/gng_self_collision_fix_20261001/zero_voxel_long_1mm.json)。

時間の取扱い: 上記1 mmのゼロ姿勢記録は、占有木どうしの直接判定かつ外接範囲の修正前の計測。約181 / 162 msの初回判定時間は最終実装の性能値に不使用。全件姿勢監査はmax約843秒、long約1,534秒で完了。再構成はmax約57.2分、long約92.9分で完了し、保存後の被覆は上記の結果。元メッシュ診断の全周角度サンプルはURDF限界外を含むため、許容可動域での衝突件数への転用なし。

先行のbuild・回帰・旧pilotには失敗記録あり。先行34件の成功はメッシュ読込みと縮退面処理の修正後の回帰結果。最終方式の統合回帰とは別記録。旧pilotの途中失敗を、姿勢監査完了や安全姿勢0件という結果へ換算しない。[先行回帰](../../artifacts/gng_self_collision_fix_20261001/test_batch/report.json)、[先行pilot](../../artifacts/gng_self_collision_fix_20261001/pilot_batch/report.json)。

## 最終集約と終了確認

[両モデルの最終集約](../../artifacts/gng_self_collision_fix_20261001/final_summary.json)。安全参照点の全ノード被覆と最大連結成分の被覆は両モデルで一致。

| モデル | 棄却姿勢 | 追加証拠姿勢 | 最終ノード | 辺数／層 | 連結成分／層 | 安全参照2 cm | 独立安全参照3 cm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| max | 123 | 20 | 19,139 | 38,162 | 1 | 100% | 99.51% |
| long | 374 | 78 | 24,006 | 47,857 | 1 | 100% | 99.20% |

終了確認: 所有試験プロセス残存0。開始時の既存20プロセスはPID・起動時刻・argvを維持し、3コンテナの状態も一致。[最終runtime照合](../../artifacts/gng_self_collision_fix_20261001/runtime_final.json)。実行した32バッチ・47ケースのコマンド、終了状態、先行失敗、および追加コマンド記録の参照先は[実行コマンド集約](../../artifacts/gng_self_collision_fix_20261001/execution_commands.json)。

保全確認: 旧GNG / VLUTの8ファイルとSTL資産174ファイルのSHA・サイズが記録時と一致。[ファイル保全](../../artifacts/gng_self_collision_fix_20261001/file_integrity_final.json)。再構成入力、保存各段階の生成物、最終バイナリ等の44 SHA・14状態照合も全件一致。[生成物の最終照合](../../artifacts/gng_self_collision_fix_20261001/rebuild_provenance_final_check.json)。

一時ファイル清掃: 所有PID・実行区間・ファイル時刻・PID消滅が確認できた一時URDF45件のみ削除。未知PID・既存ユーザーファイル・`/tmp/temp_robot_online.urdf`は保持。[削除対象と根拠](../../artifacts/gng_self_collision_fix_20261001/owned_resolved_urdf_cleanup_20260930T224631_319015Z.json)。清掃コマンド（終了済み）:

```bash
docker exec gng_cpu_container python3 /ros2_ws/src/artifacts/gng_self_collision_fix_20261001/cleanup_owned_resolved_urdf.py --apply
```

## 修正版の表示

両モデルとも保存・VLUT・Viewer配信の照合済み。Docker内でのmax表示コマンド:

```bash
source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/artifacts/gng_self_collision_fix_20261001/max/model/preview.yaml \
  joint_control_backend:=viewer enable_dynamixel_input:=false
```

longの表示は上記パスの `/max/model/` を `/long/model/` へ変更。Viewer上の名前はそれぞれ `self_collision_max_2cm` / `self_collision_long_2cm`。上記は表示用backendの指定。旧モデルの既定パスへの上書きは未実施で、修正版の設定ファイルを明示する運用。

## 再現手順

前提: `gng_cpu_container`、`/ros2_ws/src`へのworkspaceマウント、修正後のROS build。package依存は `libfcl-dev` / `octomap` を含む構成。既存出力先は上書き拒否。再実行先には未使用のディレクトリ名を指定。

```bash
docker exec gng_cpu_container python3 \
  /ros2_ws/src/artifacts/gng_self_collision_fix_20261001/run_batch.py \
  /ros2_ws/src/artifacts/gng_self_collision_fix_20261001/build_cases.json \
  --output /ros2_ws/src/artifacts/gng_self_collision_fix_20261001/build_reproduction \
  --repeats 1 --timeout-sec 600 --max-total-sec 650 --estimate-sec 400

docker exec gng_cpu_container python3 \
  /ros2_ws/src/artifacts/gng_self_collision_fix_20261001/run_batch.py \
  /ros2_ws/src/artifacts/gng_self_collision_fix_20261001/test_cases.json \
  --output /ros2_ws/src/artifacts/gng_self_collision_fix_20261001/test_reproduction \
  --repeats 1 --timeout-sec 180 --max-total-sec 240 --estimate-sec 60

docker exec gng_cpu_container python3 \
  /ros2_ws/src/benchmarks/gng_self_collision_fix_20261001/build.py \
  --output /tmp/gng_self_collision_fix_20261001/safe_graph
```

`build.py` のコンパイル上限: 180秒。ROS常駐ノードの起動なし。runnerの各ケースに終了状態・実argv・ログ・所有プロセスのcleanup結果を保存。

保存モデルの再検証手順:

| 順序 | 入口 | 内容 |
| --- | --- | --- |
| 1 | `prepare_exclusions.py --config <model.yaml> --output <新規CSV>` | `collision.self_collision_exclusion_pairs` と `collision.apply_self_collision_exclusion_pairs` を検査し、設定とCSVのSHA-256付き記録を生成。適用無効時は空集合 |
| 2 | `pipeline.py prepare --root ... --model max\|long --output <新規dir>` | 旧補完実験の片腕到達証拠をCSVへ展開、入力ハッシュ保存 |
| 3 | `safe_graph --mode audit --exclusion-pairs <CSV> --voxel-size 0.001 ...` | 保存全姿勢・参照・独立参照の関節限界、FK、修正済み全身衝突判定。CLIは[audit_cases.json](../../artifacts/gng_self_collision_fix_20261001/audit_cases.json) |
| 4 | `pipeline.py select --audit <audit結果> ...` | 安全な元姿勢を保持、再判定に合格した参照点を2 cmで覆う証拠姿勢を追加。平均化なし |
| 5 | `safe_graph --mode rebuild --exclusion-pairs <同じCSV> --voxel-size 0.001 --safe-node-ids ... --extra-nodes ...` | 旧辺を破棄、関節近傍候補を端点・補間で再検査。合格辺を角度・左右TCPの各層へ保存 |
| 6 | `prepare_model.py --root ... --model max\|long --model-dir <再構成dir>` | 保存・再読込成功、ノード数、設定との分解能一致、`base_link`を確認後、`preview.yaml` / `expected.json`を新規生成 |
| 7 | 更新済みURDFと `preview.yaml` からのVLUT再生成 | 削除ノード参照の除去と残るノード占有の更新。設定は `gng.vlut_only: true`、処理完了は別途確認 |
| 8 | `pipeline.py verify --audit ... --selection ... --model-dir ... --require-vlut ...` | ノード集合、保持レコード、全層辺と監査の一致、VLUT参照集合、安全参照の2 cm被覆を検証 |
| 9 | `audit_runtime.py <記録dir>` | 全所有処理の終了後、事前記録のPID・起動時刻・argvと既存コンテナを照合。所有プロセスの残存と既存プロセスへの影響を検査 |

設定公開: `prepare_model.py` はバイナリ読込みと整合確認の後に2ファイルを作成し、後段の書込み失敗時は今回の先行生成ファイルを撤回。Viewer名は `self_collision_{max,long}_2cm`、期待フレームはその名前に `/base_link` を付けた値。既存設定への上書き拒否。保存済みデータの安全性判定は前段の全件監査・再構成の責務。

実行状態の照合: `audit_runtime.py` は `runtime_baseline.json` / `runtime_baseline_identity.json` を入力とする読取り中心の監査。既存プロセスやコンテナの停止なし。`runtime_final.json` に比較結果を保存。所有プロセスの終了確認は、監査・再構成・VLUT生成・試験などの全工程終了後の手順。

[予備試験のケース定義](../../artifacts/gng_self_collision_fix_20261001/pilot_cases.json)は追加ID 10000から64姿勢と参照64姿勢の上限付き。`select` / `verify` は全件監査完了フラグが必要なため、予備試験結果からの全件評価への転用不可。さらに `select` / `verify` は上記 `collision_method` を検査し、旧方式の監査結果の混入を拒否。`verify` では監査・再構成の方式、分解能、明示除外、関節刻みの一致も確認。具体的な引数は [pipeline.py](pipeline.py)、[safe_graph.cpp](safe_graph.cpp)、[除外設定抽出](prepare_exclusions.py)、[Viewer設定生成](prepare_model.py)、[実行状態の監査](audit_runtime.py) を正本とする。

## 評価の限界

- 関節次元: 左 `L_joint1..7`、右 `R_joint1..7` の14値。腰・首・グリッパーは0固定。これらを動かす運用の自己干渉保証は対象外。
- 辺の条件: 最大関節刻み 0.025 rad の離散姿勢での判定。サンプル間や連続時間の非干渉証明は対象外。
- 幾何条件: 元メッシュの表面交差、1 mmセルでの内包補完、BOX膨張、CYLINDERのカプセル化の併用。占有木どうしの直接衝突判定と同一結果という保証なし。寸法校正やモデル誤差を含む実機の検証は未実施。
- 被覆条件: 再判定で安全となった有限の片腕参照TCP点。両腕同時の姿勢、手先方向、未観測点、連続可動域全体の被覆は対象外。
- 連結条件: 全ノード集合の被覆と、最大連結成分だけの被覆を別集計。ノードの存在だけによる経路成立の主張なし。
- VLUTの形状範囲: 環境衝突用VLUTは既存の可動18リンクの範囲。今回の全身化は自己干渉判定であり、固定ボディを含む環境占有への拡張は対象外。
- 既存データ: 旧GNG・VLUT・到達マップの無衝突フラグは旧形状と旧判定の結果。ソース修正だけによる保存ファイル再生成の完了扱いなし。
