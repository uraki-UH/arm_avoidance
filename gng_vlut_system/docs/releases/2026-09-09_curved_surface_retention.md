# 2026-09-09 - Curved Surface Retention

## Summary

曲面の新規生成と維持の条件を分離。生成済みの曲面は現在のGNGノードで再検証し、
接続切れや元平面クラスタ数の変化だけでは破棄しない。表示の描き残しではなく、モデル所属の維持。

## Changed

- 元平面を2クラスタ含む曲面という新規表示条件と、既存の形状フィット・sharp境界条件は維持。
- 表示対象として生成された曲面を実ノードIDで追跡。現在位置から前回モデルへの近似距離 `abs(F)/norm(grad F)` と現在法線でノードごとに再判定。
- 逸脱点だけを所属から除外。生成時の支持点の60%かつ12点が残り、適合点全体のRMSが6mm以内なら維持。
- 維持モデルを確保した後、残りの領域を従来方式で新規抽出。維持モデルの再確認は128回の数値フィット予算の対象外。
- 下位の平面クラスタは変更しない。必要な場合だけ上位のLocalPatchを維持所属・逸脱所属に分け、元平面の同一性を保存。
- 同じ曲面のIDと色を維持。現在の元平面が1枚へ再編成されても、生成時に条件を満たしていれば表示を維持。

## Added

- `surface_model_tracking.hpp/.cpp` に時間方向の維持状態。GNG学習コアの変更なし。
- 一度外れた元ノードも次回以降の再検証候補として保持し、再適合時には所属へ復帰。
- 接続切れ・逸脱・復帰・法線不一致・非有限座標・支持喪失・RMS・ID順序変更・消失ID・重複ID・入力リセット・表示色・探索予算の回帰テスト。
- `check_surface_models.py --disconnect-after <sec>` に、保存GNGの全edgeだけを途中で消す比較条件。

## Fixed

GNG edgeの一時的な欠落で1曲面が各1平面の候補へ分断され、表示条件を満たさずすぐ消える挙動。
既知のsharpな平面ペアをまたぐ維持候補は棄却し、現在ノードから新規抽出。

## Removed

なし。元GNGのノード・edge・平面所属、保存テンプレート、ROS msg定義は変更なし。

## Behavior Impact

- 既定で維持ON。既存の `ais_gng.launch.py` と単独の `surface_models.launch.py` の両方に適用。
- 描画座標・法線は現在フレーム由来。edgeは現在の実GNG edgeだけで、消えた接続の描き残しや架空edgeはなし。
- モデル全体の解除条件は、支持点数・支持率・RMSの不適合、現在のsharp境界、入力座標系変更・frame番号巻き戻り・stamp巻き戻り・空入力。
- 重複ノードIDがあるフレームは追跡を解除し、そのフレームだけの抽出結果を出力。
- 生成時のID集合を支持率の分母として固定。毎回少しずつ減るだけで無期限に残り続ける挙動を抑制。
- 維持係数への位置合わせ・再フィット、新しく生成されたIDの追加成長は未実装。大きな移動・変形・ID入れ替わりでは新規抽出へ復帰。

## Topics / Params / Messages

設定: `ais_gng_cpu/src/ais_gng/config/surface_model.yaml`。

| Parameter | Default | 用途 |
| --- | --- | --- |
| `surface_model.retention.enable` | `true` | 維持判定の有効化 |
| `surface_model.retention.max_point_residual` | `0.012` | 各ノードの近似曲面距離[m] |
| `surface_model.retention.max_normal_deg` | `45.0` | 各ノード法線とモデル法線の角度[deg]。符号反転は同一扱い |
| `surface_model.retention.max_rms` | `0.006` | 適合点全体のRMS[m] |
| `surface_model.retention.min_inlier_ratio` | `0.6` | 生成時の所属数に対する適合率。消失IDも不適合に算入 |
| `surface_model.retention.max_node_displacement` | `0.05` | 前回適合位置からの移動量[m]。ID再利用の誤対応対策 |

最小支持点数は既存の `surface_model.min_fit_nodes=12` を共用。
topic名・launch引数の変更なし。`/curved_surface_clusters/markers` を引き続き既定表示に使用。
`/curved_surface_clusters/models` のJSONに `retention_ms`、各モデルに `is_retained`、
`seed_plane_patch_num`、`rejected_node_num` を追加。`plane_patch_num` は現在の異なる元平面クラスタの数。
追跡IDは1073741824から生成順に採番し、維持中は不変。プロセスをまたぐ永続IDではない。
ログには `keep: ... ms` と `retained=...` を追加。

## Verification

コンテナ `gng_cpu_container` 内でReleaseビルド成功。曲面34件、平面21件、非平面2件の計57テスト成功。
既知の直角境界をまたぐ維持候補の棄却、正しい2平面の円柱統合、壁の吸収抑止も回帰確認。

```bash
colcon build --packages-select ais_gng --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
ctest --test-dir /ros2_ws/build/ais_gng \
  -R 'test_surface_model|test_plane_cluster_incremental|test_nonplane' --output-on-failure
ROS_DOMAIN_ID=197 python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_surface_models.py \
  --template /datasets/mug_gng_template.json.gz --seconds 9 --disconnect-after 3 --launch --graph
```

OFF比較は同コマンドに `--params-file /tmp/surface_retention_off.yaml` を追加。
一時YAMLは `/**.ros__parameters.surface_model.retention.enable: false` のみの上書き。
内部の起動コマンドは `ros2 launch ais_gng surface_models.launch.py input_topic:=<prefix>/map plane_clusters_topic:=<prefix>/planes output_topic:=<prefix> enable_markers:=true enable_graph:=true`。
prefixは `/surface_model_check_<observer PID>`。domain 197で既存bag・Viewerの通信から分離。

| 入力・条件 | 切断後の表示更新数 | 表示ノード | 切断後の処理平均 | 維持判定平均 |
| --- | --- | --- | --- | --- |
| mug 502ノード、維持OFF | 0/10 | 0 | 0.338ms | 0ms |
| 同じmug、維持ON | 10/10 | 71 | 0.409ms | 0.046ms |
| 保存semantic入力 837ノード、維持ON | 8/8 | 110 | 0.254ms | 0.049ms |

mugは15更新、semanticは13更新を計測。最初の3秒は元edgeで生成、その後は位置・法線を固定して全edgeを削除。
mugは元の79ノードから71ノードへ、semantic円柱は114ノードから110ノードへ、維持条件で所属を選別。
Marker・任意有効化したTopologicalMap・JSONの所属一致、ノードの重複所属なし、全有効ノードの被覆、ノードとedgeの共通色を確認。
semanticは以前の `/tmp/curved_semantic_before.json` のmap/planesをROSメッセージへ復元して同じ検証器に入力。
これは固定した実データへの接続切れ試験であり、実行中GNGの長時間変動試験や形状分類精度の評価ではない。
時間は上位クラスタ処理のみ。維持判定平均は全更新を対象とし、ROS転送・JSON化・Marker生成・ブラウザ描画は含まない。

検証launch 3回はSIGINTで正常停止。自分が起動したノード・ROSデーモンの残存なしを確認。
既存bagとViewerは停止・再起動していない。テスト開始時点ではユーザーのGNGは停止済み。

その後ユーザーが起動したsemantic_points用GNGを10秒間、読み取り専用で追加観測。
frame 424〜646の18更新すべてでID 1073741826を維持し、表示所属数は21〜29ノードで更新。
`keep` は0.016〜0.034ms。短時間の継続確認であり、長時間安定性や形状の正しさの保証ではない。
観測は `docker exec -i gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout --signal=INT --kill-after=5s 18s python3 -'` 内のrclpy購読で実施。
観測ノードは終了済み。作業中にユーザーが起動・再起動したGNGとHTTPサーバーには停止操作なし。
今回の一時比較YAML・ラッパー・出力JSON・検証launchログは、結果記録後に削除済み。

## Risk / Notes

- 維持は過去モデルへの適合性の判定で、正しい物体や正しい曲面種別という保証ではない。誤生成されたモデルも条件内なら残る。
- GNG IDには世代UUIDがないため、5cmの位置ゲートと曲面・法線条件だけでは近接したID再利用を完全には識別できない。
- 曲面距離は一次近似であり、厳密なSDF距離ではない。
- 過去の所属候補だけを扱うため、異なるIDへの大幅な置換や物体移動に対する追跡改善は別作業。
- 維持中も現在のsharp境界は棄却要因。元の平面・法線の誤推定による解除の可能性は残る。
- ブラウザでの目視比較は未実施。今回はC++とROS Marker出力を検証。
