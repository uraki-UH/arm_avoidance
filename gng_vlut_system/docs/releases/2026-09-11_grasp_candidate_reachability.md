# 把持候補の保持と位置到達性による計画スキップ

`grasp_goal_planning.launch.py`の入口で、全把持候補の位置到達性を評価する。
領域外の候補も削除せず、元の姿勢・配列添字・形状スコアを含む評価結果を配信する。
計画には領域内候補と同じ登録セルにある非衝突GNGノードだけを渡す。

## 起動

ビルド後に`source /ros2_ws/install/setup.bash`を実行してから、従来のコマンドを使用する。

```bash
ros2 launch gng_vlut_system grasp_goal_planning.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  enable_motion:=false
```

既定の到達領域は`/ToPoDualArm/topological_map_static`にあるTCP位置を5 cmセルへ量子化した集合。
セル原点の既定値は各軸0 m。位置到達性は動的衝突ラベルから独立した情報であり、
領域内でも対応する全GNGノードが衝突状態なら計画対象は空となる。
登録セルなしはサンプリング上の未確認であり、連続空間での到達不能の証明ではない。
Safety VLUTのアーム本体占有セルをTCP到達領域として扱うものではない。

独立した到達可能ボクセルmapを使う場合は、そのセル中心を配信する`TopologicalMap`を
`reachability_map_topic`へ指定し、`reachability_voxel_size`と
`reachability_voxel_origin_x/y/z`を生成時のセル寸法・原点に合わせる。
可視化GNGへ再学習した疎なノードではなく、元の到達ボクセルのセル中心を入力する。
独立mapのノードIDは計画に使用せず、計画用GNGのTCPを同じ座標系・セルへ変換して照合する。
領域内でも対応する計画用GNGノードがない場合は、その旨を返して計画をスキップする。

## 出力

| トピック | 内容 |
| --- | --- |
| `/grasp_pose_cands/reachability` | `gng_control_msgs/GraspCandidateReachabilityArray`。入力と同じ順序・個数の評価 |
| `/grasp_pose_cands/reachability_markers` | 全候補のZ方向ベクトル。緑=領域内、灰=領域外、黄=未評価 |
| `/selected_goal_candidate_ids` | 計画対象となるGNGノードIDの重複を除いた集合 |
| `/ToPoDualArm/grasp_candidate_metrics` | 既存のGNG目標ごとの計画結果・コスト指標 |

評価配列の`header`は元の入力、`evaluation_header`は現在配置での評価時刻・座標系。
`candidate_idx`は当該入力配列内の添字であり、別フレーム間での物体追跡IDではない。
`pose`は元の把持姿勢、`evaluated_pose`は到達map座標系へ変換した姿勢。
`shape_score`は対応スコア配列の値。未受信または個数不一致の場合はNaN。
既存スコア入力にはheaderがないため、同数の別フレームとの厳密な時刻同期は対象外。

| `reason` | 意味 |
| --- | --- |
| `outside_skip_planning` | 登録セル外。候補を保持し計画をスキップ |
| `inside_plan_eligible` | 登録セル内かつ計画対象ノードあり |
| `inside_no_goal_nodes` | 登録セル内だが非衝突の対応GNGノードなし |
| `inside_unknown_planning_transform` | 登録セル内だが計画用座標系へのTFなし |
| `unknown_map` | 到達map未受信 |
| `unknown_transform` | 座標系不明または到達mapへのTFなし |
| `invalid_pose` | 非有限値または無効な姿勢 |

`has_evaluation`がfalseの場合は内外未評価。`can_plan`は計画対象ノードの有無であり、
経路成立を表さない。`goal_node_ids`で既存の計画指標の`goal_node_id`へ対応付ける。
計画実施後の成否は既存指標の`feasible`を参照する。
方向到達性・接触成立・実機把持成功は今回の位置領域判定の対象外。

## 再評価と旧計画の扱い

- 入力候補、到達map、計画用map、特徴量の更新時に再評価。
- 既定5 Hzの周期処理による、候補再受信なしでの最新TFを用いた再評価。
- 台車移動後も同じ物体候補を利用する場合は、候補をworld等の固定座標系で保持することが前提。
- 目標ID集合の変更時に旧経路・旧評価を失効させ、新しい集合で再計画。
- 全領域外・空入力・TF取得失敗時には目標IDを空配列として配信。
- 採用経路トピックには既存仕様により現在手先の仮想ノードだけが残る場合あり。旧経路のエッジは消去。
- `candidate_count`は各把持候補に対応付けるGNGノード数。既定値8。
- `enable_motion:=false`でも領域内候補の経路計算は有効。関節目標の配信・仮想関節駆動は無効。

## 検証

- `gng_control_msgs`、`gng_vlut_system`のビルド。
- `test_grasp_candidate_reachability.py`による回帰テスト11件。
  混在候補の保持、全領域外、空入力、TF欠落、台車移動相当のTF変更、負座標境界、
  衝突状態、独立mapのID分離、map更新、不正入力。
- `check_grasp_goal_planning_integration.py`による実GNG・VLUTを用いた結合確認。
  混在入力→全領域外→領域内復帰→空入力、旧経路・旧評価の失効、関節目標配信なし。
  結合確認の現在関節角は既定のゼロ姿勢。実機での把持・経路安全性検証は未実施。

```bash
ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 \
  python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_goal_planning_integration.py
```

結合確認スクリプトは隔離ドメイン内で次のコマンドを起動し、終了時に両方を停止する。

```bash
ros2 run gng_vlut_system safety_monitor_node --ros-args \
  --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  -p gng_model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin \
  -p vlut_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vlut.bin \
  -p base_frame:=ToPoDualArm/base_link \
  -r topological_map:=/ToPoDualArm/topological_map_static

ros2 launch gng_vlut_system grasp_goal_planning.launch.py \
  params_file:=/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  enable_motion:=false
```
