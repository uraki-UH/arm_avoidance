# 2026-09-11 - 上方把持候補の非平面付属部分評価

## Summary

平面クラスタを把持シードとして維持し、局所非平面ノードの開口包含と上方観測障害物で候補を選別。

## Changed

- 法線とup_axisの内積絶対値による傾斜角判定。既定25度、法線符号反転を許容。
- 同じ平面だけに接続する非平面成分を、距離制限付きDijkstraで探索。
- 付属ノードの開口外へのはみ出し、全グラフの上方矩形柱内の観測ノードによる棄却。

## Added

- 付属ノード添字・成分数・付属込み局所外形を推定結果へ追加。
- summaryに`attached_node_num`、`attached_component_num`、`target_extent_x/y`を追加。
- `rejected_surface_tilt`、`rejected_attached_oversize`、`rejected_approach_obstacle`をsummaryとログへ追加。

## Fixed

- 水平投影が小さい壁面を上方把持面として採用するケース。
- 接続のない上方障害物を、平面間の隣接判定だけでは見落とすケース。

## Removed

なし。ROSメッセージ、既存トピック、平面OBBの定義は維持。

## Behavior Impact

同日、ユーザー指示により`ToPoDualArm.yaml`を許容角90度・非平面付属判定OFF・上方障害物判定OFFへ変更。
旧方式に近い構成への復帰であり、ゼロ・不正法線の除外は維持。C++既定値と追加方式の実装は維持し、結合テストでは追加判定を明示的に有効化。
既存プロセスへの動的反映なし、推定器の再起動が必要。
平面OBB・TCPを非平面成分へ合わせて拡大・移動する処理はなし。取っ手が上方許容幅を超える場合はそのシードを棄却。
グリッパ体積graphの`required_occupied`等との接続は未実装。実形状の指・手首衝突ではなく、粗い候補選別。
到達性状態の共通配信と計画側のno-motion動作は変更なし。

## Topics / Params / Messages

出力は`/grasp_pose_cands`の`GraspCandidateArray`を維持。新しい可視化トピックの追加なし。
上方推定器と`ToPoDualArm.yaml`に以下を追加。距離はm、角度はdeg。

| パラメータ | C++既定値 | 用途 |
| --- | --- | --- |
| max_surface_tilt_deg | 25.0 | 上方向に対する平面傾斜角 |
| enable_nonplane_attachment | true | 非平面付属探索と開口包含 |
| nonplane_margin | 0.03 | 平面OBB外側の探索余白 |
| max_nonplane_graph_dist | 0.15 | 平面からの累積エッジ長 |
| max_nonplane_depth | 0.08 | 平面最高位置から下方の付属探索幅 |
| max_nonplane_height | 0.01 | 平面最高位置から上方の付属探索幅・進入障害物の接触許容幅 |
| enable_approach_check | true | 上方矩形柱内の観測ノード確認 |
| approach_height | 0.10 | 平面最高位置からの進入領域高さ。正のtcp_standoffを加算 |
| approach_margin | 0.01 | 開口XY寸法に対する片側外側余白 |

## Verification

Docker `gng_cpu_container`内でビルド・合成入力検証。以下は`source /ros2_ws/install/setup.bash`後のコマンド。

```bash
cd /ros2_ws
timeout 180 colcon build --packages-select grasping_system --symlink-install --executor sequential
timeout 30 ctest --test-dir /ros2_ws/build/grasping_system --output-on-failure
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15 90 \
  python3 /ros2_ws/src/grasping_system/test/check_top_grasp_topic_integration.py
```

単体テスト: 付属抽出・OBB/TCP不変・包含棄却・探索上限・自由空間境界・橋渡し・未分類ノード・上方障害物・法線符号・フレーム整合・座標変換。
結合テスト: 標準/個別トピックの実launchで、付属数と候補生成、障害物による空配信、除去後の復帰。
テストは独立ドメインで実行し、起動launchとテストROSノードを停止。既存の実運用ノードの停止・再起動なし。

## Risk / Notes

- 入力は現在の`ais_gng_cpu/src/ais_gng_msgs`の非平面成分・境界フィールドを使用。GNG側のロジック変更なし。
- 複数平面への接続成分は保守的に付属対象外。同一物体の側面への接続も除外され得る。
- 成分IDはフレーム内だけで有効。探索距離や局所領域外のノードを同一物体と確定する処理なし。
- ノードがない場所を自由空間と証明する処理なし。エッジ途中・未観測の衝突、両指接触、実把持成功は保証外。
- 進入領域は矩形柱のため、実形状では空隙になる部分も保守的に棄却され得る。
- 候補ごとの全ノード走査と局所Dijkstraの追加コスト。計算量は詳細資料に記載、実点群上の性能測定は未実施。
- 入力停止時の旧候補失効、候補の時間追跡は従来どおり未実装。
