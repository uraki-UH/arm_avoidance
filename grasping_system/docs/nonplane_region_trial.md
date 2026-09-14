# 非平面起点の領域抽出試験

既存の把持姿勢出力とは独立した、観測形状の試験表示。把持成立や到達性の保証なし。

既存ノードの設定に `enable_nonplane_region_trial: true` を追加して再起動するか、既存ノードとの二重起動を避けて以下を使用。

```bash
ros2 run grasping_system top_grasp_surface_estimator_node --ros-args \
  --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml \
  -p enable_nonplane_region_trial:=true
```

- 出力: `/nonplane_grasp_regions/nodes` (`visualization_msgs/msg/MarkerArray`)、オレンジ色。
- 入力: 通常経路と同一フレームのGNGグラフと平面クラスタ配列。平面配列が空でも抽出可能だが、配列メッセージ自体の受信は必要。
- 除外: 全平面の所属ノード、通常経路の当該フレームの出力候補ノード、非有限座標、FREE_SPACE境界ノード。
- 抽出: ノード配列順の起点から接続探索。固定投影方向の範囲を更新し、`grasp_size_x/y`を超えるノードは不採用・経由禁止。
- 抽出済みノードの重複使用なし。サイズ超過ノードは別領域の起点候補。`minimum_region_nodes`未満の領域は非表示。
- 方向: 候補座標系のXを`up_axis`直交面へ投影した固定方向。通常の上面把持では候補座標系のXY。
- 表示: 毎フレーム全置換、2秒の有効期限。IDの時間追跡なし。
- 計測: ノードログの`Nonplane trial`に領域数と抽出・配信時間。

試験対象外は、長いエッジの信頼性、高さ制限、巨大平面からの離隔、物体境界の確定、観測充足、衝突、IK。大きな物体の一部を切り出す可能性や、入力順による領域の変化あり。通常の把持候補への自動昇格なし。
