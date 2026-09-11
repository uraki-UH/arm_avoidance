# 2026-09-11 - 採用環境ノードの到達性色分け

## Summary

`/grasp_pose_cands/nodes`の色を、同じ候補IDの`GraspCandidate.state`と同期。

## Changed

- HANDLE既定色の水色`#00d1ff`: INSIDE、TCP位置が登録済み到達セル内。
- 従来の青系候補色（線形RGB `[0.1, 0.85, 1.0]`）: OUTSIDE・UNKNOWN、範囲外または未評価。
- 平面・非平面付属ノードの両方に同じ候補の状態を適用。

## Added

共通`grasp_candidate_publisher`に省略可能な配信通知コールバックを追加。通常の候補配信と状態更新の両方で通知。
上方推定器はノード位置を保持し、状態だけの更新では再抽出せず色を更新。

## Fixed

候補の到達性が変化しても対象ノードの色が変わらない表示上の不足。

## Removed

オレンジによる付属種別の配色。namespaceによる平面・非平面の区別は維持。

## Behavior Impact

把持判定アルゴリズム・閾値・学習パラメータの変更なし。no-motion動作の変更なし。
候補矢印の状態と同期し、ノードの配色はユーザー指定の水色・青系。矢印側の配色は変更なし。
状態のみの変更では候補ID・更新番号・対象ノードの位置を維持。
候補ゼロ・TF欠落時のDELETEALL、遅延購読用transient_localを維持。

## Topics / Params / Messages

新規トピック・パラメータ・ROSメッセージ構造の追加なし。`/grasp_pose_cands/nodes`のMarker色のみ変更。
既存の`reachability_map_topic`等で到達評価を設定。ビルド後は把持推定器の再起動が必要。

## Verification

Docker `gng_cpu_container`内でビルド、CTest5件と実launch結合テストに成功。

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
timeout 180 colcon build --packages-select grasping_system --symlink-install --executor sequential
timeout 30 ctest --test-dir /ros2_ws/build/grasping_system --output-on-failure
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15 100 \
  python3 /ros2_ws/src/grasping_system/test/check_top_grasp_topic_integration.py
```

未評価、到達mapだけによるHANDLE色・候補色への更新、位置・ID・更新番号の保持、異なる候補IDの2色同時表示を検証。
Three.jsのColorで、Marker用線形RGBとHANDLE既定色`#00d1ff`の一致を確認。
標準/個別トピック、候補座標系の変換有無、障害物やTF欠落時の消去を検証。
検証で起動したlaunch・ROSノードは停止。既存プロセスの停止・再起動なし。

## Risk / Notes

- HANDLE色はTCP位置の到達範囲であり、姿勢IK成立・衝突回避・両指接触・把持成功の保証ではない。
- GUIで変更したHANDLEラベル色への自動追従は対象外。既定色との一致。
- ノード自体を個別に到達判定する処理ではなく、所属候補の状態の表示。
- 実ブラウザ画面の確認は未実施。既存MarkerArray描画経路へのROS出力を検証。
