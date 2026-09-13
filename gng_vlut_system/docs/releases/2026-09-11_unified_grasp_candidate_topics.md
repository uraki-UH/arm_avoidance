# 2026-09-11 - 上方把持候補トピックの共通化

## Summary

上方把持方式の既定出力をボクセル方式と共通化し、`grasp_candidate_joint_planning.launch.py`の既定入力へ接続。

## Changed

- 上方把持C++ノード、launch、`ToPoDualArm.yaml`の候補・スコア・summary既定出力の統一。
- 上方把持launchのMarker既定出力を`/grasp_pose_markers`へ変更。
- 起動ガイド、概要資料、アルゴリズム資料、技術仕様の現行トピック更新。

## Added

- `grasping_system/test/check_top_grasp_topic_integration.py`による合成平面のROS結合確認。
- 共通出力およびYAML・launch引数を揃えた個別出力の2ケース。

## Fixed

- 上方把持出力と`grasp_candidate_joint_planning.launch.py`既定入力の名称不一致。

## Removed

- 上方方式の旧`/top_grasp_pose_*`既定出力。旧トピックへの互換二重配信はなし。

## Behavior Impact

- 候補推定アルゴリズム、姿勢、スコアの定義、メッセージ型は変更なし。
- `enable_motion:=false`の意味は変更なし。動作指令は抑制、経路計算は有効。
- ロボットGNG・TF・候補生成の起動前提は従来どおり。姿勢実現性や計画成功の追加保証はなし。
- 上方方式とボクセル方式の同時配信は混在の原因。共通出力を使う候補生成は一方式のみ起動。

## Topics / Params / Messages

| 出力 | 既定トピック | 型 |
| --- | --- | --- |
| 候補姿勢 | `/grasp_pose_cands` | `geometry_msgs/msg/PoseArray` |
| 面積比 | `/grasp_pose_cand_scores` | `std_msgs/msg/Float32MultiArray` |
| 判定概要 | `/grasp_pose_cands/summary` | `std_msgs/msg/String` |
| Marker | `/grasp_pose_markers` | `visualization_msgs/msg/MarkerArray` |

既存の`candidate_topic`、`score_topic`、`summary_topic`、`marker_topic`による個別設定は維持。
名前付きYAMLの出力値がlaunchのパラメータ辞書より優先される既存挙動があるため、比較用の個別出力ではYAMLとlaunch引数の両方を同じ値へ設定。

## Verification

Dockerの`gng_cpu_container`内で、`/ros2_ws/install/setup.bash`の読み込み後に実行。

```bash
cd /ros2_ws
colcon build --packages-select grasping_system --executor sequential --cmake-args -DBUILD_TESTING=ON
ctest --test-dir /ros2_ws/build/grasping_system --output-on-failure
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15 90 \
  python3 /ros2_ws/src/grasping_system/test/check_top_grasp_topic_integration.py
```

- ビルド成功、既存単体テスト4件成功。
- 合成平面4ノードから候補1件、対応スコア、summary、Markerの受信確認。
- 共通トピックと`/topic_contract`配下の個別トピックの両ケース成功。
- 最初の個別指定試験ではlaunch引数だけを変更し、名前付きYAMLの優先によって候補を受信できず時間超過。YAMLも整合させた再試験で成功。
- インストール先の`ToPoDualArm.yaml`が更新済みソースへのリンクであることを確認。
- 隔離ドメインの検証用launch・子ノードは全停止済み。テスト用一時YAMLは削除済み。
- 計画ノードを含む実環境のend-to-end動作、実機把持は今回の検証対象外。

## Risk / Notes

- 旧トピックを明示購読するViewer設定・bag設定などは移行が必要。
- 起動済みノードの既定値は自動更新されないため、切り替え時は候補生成launchの再起動が必要。
- 自動的な発行者排他や候補統合は未実装。
- 既存ユーザープロセスの停止・再起動操作は未実施。検証中に外部で変更された稼働状態への復元操作もなし。
