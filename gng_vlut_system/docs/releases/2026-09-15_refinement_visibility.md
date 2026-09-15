# 2026-09-15 - 把持補正の待機・棄却理由表示

## Summary

補正結果が届いていても、全候補棄却時の理由が通常ログとMarkerから分からない問題を修正。

## Changed

- 入力トピック名と局所点数上限を起動ログへ表示。
- 候補未受信、候補数・接触対数・IK成立数、理由別件数をINFOへ表示。状態変化時のみ最短5秒間隔で更新し、入力停止後の最終状態も対象。
- 幅未計算の候補も元位置に理由付きテキストMarkerを表示。不正な非有限位置は描画対象外。

## Added

未受信ログ、点群未受信・局所点数超過の理由Marker、ログ間隔の結合検証。

## Fixed

起動INFO以外はDEBUGのみで、幅未計算候補のMarkerを省略していたための診断情報不足。

## Removed

なし。

## Behavior Impact

`/grasp_pose_refined/markers`で棄却候補の理由も確認可能。幅・接触線の捏造や、棄却を成立扱いにする変更なし。
局所点数の既定30,000、接触条件、74 mmの最大開口、IK・衝突判定、候補選択は変更なし。
動作中の既存ノードは再起動せず維持。反映には既存launchの再起動が必要。

```bash
ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py
```

## Topics / Params / Messages

追加・形式変更なし。既存`candidate_goal_preview`への補正ロボット表示の統合なし。

## Verification

### 実行中ノードの調査

既存候補・実点群・関節候補を有限購読で確認。点群は約213,000点、候補は観測時4件。
`/grasp_pose_refined`は約2 Hzで配信されていたが、3件が`local_point_budget`、1件が
`insufficient_contact_support`で接触対・IK成立0件。旧Markerは幅を観測できた1件のテキストのみ。
起動ログで止まっていたのではなく、結果の棄却理由がログに出ていない状態。

### ビルド・結合検証

DockerでReleaseビルドと幾何テスト8件に成功。隔離domain 218で40/46 mmの把持幅・開口、
IK補正、従来入力への非干渉・失効・空配信処理に加え、未受信ログ、未計算理由Marker、
接触線を出さないこと、状態ログの5秒間隔を確認。

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 240s cmake --build /ros2_ws/build/gng_vlut_system --target grasp_candidate_refiner_node test_grasp_refinement -j2 && timeout -s INT -k 5s 60s ctest --test-dir /ros2_ws/build/gng_vlut_system --output-on-failure -R "^test_grasp_refinement$"'
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/tmp/refinement_visibility_20260915_logs gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 20s 120s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_refinement.py --output /ros2_ws/src/tmp/refinement_visibility_20260915/integration.json'
```

結合テストがdomain 218で起動した子launch:

```bash
ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py params_file:=/tmp/grasp_refinement_vc5dprdu/params.yaml candidate_topic:=/grasp_refinement_test/source seed_topic:=/grasp_refinement_test/seeds point_cloud_topic:=/grasp_refinement_test/points output_topic:=/grasp_refinement_test/result
```

### 実入力での別出力検証

入力・TF・既存出力を変更せず、以下の別名・別出力で各9秒の有限検証。
各条件17更新を受信。30,000点では従来と同様の棄却、100,000点では点数上限の棄却がなくなり、
支持不足・非対向接触・開口超過まで評価されたが、接触対・IK成立は0件。
観測幅73.6 mmの候補も左右合計6 mmの開口余裕を含めると74 mmを超える。
更新中央値はそれぞれ5.24 ms・8.65 ms。入力候補が時系列で変わるため厳密な速度比較ではない。

```bash
/ros2_ws/build/gng_vlut_system/src/grasp_candidate_refiner_node --ros-args -r __node:=refinement_visibility_30000 --params-file /ros2_ws/src/gng_vlut_system/config/grasp_candidate_refinement.yaml -p output_topic:=/refinement_visibility_20260915/result_30000 -p max_local_points:=30000
/ros2_ws/build/gng_vlut_system/src/grasp_candidate_refiner_node --ros-args -r __node:=refinement_visibility_100000 --params-file /ros2_ws/src/gng_vlut_system/config/grasp_candidate_refinement.yaml -p output_topic:=/refinement_visibility_20260915/result_100000 -p max_local_points:=100000
```

初回100,000点検証は非同期の最新結果と最新Markerを比較して件数照合に失敗。
プローブ側を同一header時刻の照合へ変更し再検証成功。受信メッセージ上で全候補の理由表示を確認。
起動直後のTF・点群未受信も理由として出力。Viewerの実画面・実機把持は未検証。

### 後片付け

有限購読プローブ、結合テスト、別出力ノード、ビルドは全終了。子launch PID 1548900、
別出力PID 1557028・1559616・1571046を停止済み。一時プローブと専用ROSログは削除。
既存補正PID 1443642とlaunch PID 1443581、その他既存ROS PID・コンテナの稼働を維持。
新規ROSデーモン・関節指令なし。結果は`tmp/refinement_visibility_20260915/`へ保存。

## Risk / Notes

- 接触条件の成立を実入力で確認できたわけではない。点数予算を増やすだけでは把持成功にならない。
- `insufficient_contact_support`は点数だけでなく面内広がり・平面性の不成立も含む。
- 理由テキストの追加で表示量が増える。状態が同じ間はINFOの繰り返しなし。
- 後続調査でViewerの本文欠落・文字描画未対応を確認し、[文字Marker対応](2026-09-15_viewer_text_markers.md)で修正。上記の検証はROS配信までの結果。
