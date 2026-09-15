# 2026-09-15 - ロボット座標ROIとworld表示の整合

## Summary

入力の短いframe名をロボットframeへ誤って読み替える処理を削除。ロボット座標でのボクセル化とworld表示の位置関係を回帰検証。

## Fixed

`world_index_to_voxel_node::resolveSourceFrameId`の末尾一致処理を削除。
`base_link`は`ToPoDualArm/base_link`とは別frameであり、入力TFからworldへ変換後、逆ロボットTFでロボット座標へ変換してvoxel化。
Viewerは既存のframeIdとTFを使用してボクセル格子をworldへ回転・並進。Viewerソースの変更なし。

## Changed

`ToPoDualArm.yaml`の`environment_voxelization.enable_static_tf`をfalseへ変更。
外部TF運用ではURDFルート`ToPoDualArm/base_footprint`へ姿勢を与え、URDFの固定関節から`base_link`へ接続。

```text
world -> ToPoDualArm/base_footprint -> ToPoDualArm/base_link
            外部TF                        URDF固定関節
```

## Behavior Impact

- 格子軸はロボットに追従し、world表示ではyawに応じて斜めになる。環境点群とセル中心の差は量子化誤差の範囲。
- 短いframe名からロボットframeへの暗黙の別名解決なし。必要なら入力frame/TFを正しく指定するか、既存の`source_frame_id`を明示設定。
- `source_frame_id`による明示上書き、空frame/`map`の既存world扱い、TF不明時の`allow_unconnected_source_as_world`互換動作は変更なし。
- 未接続入力をworld扱いする互換動作はworld-index経路の既存仕様。直接ROI経路では必要なTFがない場合は出力なし。
- 外部TF未起動でロボットへの経路がなければROI出力不可。固定設置運用は外部TFを停止してYAMLの固定TFを有効化。
- 稼働中プロセスの再起動なし。旧固定TFを含むenvironment launchと旧TF配信はユーザー側で終了・切替が必要。

## Topics / Params / Messages

メッセージ・トピックの追加なし。ROIのframe_idは引き続きロボットのbase_link。
起動例と座標系の前提は[README](../../README.md#環境点群からvlutへの入力)を参照。

## Verification

修正前の実ノードで`base_link`入力がロボット座標のまま扱われる不具合を再現。修正後、既存単体テスト21件と隔離ROS回帰テストに成功。

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 20s 600s colcon build --packages-select gng_vlut_system --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON --cmake-target world_index_to_voxel_node && timeout -s INT -k 15s 300s cmake --build /ros2_ws/build/gng_vlut_system --target test_reachability_voxel_accumulator -j2 && timeout -s INT -k 5s 90s /ros2_ws/build/gng_vlut_system/test_reachability_voxel_accumulator'

docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 150s python3 /ros2_ws/src/gng_vlut_system/test/test_roi_frame_ros.py'
```

検証スクリプトはROS domain 223・Viewer port 19095で専用ノードを起動。子ROIの起動引数を`START`行へ出力し、Viewerは`viewer_ws_gateway_node --ros-args -p port:=19095`で起動。スクリプト内で配信するTFと点群は通常ROS環境から分離。

| 入力条件 | ROI経路 | 結果 |
| --- | --- | --- |
| `base_link`が未接続、world互換動作ON | world index | 成功 |
| `base_link`が別の非単位TFでworldへ接続 | world index | 成功 |
| 上記と同じ接続済み入力 | 直接ROI | 成功 |
| 明示的な`ToPoDualArm/base_link`入力 | world index | 成功 |

各条件でx=0.15/yaw=1.5とx=0.3/yaw=3.14を確認。Viewerから受信したロボット座標のvoxel IDが期待値と一致。セル中心をworldへ戻した位置は入力点から2 cm格子の半対角長（約1.73 cm）以内。セルはロボット座標で構築しており、world軸に再ボクセル化していない。

初回ビルドコマンドはcolconの複数ターゲット指定不可で失敗し、ターゲット別コマンドへ修正。テストはID不変時の差分省略を考慮してsnapshotを要求する形へ修正し、状態取得の`type`プロトコルへの修正後に全件成功。

専用ROI・Viewer・Pythonノードは全試行で停止済み。終了後のプロセス一覧で残留なしと既存ROI PID 1251375・GNG PID 1251223・Viewer PID 419809の維持を確認。ユーザー側のロボット表示・TF・bag操作によるプロセス変更を観測したが、本作業から既存プロセスへの停止操作なし。

## Risk / Notes

実ブラウザの目視確認は未実施。過去のTFがViewerに残る場合はTF運用の切替後にブラウザをリロード。
入力時刻が過去のbagでは既存の最新TFへの代替経路を使用。実際に移動するロボットと時間同期した点群での時刻精度検証は対象外。
