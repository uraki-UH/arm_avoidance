# 2026-09-16 - Tmap_staticのBBoxに基づくROI範囲

## Summary

ToPoDualArmのROI範囲を、学習用の固定範囲から`Tmap_static`の全ノードBBox＋各面20 cmへ変更。

## Changed

- `environment_to_vlut.launch.py`からマップtopicを伝達。単一ロボット・共有world indexのconsumer別設定に対応。
- 直接方式・world index検索方式とも同じ範囲を適用。マップが別frameの場合はROI座標へのTF変換後にBBoxを計算。

## Added

- `reachability_map_topic`パラメータと低レベル3launchの同名引数。
- マップ形状・TF不変時のBBox再利用と、範囲変更時だけのボクセル集約器再構築。
- `test/test_tmap_roi_ros.py`による隔離ROS回帰検証。

## Fixed

ROI範囲が配信中の静的マップではなく、学習用YAMLのサンプリング範囲に依存していた挙動。

## Removed

トピック・メッセージ・既存パラメータの削除なし。

## Behavior Impact

`ToPoDualArm.yaml`でBBox方式を有効化。マップとTFの準備までROI配信を待機する。
マップを取得できない場合に旧範囲で配信するフォールバックはない。
空文字列指定または項目なしの構成は従来の固定範囲を維持。ボクセル幅・ID・自己領域除去処理は変更なし。

## Topics / Params / Messages

```yaml
environment_voxelization:
  reachability_map_topic: "Tmap_static"
  reachability_margin_x: 0.2
  reachability_margin_y: 0.2
  reachability_margin_z: 0.2
```

相対topic名は`robot_name`で名前空間化。ToPoDualArmの入力は`/ToPoDualArm/Tmap_static`、
出力は既存の`/ToPoDualArm/roi_voxels`。余白はBBoxの各面からの距離[m]。
詳細は[現行仕様](../TECHNICAL_SPEC.md#17-roi範囲の静的マップbbox追従)。

## Verification

コンテナ内で対象ノードをビルド、既存単体テスト21件に成功。
ROS_DOMAIN_ID=224の隔離試験で、直接方式・world構築＋直接方式・world検索方式の3条件を検証。
マップ遅着、transient-localでの後起動購読、異なるframeの回転並進、各軸の余白、複数consumerの独立待機、
BBox拡縮、状態更新時の再構築省略、無効マップ／未接続TFの待機と復帰、マップ再配信なしのTF変更に成功。
単一／共有launchのtopic伝達と空文字列設定を評価し、world検索方式はlaunch経由でも起動検証。
初回試験ではテスト用JSONのROS引数引用不足で起動失敗。テスト側の引用を修正して再検証。

```bash
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cmake --build /ros2_ws/build/gng_vlut_system --target world_index_to_voxel_node test_reachability_voxel_accumulator -j2'
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && /ros2_ws/build/gng_vlut_system/test_reachability_voxel_accumulator'
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/gng_vlut_system/test/test_tmap_roi_ros.py'
```

## Risk / Notes

試験用ROSノードは全停止済み。既存の再生・Viewer・ROIノードの停止／再起動なし。
実環境の表示確認は未実施。反映には既存の`environment_to_vlut.launch.py`の再起動が必要。
対象マップのBBoxはロボット全体の掃引体積や衝突安全域の保証ではない。
マップ／TF喪失時の新規配信停止は、既存のtransient-local配信値を消去しない。
