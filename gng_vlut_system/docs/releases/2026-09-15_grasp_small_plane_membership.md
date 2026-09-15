# 2026-09-15 - 接続する小平面・側面の候補所属

## Summary / Changed

平面組合せの起点条件と合算対象条件を分離。直接または非平面経由で接続し、合算寸法が開口に収まる小平面・側面を複合候補へ追加。最低ノード数・傾斜角の起点条件、単独候補の併存、参照面・付属探索の制限は維持。投影外形の重複計算を除去。

## Behavior Impact / Topics / Params / Messages

トピック・パラメータ追加なし。既存`enable_plane_combinations=true`で適用。現行ToPoDualArmの合算寸法は0.15 m × 0.15 m。出力`source_cluster_ids`・候補Tmapの所属が増える場合あり。稼働中ノードには次回launch起動から反映。

## Verification

DockerのReleaseビルド、既存・追加C++検証、CTestに成功。3ノードの小平面と起点傾斜条件外の側面、非平面経由の候補Tmap接続、合算サイズ超過・未接続・自由空間境界・不正座標・組合せOFFを検証。

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 240s cmake --build /ros2_ws/build/grasping_system --target test_top_grasp_surface_estimator top_grasp_surface_estimator_node -j2 && timeout -s INT -k 5s 60s /ros2_ws/build/grasping_system/test_top_grasp_surface_estimator'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 120s cmake --build /ros2_ws/build/grasping_system --target test_top_grasp_surface_estimator -j2 && timeout -s INT -k 5s 60s ctest --test-dir /ros2_ws/build/grasping_system --output-on-failure -R "^top_grasp_surface_estimator$"'
```

全コマンド終了済み。インストール先から更新済みビルドへのsymlink参照を確認。ROSノードの追加起動・既存プロセスの停止や再起動なし。

## Risk / Notes

実入力・実機把持・処理時間の検証なし。サイズ適合は把持成立の保証ではない。参照面なしの場合は従来どおり非平面付属抽出を省略。候補数上限・時系列確認・付属込みの開口包含等による棄却は維持。
