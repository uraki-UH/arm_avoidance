# 2026-09-14 - 非平面付属抽出の参照平面方式への一本化

## 1. 要約

不要な設定を減らす依頼に基づき、付属抽出の2つの切替設定と旧方式を削除。

- 参照平面からの符号付き離隔と入口エッジ長による付属探索を通常動作として固定。
- [上方把持仕様](../../../grasping_system/docs/top_grasp_surface_estimation.md)とスライドの設定依存表記を更新。

- 参照面の有無、未分類ノード、座標変換、開口包含の回帰確認。

- 結合テストの候補ID固定前提を、配信IDと状態の対応確認へ変更。
- テスト入力の更新間に出力を受信し、depth 1による未確定出力の上書きを抑制。

**削除**

- `enable_reference_plane_attachment`、`enable_nonplane_attachment`の設定宣言・YAML項目・条件分岐。
- 旧方式専用の非平面成分所有者マップと複合候補での再集計。

## 2. 条件・検証

`ToPoDualArm.yaml`で選ばれていた参照平面方式の挙動を維持。参照面がない場合は付属抽出を省略し、他条件を満たす平面候補を維持。平面OBBとTCPの算出は変更なし。

旧方式を選んでいた独自設定と、C++既定値だけでの利用は挙動変更。上方障害物チェック、平面組合せ、寸法や角度の設定は今回の変更対象外。

上記2つのROSパラメータを廃止。トピック・メッセージの変更なし。独自YAMLからも廃止項目の削除が必要。

Docker内でビルド、C++単体テスト、標準出力・出力先変更・入力座標系の3通りのROS結合テストに成功。
結合テストで付属抽出、候補の確定、障害物による棄却と復帰、到達性色、TF欠落時の出力消去を確認。
初回の候補ID前提不一致と次回の未確定出力取りこぼしは、上記テスト修正後の全ケース再実行で解消。

実行コマンド（すべて終了済み）:

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 180s cmake --build /ros2_ws/build/grasping_system --target top_grasp_surface_estimator_node test_top_grasp_surface_estimator -j2'
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 60s ctest --test-dir /ros2_ws/build/grasping_system -R "^top_grasp_surface_estimator$" --output-on-failure'
docker exec -e ROS_DOMAIN_ID=117 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 150s python3 /ros2_ws/src/grasping_system/test/check_top_grasp_topic_integration.py'
```

**制約**

互換用の旧方式やパラメータ別名は追加なし。稼働中の推定器は再起動せず、変更の反映は次回起動時。検証用launchと子ノードの残留なし、既存ROSプロセスのPID維持を確認。実環境での把持成功率・実機動作は未検証。
