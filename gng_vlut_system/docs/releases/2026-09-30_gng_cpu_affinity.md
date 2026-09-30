# 2026-09-30 - GNG区間のPコア配置の試作と撤去

- 状態: ユーザー指示による撤去済み。過去の試作・比較の履歴。
- 削除対象: GNG区間のPコア選択・マスク適用／復元、`enable_gng_cpu_affinity`設定・起動引数、専用テスト。
- 起動方法: 従来のROSコマンドを継続。試作用の`enable_gng_cpu_affinity:=...`指定は不要、現行機能としての受付なし。
- 判断理由: 平均・最大時間の改善は未確認。判断者・再検討条件は[不採用記録](../reject.md#2026-09-30-gng区間のpコア配置)。
- 過去の検証: 試作のReleaseビルド・単体8件・比較4回・通常ライブラリ起動成功。初回CTestのROS環境未読込による失敗は環境読込後に解消。
- 過去の試験終了: 全5回の試験launchと所有補助プロセス停止済み。既存bag・TF・Viewer維持。
- 撤去後検証: Docker Release再ビルド成功、稼働ソース・installライブラリ・設定・CTest登録の残留なし。新規ROS起動なし。
- ビルド: `CMAKE_BUILD_PARALLEL_LEVEL=2 timeout -s INT -k 20 600 colcon build --packages-select ais_gng --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON`、終了済み。[実行結果](../../../artifacts/gng_affinity_20260930/removal/build.log)。
- 保存資料: 比較ログ・集計・撤去前の対象ソース。現行仕様と独立した実験記録。

数値・比較条件・当時の全起動コマンド: [比較記録](../../../benchmarks/gng_affinity_20260930/README.md)。
