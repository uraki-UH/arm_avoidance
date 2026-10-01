# 2026-10-01 - 全身形状による自己干渉検査

変更:

- 対象: `topo_dual_arm_max` / `topo_dual_arm_max_long` の通常学習・到達域生成。
- メッシュ判定: 元三角形のFCL表面交差と、1 mm占有データによる完全内包検査の併用。非退化な連結表面成分ごとの元頂点を使用した双方向検査。
- URDF外装: max 11リンク、long 13リンクへ表示形状と同じ `collision` を追加。表示形状のあるリンクでの欠落は両モデル0件。
- カメラ: 開いたメッシュを包含する衝突用BOXへ変更。既存checkerのBOX各面2 mm膨張も適用。
- 除外規則: 同一固定剛体内、可動関節を挟む形状付き基幹リンク対、設定の明示ペアに限定。祖父母・兄弟・初期接触を理由とする一律除外の廃止。
- 高速化: 幾何と両絶対姿勢が完全一致するペアの表面判定結果を再利用。姿勢の量子化なし。球形状登録時の回転未初期化も修正。
- 検査不能時: STL欠落・破損・開面・リンク変換欠落のエラー化。面積ゼロ面は閉面の辺収支だけに保持し、内包検査用の成分統合から除外。
- GNG辺: 角度層と左右TCP層の全層で、端点と最大関節刻み 0.025 rad の補間姿勢を検査。座標辺生成後の最終フィルタを追加。

検証:

- 最終方式の統合回帰: 6対象49件成功。表面判定だけで見逃す完全内包、分離成分、開放空洞、回転・並進原点を含む試験構成。
- 通常launch: max / longの2件成功。上限32ノード・1000反復で実際6 / 7ノードを保存し、全身表面交差＋1 mm内包判定と座標辺生成後の最終フィルタを確認。保存全6 / 7ノードの別ツール再監査も全件合格。[学習結果](../../../artifacts/gng_self_collision_fix_20261001/training_smoke_training_summary.json)、[再監査](../../../artifacts/gng_self_collision_fix_20261001/training_smoke_audit_summary.json)、[起動コマンド](../../../artifacts/gng_self_collision_fix_20261001/training_smoke_commands.jsonl)。
- 上記smokeの終了確認: 学習2件・再監査2件の所有プロセス残存0、既存コンテナ状態の一致、新規ROS 2 daemonなし。[前後照合](../../../artifacts/gng_self_collision_fix_20261001/training_smoke_cleanup.json)。大モデルの全体cleanupとは別集計。
- 保存姿勢の全件監査: max 19,242姿勢中123、long 24,302姿勢中374を自己干渉として棄却。関節限界違反0件。[全件集計](../../../artifacts/gng_self_collision_fix_20261001/full_audit_summary.json)。棄却は両モデルとも旧補完姿勢だけで、元の1万姿勢は全件合格。
- maxの再構成: 19,139姿勢、各層38,162辺、連結成分1・孤立0。安全参照13,727点の2 cm被覆100%、独立参照19,167点の3 cm被覆99.51%。VLUT参照集合とViewerのノード・辺・角度・TCPが一致。[最終集約](../../../artifacts/gng_self_collision_fix_20261001/max_postprocess_summary.json)。
- longの再構成: 24,006姿勢、各層47,857辺、連結成分1・孤立0。安全参照21,020点の2 cm被覆100%、独立参照19,127点の3 cm被覆99.20%。VLUT参照集合とViewerのノード・辺・角度・TCPが一致。[最終集約](../../../artifacts/gng_self_collision_fix_20261001/long_postprocess_summary.json)。
- 検証範囲: 到達マップの新規再生成は対象外で、旧参照姿勢の安全性を全件再監査。ViewerはROS配信内容の照合であり、ブラウザ描画の目視検査は対象外。

終了・保全: 所有試験全終了、開始時の既存20プロセス・3コンテナを維持。旧GNG/VLUT 8ファイルとSTL 174ファイルのSHA・サイズ一致。[両モデル最終集約](../../../artifacts/gng_self_collision_fix_20261001/final_summary.json)、[起動コマンドと終了記録](../../../artifacts/gng_self_collision_fix_20261001/execution_commands.json)、[表示コマンド](../../../benchmarks/gng_self_collision_fix_20261001/README.md#修正版の表示)。

設定移行: `collision.voxel_size: 0.001` を使用。通常2モデルの設定と、trainer・到達域生成の互換項目の既定値も1 mm。旧 `collision.voxel_ball.voxel_size` は新項目未指定時の値の引継ぎのみ。依存は `libfcl-dev` / `octomap` を明示。

条件: 旧辺を検査済み近傍辺へ再構成したため、角度層の接続は以前より疎。旧グラフとの経路長・探索性能の同等性は未検証。腰・首・グリッパー0固定の14関節モデル。辺は離散検査であり、連続時間の非干渉証明は対象外。内包のセル近似、BOX膨張、カプセル近似による判定余裕への影響あり。元メッシュ表面の接触判定と占有木どうしの直接衝突判定は別条件。保存済みフラグは生成時の判定条件に依存。

詳細・再現・先行測定の条件: [自己干渉修正の検証手順](../../../benchmarks/gng_self_collision_fix_20261001/README.md)。
