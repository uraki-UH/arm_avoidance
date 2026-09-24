# 2026-09-24 - 共通空間索引のbsp3dへの統一

> 同日追記: 本文は移行直後の比較。現在は目標選択の全体走査削減と位置索引の差分更新を実装済み。[新しい仕様と比較](2026-09-24_goal_selection_efficiency.md)を参照。座標変更時の全再構築はbsp3dの必須動作ではなく、当時の呼出し側実装。

## Summary

目標選択が使用していたSpatialTreeをbsp3dへ移行し、共通の旧8分木実装を削除。選択結果の一致と通常Releaseビルドを確認。

## Changed

- 目標選択の索引を`bsp3d::Index<entry, double>`へ変更。元座標の精度・TF逆変換・元セル判定・同点順・索引再利用を維持。
- 独立GNG比較パッケージはgrid・bsp3dの2方式へ整理。既存スクリプト用にディレクトリ名`gng_spatial_tree`は維持。
- bsp3d単一ヘッダを保守の正本として明示。内部の`SpatialTree::`名前空間は互換用に維持。

## Added

- 閉区間AABB検索`query_aabb`とdouble用の公開型指定。
- 動的更新・境界・非有限値の範囲検索回帰試験、移行前後の速度・結果照合。

## Fixed

- `benchmarks/COLCON_IGNORE`で計測用`setup.py`をcolconのパッケージ探索から除外。
- bsp3dのCMakeから`-ffast-math`を除き、非有限値入力の検査を維持。

## Removed

- 共通`SpatialTree/`の管理対象22ファイル、GNG実験版の旧8分木ビルド・実装。
- bsp3d内の重複`AdaptiveTree`と、削除した共通ソースに依存する`tools/amalgamate.py`。
- 過去の比較結果は維持。旧方式の再現はGit履歴または保存済み比較ライブラリが必要。

## Behavior Impact

範囲候補の列挙順は変更し得るが、検証した目標ID・出力mapは全走査基準と一致。実GNG18,729点の範囲検索1024回は0.664→0.461 ms、初期構築は1.371→4.420 ms。目標選択全体は約0.075〜0.116 ms増加し、全体高速化としての扱いなし。座標不変時の再構築不要という挙動を維持。

## Topics / Params / Messages

ROSトピック・パラメータ・メッセージ変更なし。元CPU GNGの学習ライブラリ・YAML・起動先の差替えなし。内部の旧`AdaptiveTree`を直接使う外部コードには移行が必要。

## Verification

- bsp3d CTest 2/2、AABB 37,600比較、ASan/UBSan、GNG独立版CTest 18/18成功。
- 通常`gng_vlut_system`のReleaseビルド・インストールと目標選択のGTest12件成功。
- 3試行、2種類の入力、候補数3条件、旧・新それぞれの全走査照合7,920組が一致。
- [全条件・実測表・実行コマンド](../../../benchmarks/bsp3d_migration_20260924/README.md)。生ログは`artifacts/bsp3d_migration_20260924/`。

## Risk / Notes

静的索引では構築費を再利用できるが、座標が毎回変化する用途では構築増加に注意。今回の一括構築試作はさらに遅く、不採用。旧8分木の仕組みを追加移植する速度上の根拠はなし。実ROSの目標選択を新規起動した再生試験は未実施で、検証は実座標入力の単体比較・回帰・通常ビルドの範囲。

検証で起動したプロセスは全終了、専用一時ビルドは削除。既存ROS・bag13プロセス、3コンテナの状態を維持。
