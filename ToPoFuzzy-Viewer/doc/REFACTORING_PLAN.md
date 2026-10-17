# Viewer整理計画

旧単一ViewerServerを対象にした計画。現行の分割ノード構成は[Architecture](ARCHITECTURE.md)、実装上の規約は[AGENTS.md](../AGENTS.md)。

| 旧計画 | 記録上の状態・扱い |
| --- | --- |
| PCL変換の共通化 | 完了。[共通変換](../backend/src/topo_fuzzy_viewer/src/common/pcl_converter.cpp) |
| PointCloudBridgeの削除 | 完了。旧ros2_bridge.cppの重複実装を削除 |
| ViewerServerの分割 | 旧計画では未着手。現行はROSノード分割構成のため、旧ファイル名・メソッド数を前提に再開しない |
| 名前空間の統一 | 旧提案。適用範囲・互換性の再評価が必要 |
| loadLAS・publishStaticCloudの整理 | 旧候補。現行の呼出し元・代替処理の確認後に判断 |

新機能の追加時は既存の責務境界を使用し、旧RPC名・仮のクラス構造を再導入しない。
