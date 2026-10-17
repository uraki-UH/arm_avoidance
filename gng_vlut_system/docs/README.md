# Documentation Index

このディレクトリは `gng_vlut_system` の運用仕様と変更履歴をまとめるための入口です。

## 主な文書

- [経路GNGから独立した左右到達map](reachability_maps.md)
- [軌道計画・回避・把持の部品配置](component_layout.md)
- [経路計画・目標タスクの差し替え](planning_components.md)

- [CPU版GNGの高速化報告：要約・処理時間・測定条件](./gng_cpu_optimization_summary_20260924.md)
- [把持推定・ファジィ評価の調査要約（2026-09-13）](./FUZZY_GRASP_CURRENT_OVERVIEW.md)
- [GNG・Viewer・入力経路の作業要約（2026-09-02）](./WORK_LOG_2026-09-02.md)
- [可視化・把持・実機接続の開発要約（2026-07-29〜08-14）](./RELEASE_SUMMARY.md)
- [把持ファジールール統合設計書：入力・実装契約・145ルール・拡張指標](./designs/fuzzy_grasp_design.md)
- [把持ファジィ評価の相談用スライド（PowerPoint・PDF）](./presentations/README.md)
- [TECHNICAL_SPEC.md](./TECHNICAL_SPEC.md)
- [TASK_LIST.md](./TASK_LIST.md)
- [TASK_CANDIDATES.md](./TASK_CANDIDATES.md)
- [pending.md: 時間・依存作業などの都合で保留中の作業](./pending.md)
- [progress.md: 実施済みの変更・調査・検証](./progress.md)
- [reject.md: 不採用・採用見送りの判断](./reject.md)
- [RELATED_WORK.md](./RELATED_WORK.md)
- [IDEA_NOTES.md](./IDEA_NOTES.md)
- [RELEASE_NOTE_TEMPLATE.md](./RELEASE_NOTE_TEMPLATE.md)
- [文書更新規約](../../AGENTS.md)
- [旧グラフ縮約案（置換済み）](./releases/2026-08-03_graph_coarsening_visualization.md)
- [designs/2026-08-14_grasp_contact_activation_lookup.md](./designs/2026-08-14_grasp_contact_activation_lookup.md)

## 運用ルール

更新基準・記録先は[文書更新規約](../../skills/maintain-project-docs/SKILL.md)。
読者はROS開発者。利用手順は用途・最短コマンド・設定先・制限を基本とし、設定値はYAML、通信契約は仕様書を正本とする構成。
正式文書への試行履歴の逐次追記・同じ内容の重複記録は不要。
