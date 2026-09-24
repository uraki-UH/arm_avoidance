# Documentation Index

このディレクトリは `gng_vlut_system` の運用仕様と変更履歴をまとめるための入口です。

## 主な文書

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
- [releases/2026-07-29_documentation_workflow.md](./releases/2026-07-29_documentation_workflow.md)
- [designs/2026-08-03_graph_coarsening_visualization.md](./designs/2026-08-03_graph_coarsening_visualization.md)
- [designs/2026-08-14_grasp_contact_activation_lookup.md](./designs/2026-08-14_grasp_contact_activation_lookup.md)

## 運用ルール

1. 実際に進める順序がある作業は `TASK_LIST.md` に置く。
2. まだ決め切っていない候補は `TASK_CANDIDATES.md` に置く。
3. 仕様として固定した内容は `TECHNICAL_SPEC.md` に置く。
4. 報告は[短い書式](RELEASE_NOTE_TEMPLATE.md)へ統一。同じ作業の続報は既存ノートへ統合し、独立した変更だけ`releases/`へ追加する。
5. 仕様書の変数一覧、トピック一覧、フローチャートは実装と一致させる。
6. 変更の影響が launch 引数、topic、service、message field に及ぶ場合は、必ず仕様書も更新する。
7. 先行研究を調べたら `RELATED_WORK.md` に「手法 / 本プロジェクトとの差分 / 引用する場面」で追記する。
8. 着想レベルで流用できる技術は `IDEA_NOTES.md` に 1 件ずつ短く登録する。
9. 実施済みの事実と検証結果は `progress.md`、不採用・採用見送りの判断は `reject.md` に分離する。進捗には予定を書かず、不採用記録には判断根拠と再検討条件を残す。
10. 詳細な記録先の選択は [作業記録スキル](../../skills/maintain-project-docs/SKILL.md) を参照する。
11. 取り組む意向はあるが今は進めない作業は `pending.md` に置く。理由・完了済み範囲・再開条件・次の一手を残し、再開決定時に `TASK_LIST.md` へ移管する。本文の二重管理や既存タスクの自動的な保留扱いはしない。
