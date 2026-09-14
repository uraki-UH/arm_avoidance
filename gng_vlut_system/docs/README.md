# Documentation Index

このディレクトリは `gng_vlut_system` の運用仕様と変更履歴をまとめるための入口です。

## 主な文書

- [把持推定・ファジィルールの現状と全体フロー](./FUZZY_GRASP_CURRENT_OVERVIEW.md)
- [ROS把持ファジィ評価の入力・制約・集合の設計案](./designs/fuzzy_grasp_input_design.md)
- [ROSファジィルールエンジンの実装雛形設計（未実装）](./designs/fuzzy_rule_engine_design.md)
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
4. 仕様変更が入ったら、まず `releases/` に 1 件のリリースノートを追加する。
5. 仕様書の変数一覧、トピック一覧、フローチャートは実装と一致させる。
6. 変更の影響が launch 引数、topic、service、message field に及ぶ場合は、必ず仕様書も更新する。
7. 先行研究を調べたら `RELATED_WORK.md` に「手法 / 本プロジェクトとの差分 / 引用する場面」で追記する。
8. 着想レベルで流用できる技術は `IDEA_NOTES.md` に 1 件ずつ短く登録する。
9. 実施済みの事実と検証結果は `progress.md`、不採用・採用見送りの判断は `reject.md` に分離する。進捗には予定を書かず、不採用記録には判断根拠と再検討条件を残す。
10. 詳細な記録先の選択は [作業記録スキル](../../skills/maintain-project-docs/SKILL.md) を参照する。
11. 取り組む意向はあるが今は進めない作業は `pending.md` に置く。理由・完了済み範囲・再開条件・次の一手を残し、再開決定時に `TASK_LIST.md` へ移管する。本文の二重管理や既存タスクの自動的な保留扱いはしない。
