# Documentation Index

このディレクトリは `gng_vlut_system` の運用仕様をまとめるための入口です。

## 主な文書

- [TECHNICAL_SPEC.md](./TECHNICAL_SPEC.md)
- [TASK_LIST.md](./TASK_LIST.md)
- [TASK_CANDIDATES.md](./TASK_CANDIDATES.md)
- [RELATED_WORK.md](./RELATED_WORK.md)
- [IDEA_NOTES.md](./IDEA_NOTES.md)
- [RELEASE_NOTE_TEMPLATE.md](./RELEASE_NOTE_TEMPLATE.md)

## 運用ルール

1. 実際に進める順序がある作業は `TASK_LIST.md` に置く。
2. まだ決め切っていない候補は `TASK_CANDIDATES.md` に置く。
3. 仕様として固定した内容は `TECHNICAL_SPEC.md` に置く。
4. 仕様変更が入ったら、まず `releases/` に 1 件のリリースノートを追加する。
5. 仕様書の変数一覧、トピック一覧、フローチャートは実装と一致させる。
6. 変更の影響が launch 引数、topic、service、message field に及ぶ場合は、必ず仕様書も更新する。
7. 先行研究を調べたら `RELATED_WORK.md` に「手法 / 本プロジェクトとの差分 / 引用する場面」で追記する。
8. 着想レベルで流用できる技術は `IDEA_NOTES.md` に 1 件ずつ短く登録する。
