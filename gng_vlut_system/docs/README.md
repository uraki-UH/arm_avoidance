# Documentation Index

このディレクトリは `gng_vlut_system` の運用仕様と変更履歴をまとめるための入口です。

## 主な文書

- [TECHNICAL_SPEC.md](./TECHNICAL_SPEC.md)
- [RELEASE_NOTE_TEMPLATE.md](./RELEASE_NOTE_TEMPLATE.md)
- [releases/2026-07-29_documentation_workflow.md](./releases/2026-07-29_documentation_workflow.md)

## 運用ルール

1. 仕様変更が入ったら、まず `releases/` に 1 件のリリースノートを追加する。
2. 仕様書の変数一覧、トピック一覧、フローチャートは実装と一致させる。
3. 変更の影響が launch 引数、topic、service、message field に及ぶ場合は、必ず仕様書も更新する。
