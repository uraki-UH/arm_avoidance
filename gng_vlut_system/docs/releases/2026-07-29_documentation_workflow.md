# 2026-07-29 - technical spec and release-note workflow

## Summary

`gng_vlut_system` の仕様把握用ドキュメントと、変更ごとに記録するリリースノートのテンプレートを追加した。

## Added

- `docs/TECHNICAL_SPEC.md`
- `docs/RELEASE_NOTE_TEMPLATE.md`
- `docs/releases/2026-07-29_documentation_workflow.md`

## Behavior Impact

- 実行時の挙動変更はない
- 仕様変更時の記録方法だけを追加した

## Verification

- `python3 -m py_compile` を含む既存の検証フローに影響しないことを確認

## Risk / Notes

- 今後の変更では、このテンプレートに従って 1 変更 1 リリースノートを残す
