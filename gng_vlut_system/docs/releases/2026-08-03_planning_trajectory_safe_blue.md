# 2026-08-03 - planning trajectory safe blue

## Summary

計画済み軌道と候補軌道を静的GNGから区別しやすい青色で初期表示する。

## Changed

- `planned_topological_map`と`candidate_topological_map`のsafeノード既定色を`#3b82f6`にした。
- 同じ軌道レイヤーのedge既定色を`#2563eb`にした。

## Added

- 既に生成済みの軌道レイヤーが旧既定の緑色なら、青色へ移行する処理を追加した。

## Fixed

- なし。

## Removed

- なし。

## Behavior Impact

dangerノードの黄、collisionノードの赤、候補目標の紫は変わらない。色設定で既定色以外を
選択済みの場合は自動変更しない。

## Topics / Params / Messages

- topic、parameter、messageの変更はない。

## Verification

- Frontend production build。
- 変更対象FrontendファイルのESLint。

## Risk / Notes

- edgeは状態labelを持たないため、軌道レイヤー内のedge全体を青で表示する。
