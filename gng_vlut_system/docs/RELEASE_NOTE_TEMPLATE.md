# Release Note Template

変更ごとに 1 ファイルで残すことを前提にしたテンプレートです。

```md
# YYYY-MM-DD - short title

## Summary

## Changed

## Added

## Fixed

## Removed

## Behavior Impact

## Topics / Params / Messages

## Verification

## Risk / Notes
```

## 書き方のルール

- 1 変更 1 ファイルを基本とする
- launch 引数、topic、message field、service の変更は必ず記載する
- no-motion モードや fallback のような挙動変化は `Behavior Impact` に書く
- 検証したコマンドは `Verification` に残す
- 後から戻せるように、破壊的変更は `Removed` と `Risk / Notes` に明記する
