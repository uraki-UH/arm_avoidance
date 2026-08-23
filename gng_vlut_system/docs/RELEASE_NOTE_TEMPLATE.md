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

## まとめ版の書き方

複数のリリースノートを 1 件へまとめるときは、個別ノートと主題を入れ替える。

- 主題は「最終的にどうなったか」。到達した仕様と挙動を本文の中心に据える。
- 「〇〇を修正した」「〇〇を変更した」のような途中経過は、箇条書き 1 行まで畳む。経緯や試行錯誤は書かない。
- 同じ対象を何度も直した場合は、最後の状態だけを本文に書き、途中の修正は箇条書きへ送る。
- topic、parameter、message は最終形を必ず残す。途中で廃止されたものは「廃止」とだけ書く。
