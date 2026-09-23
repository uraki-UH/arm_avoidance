# 2026-09-23 - 曲面検出の既定無効化

## Summary

処理負荷抑制の依頼に基づく曲面検出の既定OFF化。

## Changed

共通設定の`surface_model.enable`を`true`から`false`へ変更。

## Behavior Impact

再起動後は曲面抽出・追跡と曲面出力を停止。GNG学習・平面検出は維持。CPU・GPU・単独曲面launchで共通設定を使用。

## Topics / Params / Messages

`surface_model.enable: false`。曲面の`/curved_surface_clusters`配下のpublisher生成を抑止。launch引数・メッセージ定義の変更なし。

## Verification

`surface_model_visualization.cpp`の初期化とupdateの無効時returnを確認。コンテナのインストール済み設定がソース設定へのシンボリックリンクであることを確認。YAML読込と`git diff --check`で検証。実行時間の比較は未実施。

## Risk / Notes

既存プロセスの停止・再起動なし。反映には利用者によるlaunch再起動が必要。再有効化は同設定を`true`へ復帰。
