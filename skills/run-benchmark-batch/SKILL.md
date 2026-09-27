---
name: run-benchmark-batch
description: ローカルの反復テスト・性能比較を、回数指定、残り時間予測、結果保存、完了待ちで実行する。長いテスト中の頻繁なログ確認を避けたい場合に使用。
---

# 反復テストの実行と完了待ち

同梱の`scripts/run_batch.py`へJSONの試験一覧を渡す。ROS・GNG非依存、Linux向け。
実行回数は`--repeats`、試験ごとの上限は`--timeout-sec`、全体上限は`--max-total-sec`。
各試行の種は`--start-seed`から順に割当。条件順は試行ごとに循環し、同時実行なし。

```json
{"cases":[{"name":"baseline","argv":["python3","measure.py","--seed","@seed@","--output","@case_dir@/metrics.json"],"metrics":"metrics.json"}]}
```

`argv`はシェル展開なし。`@seed@`、`@trial@`、`@case_dir@`のみ置換。
省略可能な`cwd`はmanifestの親基準、`env`は継承環境への追記。
`metrics`指定時は、試験が書く有限数値だけのJSONオブジェクトが必須。
結果の良し悪しは試験の指標で判断し、正常終了と性能改善を混同しない。

```bash
python3 <skill-dir>/scripts/run_batch.py cases.json --output <新しい結果ディレクトリ> --repeats 5
```

開始・進捗・終了は短いJSONイベント、詳細は条件別ログ・`report.json`・`events.jsonl`。
初回前は予測不明。`--estimate-sec`で1試験分の仮定値を指定可能。
実測後は条件別平均を優先し、未測定条件は全体平均で推定。予測は上限保証ではない。
失敗後は既定で停止。全ケース診断が必要なときだけ`--continue-on-error`を指定。
既存出力の上書き・自動再試行なし。中断時は同じプロセスグループだけ終了。
Docker試験ではrunner自体をコンテナ内で実行。ホストからdocker execだけをkillして終了扱いにしない。

## AIの待機

- 実行ツールの継続セッションで起動し、実行中の会話を維持。
- 初回予測を伝えた後は、利用環境の完了イベントまたは`write_stdin`等のブロッキング待機を使用。
- 高位の待機上限に従う。この環境では1回60秒以内。時間上限・ユーザー入力で戻った場合だけ再待機。
- 待機の合間に`ps`・ログ全文・状態ファイルを何度も読まない。進捗イベントを再利用。
- 終了時に`report.json`を一度読み、試行数、失敗、予測と実時間、品質と全体時間を報告。
- トークン消費ゼロや、終了済みの会話の自動再開は約束しない。スキルと標準出力だけでは外部のAI再起動機構にはならない。
- Codexの`notify`は外向き通知。AI再開用コールバックとの混同禁止。別AI呼出し・常駐監視・通知サービスの勝手な追加なし。

既存プロセスの停止や本番への自動適用は対象外。試験コマンドの権限・副作用は通常どおり確認。
このworkspaceでは`restore-runtime-after-tests`も併用し、終了後の実行状態を確認。
