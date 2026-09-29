# 指示書最適化の試作

作業AIの `SKILL.md` を改善AIが書き換え、正解付きタスクで品質と実行コストを比較する最小構成。Python 3.11以上の標準ライブラリとログイン済みCodex CLIが必要。追加のPython依存なし。

```text
固定仕様 + 現行SKILL + train入力 → 作業AI → 正解との比較・usage記録
                                        ↓
                train実行記録 → 改善AI → 候補SKILLとdiff
                                        ↓
          現行/候補のvalidation比較 → 品質維持・コスト減の候補
                                        ↓
                    独立test比較 → 採否・selected.SKILL.md
```

最初の対象はテキストの抽出・分類・定型報告。Codexが与えられた指示を使って最終出力を返す実試験であり、SKILLの自動発見や本番リポジトリでのコード実装能力までの証明ではない。GEPA自体は未導入で、探索部分はtrainの失敗・実行記録を使った単純な逐次改善。不採用候補もtrain成績とともに次の改善AIへ渡し、同じ失敗の繰り返しを避ける構成。

[2026-09-29の運用試験](reports/2026-09-29.md)では、初回のコスト増候補を不採用とし、改善AIへの指示を調整した2回目で新規test全問正解と総トークン3.29%削減を確認。ただし最終testは3問×2反復であり、一般的な開発タスクへの効果は未検証。採用候補は `examples/report_extraction/optimized.SKILL.md` に原本と分けて保存。

## 実行

リポジトリルートで実行。

```bash
codex login status
python3 -m unittest discover -s tools/skill_optimizer/tests -v
python3 -m tools.skill_optimizer.optimizer \
  tools/skill_optimizer/examples/report_extraction/experiment.json \
  --output tools/skill_optimizer/runs/first
```

`runs/` はGit管理対象外。既存結果への上書きは禁止。出力先を変えて再実行可能。CLIはユーザーのログインを使用し、契約プランの実行枠またはAPI利用量を消費。

`skills/optimize-agent-instructions/SKILL.md` がAI向けの入口。リポジトリ内配置が前提で、グローバルのskillsへは未インストール。スキルのパスを指定して利用可能。

`run-benchmark-batch` がある環境では、`examples/report_extraction/batch.json` をmanifestとして `--repeats 1 --timeout-sec 1200 --max-total-sec 1250 --estimate-sec 300` で実行可能。内部で問題ごとの2反復を行うため、外側は1回。外側の反復を増やして同じtestを見ながら改修する用途には使用しない。

サンプルは日本語CIメモの状態・時間・件数をJSON抽出する9問。train / validation / test各3問、validationとtestは各2反復。改善案は1案。改善候補が採用される経路で31回、採用されない経路で25回のモデル呼出し。試行順は同じケース内で現行/候補を交互に入れ替え。反復は独立セッションだが乱数seed固定や完全再現性の保証なし。

`experiment_v2.json` は初回運用後の再試験用で、train/validationを維持し、最終test3問を新規作成した設定。対応する外側の反復試験manifestは `batch_v2.json`。

## 設定

`experiment.json` からの相対パスで `policy`（固定仕様）、`skill`（初期指示書）、`dataset`（正解付き問題）を指定。

| 項目 | 内容 |
| --- | --- |
| `worker_model` / `teacher_model` | モデル名。nullはユーザー設定のmodelのみ参照し、呼出しmetadataへ記録 |
| `max_rounds` | 改善AIによる最大提案数 |
| `repeats` | validation / test各問の反復数。trainは各問1回 |
| `max_calls` | 改善AIと作業AIを合計した最大呼出し数 |
| `timeout_sec` / `max_total_sec` | 1呼出し / 全体の実時間制限 |
| `max_total_tokens` | 全呼出しの入力+出力トークン上限。使用量が完了時のみ通知されるため、進行中1呼出し分は超過の可能性 |
| `min_quality` | 候補に要求する完全一致の正解率 |
| `min_token_improvement_ratio` | 現行に対する総トークン削減率の採用下限 |
| `grader` | `exact_json`（型・キーを含む一致）または `exact_text`（両端の空白を除いた一致） |
| `prices` | 未設定はnull。設定時は下記の単価による参考金額 |

データ形式:

```json
{"cases":[{"id":"unique_case_id","split":"train","input":"入力文章","expected":{"answer":1}}]}
```

3分割とも最低1問、IDと入力は全体で重複不可。validationは候補選択に使用するため完全な未知データではない。testは候補確定後だけ使用し、結果を改善AIへ返さない。同じtestで繰り返し調整すると過適合するため、新たな最終評価データが必要。

単価の例の構造（値は利用モデルの現在の単価を利用者が指定）:

```json
{
  "prices": {
    "worker": {"input_per_million": 1, "cached_input_per_million": 0.1, "output_per_million": 4},
    "teacher": {"input_per_million": 1, "cached_input_per_million": 0.1, "output_per_million": 4}
  }
}
```

キャッシュ済み入力は入力総数の内数。通常入力と二重計上せず、出力はプロバイダーusageの値を利用。実請求額ではなく指定単価での参考値。ChatGPT契約枠から金額へ換算する機能はない。

## 成果物と採用条件

- `report.md` / `report.json`: 品質、入力・出力・キャッシュ済みtoken、時間、各段階の採否、全呼出しのコスト。
- `candidate_*.SKILL.md` / `.diff`: 改善AIの提案と親候補からの差分。
- `selected.SKILL.md`: 最終採用候補。品質・コスト条件未達や途中エラーでは原本相当。
- `calls/*`: 渡したprompt、JSONLイベント、標準エラー、最終出力、metadata。
- `evaluations.jsonl`: 決定的な評価器の採点結果。`metrics.json` は反復試験runner向けの有限数値。
- 入力ファイルのスナップショットとSHA-256: 別の条件で評価した結果の混在防止。

同じケース・反復数で、全体品質と各問の品質を下げず、品質下限と総トークン削減下限を通過した候補のみ採用。平均だけ良くなって苦手問題が増える変更は不採用。最終testで同じ条件を再確認し、失敗なら現行を維持。経過時間と金額は参考記録で、現試作の採用目的関数は品質制約付き総トークン削減。価格を含む複数目的のPareto探索は未実装。

原本の自動更新なし。空の一時作業ディレクトリ、read-only設定、ツール無効設定で呼出し、ツールイベントを検出した試行はエラー。モデルへ渡すのは当該入力・固定仕様・候補だけ。改善AIへ渡すのはtrain記録だけ。完全なOSレベルの秘匿環境ではないため、将来ファイル・ツール実行を許可する場合は別コンテナ等による評価データの分離が必要。

タイムアウト・中断時は起動したプロセス群のみ停止。既存ROSノードやコンテナには触れない。元ワークスペースの設定と本番の指示書への変更なし。

## 実装の参考

実行トレースとusageの取得は[Codex非対話実行](https://developers.openai.com/codex/noninteractive)、スキルの評価設計は[Testing Agent Skills Systematically with Evals](https://developers.openai.com/blog/eval-skills)を参照。探索器を発展させる場合は[GEPA](https://github.com/gepa-ai/gepa)へのadapter化が候補。
