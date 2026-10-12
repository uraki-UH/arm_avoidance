# 参考記事と採用した判断

確認日: 2026-10-12。原文転載ではなく、本スキルの運用への要約と適用判断。

| 一次資料 | 記事の要点 | 本スキルへの適用 |
| --- | --- | --- |
| [OpenAI: Multi-agent](https://developers.openai.com/api/docs/guides/agents-api/multi-agent) | 独立タスクの委任、短い・依存する工程の主担当処理、共有編集の調整 | 目的・編集境界・完了条件を指定した委任 |
| [OpenAI: Multi-agent in Responses](https://developers.openai.com/api/docs/guides/responses-multi-agent) | 独立した作業と文脈の分離、使用量増加と共有状態の調整負担 | 通常コマンド並列化との使い分け、総コストの比較 |
| [Anthropic: Building effective agents](https://www.anthropic.com/engineering/building-effective-agents) | 指示役と実行役、明確な評価基準を持つ評価・改善ループ、必要な複雑さだけを追加 | 単独処理から開始し、測定・判断に必要な役割だけを分離 |
| [Anthropic: How we built our multi-agent research system](https://www.anthropic.com/engineering/multi-agent-research-system) | タスクの規模に応じた分担、限定した依頼、研究系マルチエージェントの大きな使用量 | 小さい依頼と証拠付き報告、担当数の抑制とusage計測 |
| [OpenAI: Using Goals in Codex](https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex) | 目標・証拠・停止条件を持つ継続処理 | 明示的なGoal依頼への対応と、通常バッチ・自動再開との区別 |

モニターの通常処理をログ・評価器へ寄せる構成と、改善一候補から始める既定範囲は、上記の単純化・実測の方針を今回の環境に適用した設計判断。記事による一律の推奨人数・停止回数ではない。

Anthropicの2025年の記事にあるトークン倍率は、同社の研究機能とチャットの比較。Codexの単独実行とサブエージェント実行の一般的な倍率・節約率として使用しない。旧記事のAPI・モデル例も現行仕様として転用しない。

本スキルは分担の判断と運用。全サブエージェントのusage取得、ネイティブ呼出しへのoptimizer移植、費用削減の保証は含まない。実測機能の追加は別の実装作業。
