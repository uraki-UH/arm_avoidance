---
name: optimize-agent-instructions
description: 正解付きタスクを使い、作業AI向けSKILL.mdの改善候補を実AIで生成し、品質・トークン・時間を比較する場合に使用。
---

# 指示書の実測改善

実装と設定仕様は [実行ガイド](../../tools/skill_optimizer/README.md) を参照。

1. 対象の指示書、変更しない作業仕様、正解付きタスクを用意。入力の重複を避けて train / validation / test に分割。
2. experiment.json にモデル・反復回数・呼出し数・時間・総トークン上限を設定。既存ログインを利用するため、実行枠を消費。単価不明時は金額を算出しない。
3. リポジトリルートで `python3 -m tools.skill_optimizer.optimizer <experiment.json> --output <新規結果ディレクトリ>` を実行し、候補生成から最終testまで確認。長い反復比較には `run-benchmark-batch` を併用。
4. `report.md` と `report.json` で採用判定・品質・実測トークン・探索分も含む総コストを確認。完走と改善成功を区別。
5. `selected.SKILL.md` と候補diffを成果物として提示。原本への適用はユーザーの作業範囲に従う。

固定仕様、正解、評価器、権限境界は改善AIの編集対象外。改善AIへ渡す実行例はtrainだけ。testを見た再調整時は新しい独立testセットが必要。

現実装はテキスト入力→JSON/テキスト出力向け。固定JSONは型まで完全一致。実ファイル編集、ROS実行、ブラウザ操作、SKILLの自動発見率は別の評価環境が必要。

比較中は同じモデルと条件を維持し、欠測をゼロ扱いしない。合格数が落ちた候補は、安くても採用しない。モデル呼出しのPIDと終了状態は各呼出しmetadataで確認。
