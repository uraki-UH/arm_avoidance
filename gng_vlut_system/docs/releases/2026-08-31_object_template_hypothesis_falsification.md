# 2026-08-31 - 物体テンプレート仮説の反証と破棄

## 概要

高scoreで確定した物体GNGテンプレートでも、環境側の高支持量GNG nodeがテンプレート構造で
説明できない場合に、仮説を破棄して静的テンプレート配信を停止する経路を追加した。

## 変更

- 対応済みnodeに隣接する説明不能nodeの検出
- `winner_point_count`に基づく`contradiction_point_ratio`の算出
- 反証率が上限を超えた姿勢候補の除外と次点候補の選択
- 全候補除外時の`no_hypothesis`出力
- 確定後の反証到着時における即時の仮説破棄と静的マップ配信停止

## 設定

`object_template_matching.yaml`の`enable_contradiction_evaluation`、
`contradiction_weight`、`max_contradiction_point_ratio`で反証評価を設定する。

## 検証

隔離ROSドメインで通常の`basket`環境GNGを確定させた後、高支持量で説明不能な隣接nodeを投入した。
37個の姿勢候補が除外され、`no_hypothesis`、仮説破棄、`/basket/topological_map_static`の配信停止を確認した。
