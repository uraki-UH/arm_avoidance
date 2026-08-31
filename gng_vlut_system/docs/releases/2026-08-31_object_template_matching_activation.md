# 2026-08-31 - 物体GNGテンプレート照合と確定配信

## 概要

深度由来で欠損を含む環境側GNGと保存済み物体GNGを照合し、検証済み一致時だけ
事前登録グラフを`/<template_id>/topological_map_static`へ配信する経路を追加した。

## 変更

- `object_template_matcher_node`によるyaw中心の姿勢候補探索
- 法線、`rho`、node次数、edge接続率の連続的なファジー評価
- `object_template_match_validator_node`による連続フレーム検証とヒステリシス
- `activation_state_topic`指定時だけ配信するテンプレート静的マップのゲート
- `object_template_matching.launch.py`と`object_template_matching.yaml`の追加
- GNG node法線の符号反転を無視する方向評価

## 制約

- XY平行移動と位置レジストレーションは実行しない。
- 平面クラスタ、共分散固有値、スケール評価は設定形式の拡張対象であり、初期照合器の評価項目には含めない。
- roll/pitch探索は設定可能だが既定では無効であり、yawだけを探索する。

## 検証

`basket_gng_template.json.gz`を環境側とテンプレート側の両方へ与える隔離ROSドメイン試験で、
score `0.889`、5フレームの確定判定、`/basket/topological_map_static`の配信開始を確認した。
