# 2026-09-02 - 物体GNG保存時のedge添字修正

## 概要

`object_gng_dataset_exporter_node`が`TopologicalMap.edges`とクラスタ所属nodeを
永続node IDとして再解決し、別nodeの配列添字へ誤変換する問題を修正した。

## 変更

- edge端点を受信したnode配列添字のまま保存
- クラスタ所属nodeも配列添字のまま保存
- 自己edgeと範囲外添字の除外は維持

## 影響

修正前のJSONは永続node IDを保持していないため、元edgeの完全な逆算はできない。
`repair_object_gng_edges`はGNGを再学習せず、node座標と既存edge数を維持したまま、
局所間隔に対して長いedgeを近傍edgeへ置換する。デフォルトはdry-runで、`--apply`時は
変更前gzipを別ディレクトリへ退避し、edge以外のJSONが不変であることを再読込してから置換する。

2026-09-02に`/datasets`の17テンプレートへ移行を実施した。長距離誤接続を持つ10件を
幾何修復し、`teapot`の重複edge 1本を正規化した。node、平面クラスタ、点群参照、metadataは
変更していない。変更前の11ファイルは
`/datasets/edge_repair_backup_20260902_200715`へ保存した。

`repair_object_gng_edges`は既存データ移行のための一時ツールとし、通常運用では使用しない。
旧バックアップを復元しないことを確認した後、通常配布物から削除する予定である。

## 検証

- `ais_gng` Releaseビルド
- node ID `[3,0,4,1,2]`とedge添字`[0,1,2,3]`を与える衝突回帰試験
- 保存JSONのedgeが`[[0,1],[2,3]]`、クラスタ所属が`[0,2]`のまま保持されることを確認
- `mug_source.pcd`の一時再生成で異常長edgeが0本になることを確認
- 全17テンプレートのgzip読込と修復ツール再実行を確認し、追加修復が0件であることを確認
- 幾何修復10件でedge数を維持し、`edge_length > 3 * median`の異常長edgeが0本になることを確認
- 複製データへの`--apply`、バックアップ、再実行時の冪等性を確認
