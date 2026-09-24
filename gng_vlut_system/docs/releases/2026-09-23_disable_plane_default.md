# 2026-09-23 - CPU直結平面クラスタリングの既定無効化

## 1. 要約

処理負荷抑制の依頼に基づくCPU直結平面クラスタ計算の停止。

共通平面YAMLのais_gng_node設定へ`plane_cluster.direct_enabled: false`を追加。

## 2. 条件・検証

通常のCPU launchで平面クラスタ計算・配信と、結果に依存する非平面成分抽出を停止。GNG学習・ノード・エッジ出力は継続。曲面検出も既存設定により無効。点群フィルタへの変更なし。

`plane_cluster.direct_enabled: false`。CPU内蔵の`/plane_clusters`配信を停止。`/nonplane_components`のpublisherは存在しても更新処理は非実行。メッセージ定義の変更なし。

初期化時のClusterizer生成条件、更新時のポインタ条件、非平面抽出の平面結果依存をソースで確認。インストール済みYAMLの読込と`git diff --check`で検証。既存プロセスの再起動・性能計測は未実施。

**制約**

反映にはlaunch再起動が必要。再有効化は同設定をtrueへ変更。GPU版と独立クラスタ計算ノードには非適用。不要な平面可視化ノードは既存引数`start_plane_cluster:=false`で起動を省略可能。
