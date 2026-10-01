# SpatialTree 固定依存

取得元: `https://github.com/urakiharuto/SpatialTree.git` のローカル clone `/home/uraki/SpatialTree`。
固定リビジョン: `92e7322a8b7dbdb50be812d0b092d90aec982414`。

通常 GNG 学習の動的近傍探索に必要な 6 ヘッダの無改変コピー。原本コメント・識別子の保持。元リポジトリへの変更なし。各ヘッダの SHA-256 とバイト数は [provenance.json](provenance.json) に記録。

`MovingBSPTree` の double 座標、`NoHysteresis`、`approx_eps=0` を使用。学習ノードとは別の安定した要素アドレスを保持し、座標変更は `updatePosition` 経由。GNG 側の float 距離と ID 順の維持は `src/core/gng/nearest_node_index.cpp` の候補再評価・境界検査による実施。

元リポジトリの指定リビジョンには LICENSE ファイルなし。外部への再配布条件は本コピーによる新規設定なし。
