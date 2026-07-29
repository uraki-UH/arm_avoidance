# gng_vlut_system Task Candidates

この文書は、まだ実装方針を固定しきっていない変更候補を手で追記していくための作業メモです。
正式な仕様は [TECHNICAL_SPEC.md](./TECHNICAL_SPEC.md) に置き、ここには残課題だけを積んでいきます。

## 運用ルール

1. 仕様として固定したものは `TECHNICAL_SPEC.md` に移す。
2. 未確定のものはここへ追記する。
3. 1 件ずつ短く、後で見返して判断しやすい形で残す。

## 候補

### 評価指標の flat 構成整理

- `GraspCandidateMetric` は flat なまま維持する。
- 可操作性は `position` / `rotation` / `dynamic` の 3 系統へ分ける候補を残す。
- `condition_number` だけでなく、`manipulability`、`min_singular_value`、`valid` も同じ粒度で保持する。
- 経路候補については `path_cost` のような単一コストへ早期圧縮せず、経由ノード ID 列と関節角度列を優先して保持する。

### 候補ポーズと GNG ノードの関係

- `grasp pose candidate` と `goal GNG node` は 1 対 1 に潰さず、多対多で扱う候補を残す。
- `pose_id` と `goal_node_id` の対応は、後段で総合評価しやすい形式へ整理する。
- 同じ GNG ノードが複数の把持候補から参照される前提で、逆引き情報も保持できるようにする。

### 今後の記入ルール

- このファイルは手で更新する。
- 新しい候補が出たら、まずここへ追記する。
- 実装済みで仕様として固定したものは、上の正式な仕様へ昇格させる。
