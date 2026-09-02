# 2026-09-02 - 遮蔽対応テンプレート照合の計算コスト見積もり

## Summary

遮蔽を含む環境で、平面クラスタ間の大まかな位置関係と非平面成分を使って
事前登録テンプレートを照合する場合の処理構成と計算コストを整理した。
既存処理の実測値と、未実装の部分グラフRANSAC照合の推定値は分けて記載する。

## Current Pipeline

1. CPU GNGが入力点群から`TopologicalMap`を生成する。
2. 同一プロセス内のincremental plane clusterizerが`PlaneClusterArray`を生成する。
3. `nonplane_component_node`が平面未所属nodeをGNG edgeで連結成分化する。
4. 現行`object_template_matcher_node`は、平面対応と10度刻みのyaw候補を評価する。

現行matcherはRANSACを使用していない。既定設定では`-180`度から`180`度まで
10度刻みの姿勢候補を生成し、各候補で平面、node、edge、反証、scaleを評価する。

## Measured Cost

以下は既存のReleaseビルドで取得済みの実測値である。

| 対象 | 入力規模 | 実測時間 |
|---|---|---:|
| GNGから平面クラスタまでの全処理 | 20,000 points、約1,480 nodes、約4,000 edges、20 Hz | 4.1-4.7 ms/frame |
| 上記に含まれる平面クラスタ更新 | 同上 | 0.38-0.49 ms/frame |
| 平面クラスタ単体の合成benchmark | 2,760 nodes、5,344 edges、2,000 updates | average 0.250 ms、p50 0.242 ms、p95 0.298 ms |

平面クラスタ単体値はROS publish、marker生成、DDS配送を含まない。
全処理値はGNG、ROS message変換、平面クラスタ処理、publishを含む。

## Nonplane Component Cost

非平面成分抽出は次の線形処理で構成される。

- 平面所属表の生成
- 非平面edgeのunion-find連結
- nodeと内部edgeの再構成
- 平面へのanchor edge抽出
- 各成分の重心とAABB算出

node数を`N`、edge数を`E`、平面所属node総数を`P`とすると、時間計算量は
概ね`O(N + E + P)`、作業メモリは`O(N)`である。

`/nonplane_components/timing`はframe番号と処理時間を配信している。ただし現行値は
抽出だけでなく、marker生成、`TopologicalMap`生成、publish呼び出しまで含む。
代表的な実点群での統計値はまだ採取していないため、以下は推定値とする。

| 対象 | 約1,500 nodes、約4,000 edgesでの推定 |
|---|---:|
| 連結成分抽出だけ | 0.1-0.8 ms/frame |
| marker生成とpublish呼び出しを含む現行timing | 0.2-2.0 ms/frame |

可視化負荷を評価から分離するには、extract、message、marker、publishを別々に計測する必要がある。

## RANSAC Estimate

導入候補は、個々の平面OBBを完全一致させる方式ではなく、法線角度、平面間距離、
隣接関係、anchor関係から2面または3面の対応仮説を作る部分グラフRANSACとする。
必要反復数は、confidenceを`p`、正しい対応候補率を`w`、標本数を`s`として次で求める。

```text
iterations = log(1 - p) / log(1 - w^s)
```

confidence 99%の場合の目安は次のとおり。

| 正しい対応候補率 | 2平面sample | 3平面sample |
|---:|---:|---:|
| 50% | 17 | 35 |
| 30% | 49 | 169 |
| 20% | 113 | 573 |
| 10% | 459 | 4,603 |

法線種別と平面間角度で候補を事前に絞り、最大200反復とした場合、1テンプレート当たりの
推定コストは次の範囲とする。

| 処理 | 推定時間 |
|---|---:|
| 対応候補生成 | 0.05-0.5 ms |
| RANSAC 200反復と平面関係検証 | 0.4-4.0 ms |
| 非平面特徴の部分対応評価 | 0.1-1.0 ms |
| 照合追加分合計 | 0.55-5.5 ms/template |

点群ICPはこの見積もりに含めない。ICPを最終検証へ追加する場合は、点数と反復数により
さらに5-30 ms以上かかる可能性がある。

## End-to-End Estimate

約1,500 nodes、約4,000 edges、単一テンプレート、RANSAC最大200反復では、
現在取得済みのGNG・平面クラスタ実測値へ非平面処理と照合推定値を加え、
概ね`4.85-12.2 ms/frame`を初期予算とする。

平面クラスタと非平面成分は全テンプレートで共有できる。テンプレート数に対して主に増えるのは
照合部分であるため、多数テンプレートでは法線角度signatureによるshortlist、変更frameだけの再評価、
早期終了が必要となる。

## Timing Contract

Release実測では、最低限次の値を同一frame番号で記録する。

```text
GNG: <ms>, Plane: <ms>, Nonplane: <ms>, Match: <ms>
Match detail: candidate=<ms>, ransac=<ms>, verify=<ms>, templates=<n>, hypotheses=<n>
```

計測はReleaseビルド、warm-up 100 frames以上、連続1,000 frames以上で行い、
averageだけでなくp50、p95、p99を保存する。非平面markerの有効・無効は別条件で測定する。

## Behavior Impact

このリリースノートは設計と計算予算の記録であり、実行時挙動、topic、parameter、messageは変更しない。
RANSACおよび非平面特徴を用いたテンプレート照合は未実装である。

## Risk / Notes

- 非平面成分とRANSACの時間は現時点では推定であり、対象PC上のRelease実測で更新する。
- 3平面RANSACは対応候補率が低いと反復数が急増するため、無制限反復にしない。
- 1平面だけでは平面内位置と回転、2平面だけでは交線方向の位置が一意に決まらない。
- 欠損したテンプレート面は減点せず、観測された構造との明確な矛盾だけを反証として扱う。
