# 把持候補近傍の重点学習

## 起動

CPUバックエンド専用、既定OFF。把持候補の生成ノードは別途起動し、クラスタ所属付きの`/grasp_pose_cands/Tmap`を配信しておく。

```bash
ros2 launch ais_gng ais_gng.launch.py \
  backend:=cpu lidar:=graspnet.yaml input_topic:=/semantic_points \
  enable_grasp_attention:=true
```

通常学習への切替は`enable_grasp_attention:=false`で再起動。launch引数を省略した場合の値`auto`はYAML設定を使用。稼働中プロセスへの自動反映なし。

## 設定

`config/gng_cpu/graspnet.yaml`の`ais_gng_node.ros__parameters`で設定。全項目は起動時設定、ROSパラメータの実行中変更は不可。

YAMLで`enable_grasp_attention: true`にすればlaunch引数の追加は不要。旧`grasp_attention.radius`は`grasp_attention.margin`へ置換。半径検索との切替オプションはなし。

| パラメータ | 既定値 | 内容 |
| --- | --- | --- |
| `enable_grasp_attention` | `false` | 重点学習の有効化 |
| `grasp_attention.topic` | `/grasp_pose_cands/Tmap` | 候補ノードのTopologicalMap |
| `grasp_attention.margin` | `0.03` | 候補AABBの各方向の余白[m]、0以上 |
| `grasp_attention.ratio` | `0.5` | 総学習回数中の重点更新の配分率、0より大きく1未満 |
| `grasp_attention.timeout_sec` | `0.5` | 受信停止・候補時刻の許容期間[s]、正数 |

## 処理

1. 候補Graphの各ノード位置を、入力点群に対応するGNG座標系へTF変換。
2. `clusters[].nodes`をノードIDとして解決し、クラスタごとのAABBを構築。TF変換後のノード位置から範囲を計算し、各方向に余白を付加。
3. GNGに既に入力済みの実測XYZから、いずれかのAABB内の元点添字を選択。複数候補を一括したAABBは作らず、重複範囲の点も1回だけ選択。学習への入力は添字のみで、点群ボクセル再登録なし。可視化購読時だけ選択点を別途配信。
4. GNG既存の入力範囲フィルタに従って重点点を限定。
5. `node.learning_num`を増やさず、指定比率で重点更新を通常更新へ挿入。

例えば5000回・比率0.5なら重点2500回、従来方式2500回。従来方式の枠内では既存の全体点群／人・未知物体重点点群の混合比を維持するため、「全体一様2500回」ではない。

実エッジ更新・勝者選択・学習係数・ノード追加条件は従来処理を使用。点の密度やGNGノード数の自動増加を保証する機能ではなく、観測済み領域への学習資源配分。

ノードからの距離にかかわらず候補内部・余白内の実測点が対象。AABBは観測済み候補の広がりであって真の物体形状の確定値ではなく、近くの床や別物体の点も範囲に入る場合あり。計算量は範囲構築が所属ノード数に比例し、入力判定は点数×候補数。ノード単位のkd-tree検索は廃止。

## 選択点群の可視化

`/downsampling/grasp`（`sensor_msgs/msg/PointCloud2`）を購読。重点学習ON時のみPublisherを作成し、購読者がいる場合だけXYZを点群化。追加のON/OFF設定はなし。

- 内容は候補AABB＋余白内の重点入力候補。GNG内部の入力範囲フィルタ適用前であり、最終的に各反復で抽選された点の順列や回数ではない。
- 重複なしのXYZのみ。RGB・semanticラベルは付加しない。
- ヘッダは`/topological_map`と同じGNG座標系・入力点群時刻。
- GNG入力処理ごとに配信。候補なし・失効・TF失敗・該当点なしの場合は空点群を配信。入力点群自体が止まった場合の独立した消去タイマーはなし。
- QoSはbest_effort、volatile、depth 1。履歴再配信なし。名前空間付き起動では同じ名前空間内の`downsampling/grasp`。
- `/downsampling/unknown`・`/downsampling/human`は従来の別出力。

設定有効化とGNG再起動後、ViewerのPointCloud2ストリームとして選択可能。CLIで確認する場合:

```bash
ros2 topic echo /downsampling/grasp --field width --qos-reliability best_effort
```

## 通常学習への復帰

候補なし、空候補、候補の受信停止、入力に対して古い/未来の候補時刻、時刻またはframe_id欠落、TF取得失敗、対象実測点なしの場合は通常学習のみ。入力自体の処理は継続。

- 受信からの経過はsteady clock、候補と入力の時間差はROSヘッダで確認。状態通知だけの再送による古い候補の延命なし。
- TFは入力点群の時刻で要求、最新TFへの代替なし。同一frame_idはTF不要。
- 候補位置の予測や、候補生成から入力取得までの物体運動補償は未実装。
- 現実装は単一入力点群のみ。複数入力時は警告と通常学習への復帰。
- 候補ノードのラベルや到達性による選別なし。クラスタ未所属ノードは範囲外。クラスタ所属がなければ通常学習へ復帰。65536ノードを超える入力は重点指定を解除。
- 空クラスタ、参照先のないID、所属ノードの不正座標は該当クラスタを除外。Graph内の重複ノードIDは全重点範囲を解除。範囲推測による代替なし。
- OFF時は候補購読・TF追加購読・近傍検索なし。

## 観測統計と公開API

重点更新分はノード位置・実エッジの学習に使用するが、共分散・支持統計・観測方向件数・統計用勝者イベントには計上しない。通常枠の観測統計は従来どおりで、全反復に対する独立観測の重複排除機能ではない。重点配分を増やすと通常枠の統計更新回数は減る。学習位置が変わるため、共分散の値そのものが従来と同一になる保証もない。

CPUライブラリへ`gng_set_priority_input(point_ids, num_points, ratio)`を追加。`gng_setPointCloud`後に、入力済み点の添字をコピーして次の`gng_exec`1回へ適用。空指定は解除、不正添字・配分率は解除して失敗返却。次の点群入力または実行終了で失効。

既存API構造体の変更なし。新ROSノードには更新した`gng_cpu`ライブラリが必要。署名付き配布は通常の再ビルド・署名工程が必要。GPU・WASMのUIへは追加していない。

## 検証

[AABB方式の実行コマンド・結果](../../gng_vlut_system/docs/releases/2026-09-15_grasp_attention_aabb.md)を参照。実物把持の成功率改善や実環境の最適な余白・配分率は未検証。
