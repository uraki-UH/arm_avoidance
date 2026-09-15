# 把持候補近傍の重点学習

## 起動

CPUバックエンド専用、既定OFF。把持候補の生成ノードは別途起動し、`/grasp_pose_cands/Tmap`を配信しておく。

```bash
ros2 launch ais_gng ais_gng.launch.py \
  backend:=cpu lidar:=graspnet.yaml input_topic:=/semantic_points \
  enable_grasp_attention:=true
```

通常学習への切替は`enable_grasp_attention:=false`で再起動。launch引数を省略した場合の値`auto`はYAML設定を使用。稼働中プロセスへの自動反映なし。

## 設定

`config/gng_cpu/graspnet.yaml`の`ais_gng_node.ros__parameters`で設定。全項目は起動時設定、ROSパラメータの実行中変更は不可。

| パラメータ | 既定値 | 内容 |
| --- | --- | --- |
| `enable_grasp_attention` | `false` | 重点学習の有効化 |
| `grasp_attention.topic` | `/grasp_pose_cands/Tmap` | 候補ノードのTopologicalMap |
| `grasp_attention.radius` | `0.03` | 候補ノード周辺の半径[m]、正数 |
| `grasp_attention.ratio` | `0.5` | 総学習回数中の重点更新の配分率、0より大きく1未満 |
| `grasp_attention.timeout_sec` | `0.5` | 受信停止・候補時刻の許容期間[s]、正数 |

## 処理

1. 候補Graphの各ノード位置を、入力点群に対応するGNG座標系へTF変換。
2. 重複中心を除去し、候補中心だけのkd-treeを構築。
3. GNGに既に入力済みの実測XYZから半径内の元点添字を選択。ROS再publish・点群複製・点群ボクセル再登録なし。
4. GNG既存の入力範囲フィルタに従って重点点を限定。
5. `node.learning_num`を増やさず、指定比率で重点更新を通常更新へ挿入。

例えば5000回・比率0.5なら重点2500回、従来方式2500回。従来方式の枠内では既存の全体点群／人・未知物体重点点群の混合比を維持するため、「全体一様2500回」ではない。

実エッジ更新・勝者選択・学習係数・ノード追加条件は従来処理を使用。点の密度やGNGノード数の自動増加を保証する機能ではなく、観測済み領域への学習資源配分。

## 通常学習への復帰

候補なし、空候補、候補の受信停止、入力に対して古い/未来の候補時刻、時刻またはframe_id欠落、TF取得失敗、対象実測点なしの場合は通常学習のみ。入力自体の処理は継続。

- 受信からの経過はsteady clock、候補と入力の時間差はROSヘッダで確認。状態通知だけの再送による古い候補の延命なし。
- TFは入力点群の時刻で要求、最新TFへの代替なし。同一frame_idはTF不要。
- 候補位置の予測や、候補生成から入力取得までの物体運動補償は未実装。
- 現実装は単一入力点群のみ。複数入力時は警告と通常学習への復帰。
- 候補ノードのラベルや到達性による選別なし。指定Graphの全ノードが中心候補。65536ノードを超える入力は重点指定を解除。
- OFF時は候補購読・TF追加購読・近傍検索なし。

## 観測統計と公開API

重点更新分はノード位置・実エッジの学習に使用するが、共分散・支持統計・観測方向件数・統計用勝者イベントには計上しない。通常枠の観測統計は従来どおりで、全反復に対する独立観測の重複排除機能ではない。重点配分を増やすと通常枠の統計更新回数は減る。学習位置が変わるため、共分散の値そのものが従来と同一になる保証もない。

CPUライブラリへ`gng_set_priority_input(point_ids, num_points, ratio)`を追加。`gng_setPointCloud`後に、入力済み点の添字をコピーして次の`gng_exec`1回へ適用。空指定は解除、不正添字・配分率は解除して失敗返却。次の点群入力または実行終了で失効。

既存API構造体の変更なし。新ROSノードには更新した`gng_cpu`ライブラリが必要。署名付き配布は通常の再ビルド・署名工程が必要。GPU・WASMのUIへは追加していない。

## 検証

[実行コマンド・結果](../../gng_vlut_system/docs/releases/2026-09-15_grasp_attention.md)を参照。実物把持の成功率改善や実環境の最適な半径・配分率は未検証。
