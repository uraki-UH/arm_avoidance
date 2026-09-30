# ボクセル占有による代表姿勢圧縮の試作・検証

2026-10-01追試: **元の1万ノードGNGにも可動域の大きな未被覆領域あり**。独立した既知到達点の5 cm以内に元GNGノードがある割合は約16〜24%。元GNGとの一致・保持率を可動域全体の被覆保証とは扱わない。[被覆の測定条件・結果](COVERAGE.md)。

評価: ボクセルによる姿勢のグループ化は可能。ただし今回の保存済み双腕モデルでは、4 cmの許容差でほぼ圧縮なし、8 cmで大きな占有近似誤差。GNGの姿勢・辺を代表へ置き換える用途には未適用。厳密な占有共有、または粗い候補抽出後の元占有による再判定が適用候補。

実装範囲: 保存済みGNG/VLUTの抽出、固定された実姿勢代表への割当、群のリンク別集合和の評価、同一リンク占有の可逆辞書化。ROS launch・学習器・元モデルへの変更なし。SpatialTreeのライブラリ組込みではなく、占有に基づく代表選択という方針の独立した試作。

## 入力・測定条件

- 入力: `topo_dual_arm_max`、`topo_dual_arm_max_long`、各10,000姿勢、14関節、2 TCP、2 cm格子。元GNG v9・VLUT v2。
- 比較: 同じリンク番号ごとの双方向ユークリッドHausdorff距離。全リンクが代表から許容差以内のときだけ同じ群への割当。関節角や占有座標の平均なし。代表の固定、連鎖的な合併なし。
- リンク: 22 ID、うち18 IDに占有あり。元設定で除外済みの左右link1などは保証対象外。TCP等の空4 IDも保持。
- 候補削減: 代表のbbox最小座標による格子索引、27近傍bucket、全リンクbboxの必要条件、整数格子上の厳密距離判定。索引候補の生成順整列による全探索と同一のfirst-fit規則。
- 実行: Intel Core i7-14650HX、ホストC++17、`-O3 -march=native -DNDEBUG`、1プロセスずつ、CPU固定なし。既存ROS/Viewer稼働中。
- 回数: 2モデル×4許容差×3処理順seed、24試行。初回予測96秒、実測バッチ73.14秒、24/24成功。品質の独立照合8条件、予測64秒、実測54.09秒、8/8成功。
- 測定区間: 下表の処理時間は索引準備・候補比較・割当。入力読込・bbox作成、群集合和の生成と監査、出力、FK・再ボクセル化は対象外。学習全体の高速化率は未測定。

## 代表選択の結果

値: 3試行の中央値。占有参照削減率: `1 − 群ごとのリンク別集合和セル総数 / 元の姿勢別セル総数`。ファイル容量・GNGノード削減率とは別の指標。元の姿勢と辺は全て保持。

| モデル | 許容差 | 元姿勢数 | 代表数 | 占有参照削減率 | 割当処理時間 |
| --- | ---: | ---: | ---: | ---: | ---: |
| max | 0 cm | 10,000 | 10,000 | 0% | 106 ms |
| max | 2 cm | 10,000 | 10,000 | 0% | 403 ms |
| max | 4 cm | 10,000 | 9,893 | 0.68% | 1,489 ms |
| max | 8 cm | 10,000 | 1,593 | 65.27% | 2,830 ms |
| long | 0 cm | 10,000 | 10,000 | 0% | 117 ms |
| long | 2 cm | 10,000 | 10,000 | 0% | 296 ms |
| long | 4 cm | 10,000 | 9,996 | 0.02% | 720 ms |
| long | 8 cm | 10,000 | 4,831 | 26.35% | 4,174 ms |

8 cm条件の代表数範囲: max 1,592〜1,596、long 4,822〜4,849。時間範囲: max 2,816〜2,871 ms、long 4,146〜4,212 ms。1万姿勢全体の後処理時間であり、ROSの1フレーム時間ではない。

## 圧縮の代償

値: 各条件trial 1を独立Python実装の全点対距離計算で検証。セル中心距離の許容差とTCP誤差は別の量。

| 8 cm条件 | max | long |
| --- | ---: | ---: |
| 代表の占有だけを使った場合の欠落率¹ | 51.60% | 33.93% |
| 群の集合和を使った場合の元占有欠落 | 0セル | 0セル |
| 集合和を各姿勢へ適用した占有量倍率² | 4.51倍 | 2.33倍 |
| 元姿勢と代表の最大関節差³ | 4.134 rad | 4.079 rad |
| 元姿勢と代表の最大TCP位置差 | 9.09 cm | 10.09 cm |

1. 分母: 全入力姿勢のリンク別占有セル参照数。異なる姿勢・リンクの同じ空間セルも別参照として計数。環境障害物を置いた衝突見逃し率ではない。
2. 全姿勢について`群集合和セル数`を加算した値 / 元姿勢セル数合計。辞書に集合和を1回保存する容量とは別。保存量の減少と、姿勢単体の判定に用いる占有の膨張が同時に発生。
3. 原入力角度の絶対差、周期補正なし。大きな関節差があるため、代表への置換による軌道の連続性は未保証。

評価: 8 cmでのグループ化を、そのまま精密な衝突判定や経路ノードの削除へ使うことは見送り。粗い集合和に障害物がなければ群を通過候補へ、重なりがあれば元姿勢の占有で再判定する二段構成が候補。ただし、その構成の実環境での速度・誤衝突率は未検証。

保証対象: 保存済VLUTの格子占有のみ。URDF連続形状、未保存姿勢、辺の補間経路、除外リンク・除外自由度の動作は対象外。元のボクセル化は表面を扱うため、中身まで埋まった立体の体積IoUという解釈は不可。

## 同一リンク占有だけを共有する可逆圧縮

`dedup.py`の`VOXSHAR1`出力を再読込し、全姿勢のID・関節角・TCPと全リンク占有のバイト一致を確認。姿勢数は10,000のまま。GNGの辺は元ファイルに保持し、実験形式には未収録。

| モデル | 比較元VOXPOSE1 | 共有後VOXSHAR1 | 容量削減率 | 辞書構築時間 |
| --- | ---: | ---: | ---: | ---: |
| max | 65,900,456 bytes | 60,018,704 bytes | 8.93% | 89.57 ms |
| long | 72,398,588 bytes | 67,677,856 bytes | 6.52% | 89.89 ms |

条件: 各1回、辞書構築のみ計時。元の`gng.bin + vlut.bin`に対する容量比較ではない。実験用の正規化形式同士の比較。保存済み圧縮ファイルあり、既存VLUTローダーの直接読込みは未対応。

## 検証と成果物

- 合成試験: 94姿勢×3リンク、半径4条件×seed3条件。負座標、空集合、int32座標境界を含む12条件で、索引方式と全探索のCSV一致。独立全点対判定・先行適合代表の見落としなし。
- 実入力: 24試行全てで元占有の集合和からの欠落0。trial 1の8条件では、独立Python実装で全割当を照合。代表数、集合和参照数、関節/TCP最大差もC++集計と一致。
- 元入力: GNG・VLUT・設定・URDFのSHA256非変更を確認。
- 初回バッチ: 圧縮器自体は正常終了、runnerの有限数値辞書制約で`bool`指標を拒否。`run_case.py`で真偽値を0/1へ変換後、24試行を再実行。失敗ログも保存。
- 本体未組込み: `offline_urdf_trainer_dual.launch.py`の挙動変更なし。学習前の大量サンプルに対する圧縮率・GNG学習時間は未測定。

ソース: [compress.cpp](compress.cpp)、[prepare.py](prepare.py)、[verify.py](verify.py)、[dedup.py](dedup.py)、[synthetic_test.py](synthetic_test.py)。小規模集計: [summary.json](summary.json)、[quality_summary.json](quality_summary.json)。

生ログ・24試行のCSV・メタデータ・圧縮ファイル・終了確認: [artifacts/voxel_pose_compression_20260930](../../artifacts/voxel_pose_compression_20260930)。群の集合和は集計のみで、近似圧縮バイナリの保存なし。可逆辞書圧縮バイナリは`dedup_results/*/result.voxshared`。

## 再現と起動コマンド

依存: `g++`、Python 3、NumPy、PyYAML、`run-benchmark-batch` runner。入力モデルは上記2モデル。

```bash
python3 /home/uraki/uraki_ws/benchmarks/voxel_pose_compression_20260930/reproduce.py \
  --root /home/uraki/uraki_ws \
  --output /tmp/voxel_pose_reproduction \
  --repeats 3
```

再現driver: 個別に検証した抽出・ビルド・24試行・独立照合・可逆辞書化の一括入口。既存出力先を拒否、各処理に有限の上限時間、起動コマンドと終了状態を`command.jsonl`へ保存。driver自体の確認は`--help`と構文検査まで。一括入口を通した全工程の再実行は未実施。

今回の起動コマンド（全て終了済み）:

```bash
timeout 180s python3 /tmp/voxel_pose_compression_20260930/prepare.py
g++ -O3 -march=native -DNDEBUG -std=c++17 -Wall -Wextra -Wpedantic /tmp/voxel_pose_compression_20260930/compress.cpp -o /tmp/voxel_pose_compression_20260930/compress
python3 /tmp/voxel_pose_compression_20260930/synthetic_test.py
python3 /home/uraki/.codex/skills/run-benchmark-batch/scripts/run_batch.py /tmp/voxel_pose_compression_20260930/cases_dedup.json --output /tmp/voxel_pose_compression_20260930/dedup_results --repeats 1 --timeout-sec 60 --max-total-sec 150 --estimate-sec 8
python3 /home/uraki/.codex/skills/run-benchmark-batch/scripts/run_batch.py /tmp/voxel_pose_compression_20260930/cases.json --output /tmp/voxel_pose_compression_20260930/results --repeats 3 --start-seed 20260930 --timeout-sec 90 --max-total-sec 600 --estimate-sec 4
python3 /home/uraki/.codex/skills/run-benchmark-batch/scripts/run_batch.py /tmp/voxel_pose_compression_20260930/cases.json --output /tmp/voxel_pose_compression_20260930/results_measured --repeats 3 --start-seed 20260930 --timeout-sec 90 --max-total-sec 600 --estimate-sec 4
python3 /home/uraki/.codex/skills/run-benchmark-batch/scripts/run_batch.py /tmp/voxel_pose_compression_20260930/cases_verify.json --output /tmp/voxel_pose_compression_20260930/verification --repeats 1 --timeout-sec 120 --max-total-sec 600 --estimate-sec 8
timeout 60s python3 /tmp/voxel_pose_compression_20260930/synthetic_test.py --output /tmp/voxel_pose_compression_20260930/synthetic_portable --executable /tmp/voxel_pose_compression_20260930/compress
```

初回合成試験は引数追加前の版。現行版は最後のコマンド形式。過去ログの`/tmp/voxel_pose_compression_20260930`は保存時に上記artifactsへ移動済み。個別子コマンド・seed・PID・終了処理結果は各`report.json`へ保存。新規ROS・Docker常駐プロセスの起動なし、既存ROS/Viewer・コンテナの維持。


## Viewerでの代表姿勢表示

対象: 許容差8 cm、seed 20260931（trial 2）。max 1,593代表、long 4,831代表。学習結果から選んだ実姿勢の表示用コピー。再学習・グループ間経路の再生成なし。

出力: `artifacts/voxel_viewer_preview_20260930/{max,long}/gng.bin`・`vlut.bin`・`preview.yaml`。GNG v9の代表ノード生レコードと元ID、代表同士の元辺、VLUT v2の代表姿勢の元占有を保持。非代表への辺は除外。群全体の集合和占有や群色分けは未収録。

Docker内の起動コマンド（通常のViewerと同じROSドメイン）:

```bash
source /ros2_ws/install/setup.bash
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  params_file:=/ros2_ws/src/artifacts/voxel_viewer_preview_20260930/max/preview.yaml \
  joint_control_backend:=viewer enable_dynamixel_input:=false
```

long版: 上記`/max/preview.yaml`を`/long/preview.yaml`へ変更。ホストからDockerへ入る場合は`docker exec -it gng_cpu_container bash`。

Viewerの対象トピック:

| モデル | 左TCP | 右TCP |
| --- | --- | --- |
| max | `/voxel_max_preview/Tmap_static_L0` | `/voxel_max_preview/Tmap_static_L1` |
| long | `/voxel_long_preview/Tmap_static_L0` | `/voxel_long_preview/Tmap_static_L1` |

`Tmap_static`は左TCPと同じ座標。左右を表示する場合は`_L0`と`_L1`を選択。ロボット名は`voxel_max_preview`／`voxel_long_preview`。元の1万ノードを表示する場合は、元モデルの`gng_vlut_system/config/topo_dual_arm_max.yaml`または`topo_dual_arm_max_long.yaml`を`params_file`へ指定。

検証: ROS_DOMAIN_ID=91、ROS_LOCALHOST_ONLY=1で通常launchを各1回起動。主トピックと左右各層の全代表ID一致、14関節角・両TCP座標の最大差0、全辺インデックス有効、ロボットdescription受信。maxは各1,593ノード・左右806/852辺、longは各4,831ノード・左右3,013/2,912辺。実ブラウザの描画は未確認。

起動した検証コマンド（終了済み）:

```bash
docker exec gng_cpu_container bash -c 'source /ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 && python3 /ros2_ws/src/artifacts/voxel_viewer_preview_20260930/probe_viewer.py /ros2_ws/src/artifacts/voxel_viewer_preview_20260930/max && python3 /ros2_ws/src/artifacts/voxel_viewer_preview_20260930/probe_viewer.py /ros2_ws/src/artifacts/voxel_viewer_preview_20260930/long'
```

検証終了: 全所有launch・子プロセス停止、既存13プロセスと3コンテナの同一性確認。停止時ログの`SIGINT`・`exit code -2`は検証スクリプトの終了操作に対応。停止前のERRORなし。

コード: [export_preview.py](export_preview.py)（CSVから通常形式への抽出）、[probe_viewer.py](probe_viewer.py)（ROS配信照合と終了処理）。入力・出力SHA、抽出時の全生レコード読戻し検査、起動コマンド、ログ、終了確認は[表示用成果物](../../artifacts/voxel_viewer_preview_20260930)へ保存。抽出時の実引数は`export_commands.json`。
