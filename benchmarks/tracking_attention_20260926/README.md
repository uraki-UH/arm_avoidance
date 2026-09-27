# 粗いセルの追従重点サンプリングの検証

## 1. 要約

2026-09-26開始、09-27確認。密度不足・重心ずれ・近傍非平面支持を共通抽選へ追加。
ノードの既定値はOFF、ユーザーの目視確認依頼により`at128.yaml`はON。
仕様・設定：[CPUサンプリング](../../ais_gng_cpu/docs/sampling.md)。
候補点群は`/downsampling/tracking`。実際の学習点列・重みの可視化とは異なる。
候補生成・失効・固定総学習枠は成功。ただし合成移動物体の平均誤差改善は未確認。
追加の軽量方式98試行・汎用反復実行器は[比較報告](SAMPLING_COMPARISON.md)を参照。本番方式は未置換。

同一合成入力の3試行平均（100,000点、4,000ノード上限、4,000回学習）：

| 重点セル | API全体 ms/入力 | 物体→最近傍ノード平均 m | 同p95 m | 重点学習回数/入力 |
| --- | ---: | ---: | ---: | ---: |
| OFF | 44.260 | 0.102696 | 0.174393 | 0 |
| 0.5 m | 50.216 | 0.105912 | 0.174730 | 1,000 |
| 1.0 m | 50.495 | 0.104534 | 0.173381 | 1,000 |

全体には集計だけでなく、配分変更によるグラフ・探索先の変化も含む。
同一固定入力・固定ノードの集計＋抽選準備だけでは、入力voxel 0.5 mで
OFF 2.616 ms、重点0.5 m 7.753 ms、重点1.0 m 6.720 ms（100回平均）。
入力voxel 0.1 mでは順に3.086 / 8.897 / 7.950 ms。学習・ROS配信は対象外。
追加時間の主体は粗いセル統計・近傍支持の集計。セル幅を粗くしても全点の所属集計は残る。
速度改善としての採用ではなく、効果とコストを比較できる状態での提供。

## 2. 条件・検証

- Docker `gng_cpu_container`、ROS Humble、GCC 11、Release。測定はCPU 4固定。
- 通常配布先2パッケージのビルド成功。CPU開発25対象・製品22対象・ROS関連4対象が成功。
- 負座標、voxel OFF、粗い境界を跨ぐセル、古いID世代、不正設定、固定学習枠を検証。
- OFF同値性：交差点bag80フレーム×前後×2試行、公開グラフ・法線・共分散・イベント等のハッシュ一致。
  API全体平均は変更前42.100 / 41.111 ms、変更後42.387 / 41.638 ms。微小差を改善とは判定しない。
- 合成入力：静止床95,000点＋円筒5,000点、30静止＋70移動フレーム、移動0.06 m/入力。
  入力voxel 0.1 m。既知形状の非平面IDを使用し、実平面抽出器の誤分類は対象外。
  コピーしたライブラリだけ乱数20260926・dt=0.1固定。形状乱数1/2/3、本番ソースの乱数・時刻は維持。
  表の時間・誤差は移動70フレーム平均の3試行平均。品質計算・PythonのID準備は時間外。
- 実ROS：交差点bag先頭60フレーム、入力上限100,000点、入力voxel 0.5 m、上限20,000ノード。
  実非平面所属→組込みAPI→候補点群の受信を確認。最初の候補0点、通常時最大13,350点。
  frame変更・巻戻し・時間ギャップによる末尾4入力の候補0点を確認。
  `enable_pointcloud`未指定時の出力ON、明示OFF時のPublisher不在も確認。
- 実ROSのノード内processing：点群出力OFFでOFF / 0.5 m / 1 mを順序反転した2試行。
  入力20〜54の平均は56.310 / 63.982 / 61.991 ms/入力。DDS待ち・Pythonの送受信は時間外。
  候補出力ありの機能試験と条件を混ぜず、全体の追加時間を記録。
- 交差点の正解軌跡・人車の認識精度・Viewer実描画・長期メモリ上限は未検証。
- 初回の最終ビルドで非平面ライブラリのリンクに失敗。対象シンボルを確認し再ビルドで成功、原因未確定。
  製品試験は開発版の`LD_LIBRARY_PATH`優先で1件失敗。製品パスを先頭指定・実リンク先確認後22件成功。
  既存のNumPy/SciPy推奨版不一致、`node.unknown_learning_rate: 0.3`拒否等の警告を保持。
  警告対象の既存設定・環境は変更せず、同条件で比較。

再現用正本は`verify.py`と`cost.cpp`。生ログ・JSON・変更前コピーは
`artifacts/tracking_attention_20260926/`へ保存（Git対象外）。最終結果は`final_*`、`ros_*_4/5/9_validation`。
開始時の`before/gng_cpu`と`before/at128.yaml`を保持したローカル環境が前提。
前後の決定的コピーは初回`prepare`で作成済み。再実行時は上書きせず新しい`--tag`を指定。
コンテナで`source /ros2_ws/install/setup.bash`、`cd /ros2_ws/src`の後に実行：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 benchmarks/tracking_attention_20260926/verify.py prepare --tag run2_
PYTHONDONTWRITEBYTECODE=1 python3 benchmarks/tracking_attention_20260926/verify.py off_compare --tag run2_
# cell-sizeは0 / .5 / 1、seedは1 / 2 / 3で比較
PYTHONDONTWRITEBYTECODE=1 taskset -c 4 python3 benchmarks/tracking_attention_20260926/verify.py motion --tag run2_ --cell-size .5 --seed 1
# 分離ROS。試験ノードの起動・SIGINT終了はスクリプト内
ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 160 python3 benchmarks/tracking_attention_20260926/verify.py ros --cell-size .5 --seed 9 --validation
# 性能比較は--validationなし、cell-size 0 / .5 / 1、seed 4 / 5で順序を反転
ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 20 160 taskset -c 4 python3 benchmarks/tracking_attention_20260926/verify.py ros --cell-size .5 --seed 4
```

各試験ノードの実コマンド・PID・終了コードは`ros_final_validation.log`と`ros_final_performance.log`。
全試験ノード・ベンチマークは終了、最終ROS7起動はすべて終了コード0。
既存bag・ViewerのPIDと3コンテナのID・起動状態を照合。ユーザーGNGへの停止操作なし。
作業中に利用者が起動したGNGの終了を検出、エージェントによる再起動なし。
コンテナの再作成・ROSデーモン起動なし。
