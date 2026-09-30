# GNGの60ms級遅延と周期性の調査（2026-09-30）

## 要約

確認できた60ms超の要因: 処理途中のPコアからEコアへの移動。
PASSIVE試行のframe 821はCPU 14→10→18へ移動し、以後の学習・ラベル・保守・クラスタが一様に増加。
実経過68.088msに対してthread CPU68.042ms、差0.045ms。割当待ちによる26ms増加ではない。
固定コア比較でも同規模入力の平均42.165ms／65.982msを確認。

| GNG処理スレッドの配置 | 計測入力数 | 平均 ms | p95 ms | 最大 ms |
| --- | ---: | ---: | ---: | ---: |
| Pコア4固定 | 443 | 42.165 | 43.353 | 47.636 |
| Eコア18固定 | 409 | 65.982 | 67.979 | 69.826 |

固定周期の一括重処理: 現行コードの有効な経路と今回の時間系列では根拠なし。
3分試行の1,650入力でframe剰余2〜30を確認。5周期の位相平均差0.133ms、自己相関のlag 3以降最大0.082。
ノードの3/6/20回の寿命判定は毎回の個別判定。全体のNフレーム周期ではない。
25回後の認証処理はビルド設定OFF。平面の5相処理・NN分類・接触voxel表示はGNG区間外。

未解決: Pコア11のまま58.405msとなった別の1入力。
thread CPU58.402ms、context switch 0、page fault 0、削除283点（前321／後281）。
voxelは周辺中央値4.076→7.307ms、全voxel照合は23.119→35.309ms。他段階の増加は小幅。
周辺0.5秒採取のCPU周波数2.2GHzでは瞬間的な性能変動の確定不可。SMT・メモリ等の競合も未確定。
従って全スパイクの単一原因化や、Pコア固定による上限保証は不可。
前回の60.03msそのものはCPU番号未記録。今回再現した60ms超の原因との区別が必要。

## 条件・検証

- 対象: Docker `gng_cpu_container`、ROS Humble、通常at128のCPU GNG、既存交差点bagの`/lidar_points`。
- 起動: `ros2 launch ais_gng ais_gng.launch.py backend:=cpu lidar:=at128.yaml input_topic:=/lidar_points`。
- CPU: i7-14650HX、P論理CPU 0〜15／E CPU 16〜23、`no_turbo=1`、powersave、上限2.2／1.6GHz。
- Docker: CPU quotaなし、cpuset 0〜23、スロットリング0。ホスト設定・既存プロセスのaffinity変更なし。
- 計測版: 本番ソースの独立コピーへCLOCK_MONOTONIC／THREAD_CPUTIME／RUSAGE_THREAD／sched_getcpuを追加。
- 本番のC++クラス・公開API・式は不変更、公開29シンボル一致。通常のbuild/installへの反映なし。
- 区間: entry、voxel、全voxel照合、learn、label、check、cluster、finish、total。末尾GNGログまで含む。
- 計測外: profile初期openとCSV書込、ROS側の入力整形・変換・NN分類・平面・曲面・配信・描画。
- 表示GNGとの差: stdout捕捉準備＋診断CSV出力など。平均0.10ms前後、通常割当3試行の最大0.592ms。
- 試行: 通常180秒、詳細通常120秒、PASSIVE120秒、P固定60秒、E固定60秒、各1回。全5試行正常終了。
- 集計: frame 150以降。各行は1入力、p95は線形補間。bag継続再生で条件間の点群フレームは異なる。
- 入力: 通常約16万点・約1.9万GNGノード。bagループ境界の4,352点入力も集計に含む。
- 固定: 新規試験GNGのmain threadだけを起動10秒後から固定。計測対象全入力でCPU4／18を確認。
- 観測: 初回0.5秒間隔、待機方式比較0.05秒間隔、固定比較0.1秒間隔。観測プロセスだけEコア23指定。

待機方式の比較: `OMP_WAIT_POLICY=PASSIVE`、`KMP_BLOCKTIME=0`を新規試験プロセスだけへ指定。
既存OMP/GOMP環境変数なし。Torchの依存先はlibgomp。待機規則の参考: [GCC公式仕様](https://gcc.gnu.org/onlinedocs/libgomp/GOMP_005fSPINCOUNT.html)。

| 指標 | 既定待機 | PASSIVE |
| --- | ---: | ---: |
| 補助スレッドCPU合計（1論理CPU=100%） | 269.30% | 2.56% |
| GNG平均 ms | 42.285 | 42.205 |
| GNG最大 ms | 54.680 | 68.088 |

待機CPUの減少は確認できたが、60ms遅延の解消は未確認。コア配置と待機方式は別要因。
50ms間隔の/proc採取は10ms刻みのCPU時間。SMT兄弟の最終processorだけで厳密な同時実行は証明不可。
停止直前の一部frameはOS観測範囲外であり、補助CPU推定0を不在の証拠として使用しない。
コア固定2条件は共にPASSIVE。通常2分試行との最大値差は、確率的なイベント頻度・入力差も含む。
プロファイルの計測負荷は追加あり、無計測版との差の独立定量化は未実施。
低レベルの命令数・cache miss・APERF/MPERFは未測定。perf制限やホスト設定の変更なし。

実行結果: runnerは1/1、2/2、2/2成功。予測185／246／126秒に対して実180.70／241.53／121.21秒。
解析補助表示は単一コア群の辞書キーで一度失敗、表示側を修正。生データ・試行の成功とは独立。
所有launch・GNG・補助ノード・probeは終了済み。既存bag・TF・Viewerと3コンテナを維持。

根拠: [集計](../../artifacts/gng_jitter_20260930/summary.csv)、[原ログ・CSV・解析](../../artifacts/gng_jitter_20260930/live/)、[終了確認](../../artifacts/gng_jitter_20260930/cleanup.json)。
全起動コマンド・復元手順: [REPRODUCE.md](REPRODUCE.md)。
