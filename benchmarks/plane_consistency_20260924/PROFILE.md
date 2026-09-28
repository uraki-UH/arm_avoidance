# 平面クラスタの高速化・直接入力・中央値選択（2026-09-28）

## 要約

最新は次数5〜8の下側中央値を固定比較網へ変更し、履歴コピーを配列交換へ置換。CPUはGNG配列の借用入力へ接続。
平均19,143.9ノード、5周期・交互5試行のCPU平均中央値は旧版6.701／新版ROS6.251／新版GNG配列6.278 ms。
GNG配列経路で6.3%短縮、p95中央値7.115→6.855 ms。全150入力の全出力一致、所属・ID・幾何・判定条件の変更なし。
段階時間中央値：準備2.058（32.8%）、分断1.216（19.4%）、再推定0.662（10.6%）、生成0.634（10.1%）ms。
所属保守0.614（9.8%）、統合0.597（9.5%）、出力0.250（4.0%）、逸脱0.159（2.5%）、引継ぎ・削除計1.4%。割合の分母は段階中央値の合計。
準備の内訳は読出し0.460／CSR0.330／間隔・補完法線0.941／EMA0.267／種0.059 ms。局所量は先行約1.51 msから短縮。
借用API単独は旧版6.763→新版6.909 msで高速化未確認。他の分類・出力向けROS変換は残存。全面ゼロコピー・完全差分処理ではない。
エッジ距離の共有配列は旧版6.657→新版7.158 msで撤去。重複計算削減よりCSRへの追加書込みが増大。5 ms目標・総ノード数非依存は未達。
先行1パス試作は6.816→6.644 msだが所属0.30%減・面数64.58→66.65、既定OFF。分断事前走査も6.759→6.973 msで撤去。
先行1パス詳細は`artifacts/plane_logic_20260928/final/logic_summary.json`。法線欠損5.84%、必要時補完・巡回検査・世代付き差分通知への接続は未実装。

先行のIDキー連結証明・短次数中央値・ID／ラベル読出し共通化は5周期固定・5試行で7.055→6.777 ms。
p95中央値7.771→7.659 ms、最大12.043→11.633 ms。1試行で平均悪化、2試行でp95悪化。全150出力一致。
連結再利用0→18.02面/frame、旧版はID列不変0/150入力で全体失効。分断条件の緩和なし。
法線の厳密入力キャッシュと分断のUnion-Find置換は追加効果が不安定なため撤去。[不採用判断](../../gng_vlut_system/docs/reject.md)。

可視化削減時の交互3試行は4周期8.171→7.262 ms（p95 7.939、所属12594.63）、10周期7.671→7.256 ms（p95 9.592、所属12604.71）。
全150入力はエッジ座標以外一致、互換出力ONは全フィールド一致。約55,000端点・約0.67 MB/frameの座標複製を除去、所属減少なし。
省略した平面Marker生成は計測外、上記容量は通信全体量ではない。各時点の数値を最新比較と混同しない。
把持側の同期元グラフ要件・表示の復元方法は[仕様](../../gng_vlut_system/docs/releases/2026-09-28_plane_delta_prototype.md)を参照。

可視化削減前の時系列分散は同一入力・交互3試行、先頭50除外。各試行のCPU平均／p95中央値 [ms/frame]：

変更前9.271／毎回8.697／4周期7.509／8周期7.261 ms。p95は9.734／9.302／7.976／7.723 ms。
平均所属12640.02／12640.02／12594.63／12588.29、面数64.87／64.87／65.17／68.38、残差比0.073514／0.073514／0.073826／0.073255。

4周期は変更前比約19%短縮、平均所属点数0.36%減。8周期は追加短縮が小さくクラスタ数増加。
毎回更新は全150入力の全出力ハッシュ一致。周期分散は同じ出力を保証する方式ではない。
4周期の全試行最大CPU時間12.926 ms。5 ms目標・総ノード数非依存・数学的収束保証は未達。
初回周期1、先行時点10、現在5という手元の変更を維持。今回の新旧比較は5に固定、周期変更を高速化に含めない。
初期案は未所属点の取り込みも分散し所属点数3.29%減。毎回取り込みへ戻した最終版を採用。
点数と残差は正解ラベルによる認識精度ではなく、視覚的な品質・動的追従の良否は未確定。

先行計測は別時点の比較：9月27日は11.182→9.818 ms（部分再集計・所属表・平方根・確保削減）。
9月28日初回試作は旧版9.688／試作OFF9.732／差分9.175／ブロック10.052／併用9.334 ms。
初回試作は両方既定OFF。ブロック省略0点で既定ON見送り、現行もOFF。
初回差分ONは所属・表示エッジ一致、全数値の最大絶対差7.63e-6（異種単位、距離誤差ではない）。
固定32点変更の規模比較は5,520／22,080／49,680ノードでOFF0.726／3.007／7.107、
差分0.667／2.994／7.006 ms。各50入力・1試行の探索的値で、全体規模への依存が残存。
先行原資料は `artifacts/plane_optimization_20260927/`、初回試作は `artifacts/plane_delta_20260928/`。
当時の反復・起動コマンド、出力、比較旧ソースは各ディレクトリのログ・cases.json・beforeを参照。

## 条件・検証

- 最新原資料：`artifacts/plane_direct_20260928/final/summary.json`。20/20試行、予測60秒／実46.74秒、cleanup成功。先行`trial1`／`edge_shared`／`network`は各12試行。
- `final_validation`で平面67件と合成6条件×70入力の全出力一致。借用配列の再配置、順序変更、不正接続、欠損法線、NaN、reset・空入力、次数5〜8の中央値を検証。
- 通常Release・install反映、関連147件成功。ROSの従来15チェック＋実点群30入力で内部配列／独立ROSの全出力一致、28入力が非空平面。`ros_retry.log`参照。
- 初回ROS試験は30入力とも空平面で非空条件失敗。試験用GNGセル0.05→0.5 m・間隔0.1 m・学習係数0.08へ修正後に再試験成功。本番YAMLの変更なし。
- 試験CPU510969／511164・平面510983／511177・照合510995／511190は終了コード0。全試験終了・追加デーモンなし、既存bag・Viewer・3コンテナ維持。既存GNGは作業中に503909→516250へ変更を観測、起動停止操作は未実施。
- 先行1パス原資料：`artifacts/plane_logic_20260928/final/logic_summary.json`。20/20試行、予測60秒／実48.25秒、cleanup成功。
- 同`trial1`／`trial2`は各18/18、予測各54秒／実42.66・43.06秒。比較旧版は`before`、不採用の分断試作は`precheck_prototype.cpp`。
- `validation2`では1パスONで平面64件成功、合成6条件×70入力の全出力一致。位置・法線逸脱と一様並進を統計ON/OFFで追加検証。
- 初期`validation1`は65/66成功。分断根拠数0が既存処理で1へ補正される前提を誤った試験1件。生産閾値の変更なし、対象の事前判定自体は性能理由で撤去。
- 通常Release・install反映、関連144件成功。ROSはOFF／ON各15チェック成功、試験CPU505256／505318・平面505268／505330は終了コード0。全試験終了、追加デーモンなし、既存bag・Viewer・GNG・3コンテナ維持。ログは同`build.log`・`ctest.log`・`ros_0.log`・`ros_1.log`。
- 連結再利用の先行原資料：`artifacts/plane_graph_20260928/final/summary.json`。15/15、予測45秒／実36.01秒、cleanup成功。
- `input`／`union`／`local_cache`／`id_cache`は各9試行、`detail`は2試行。最終採用版は`final`、比較旧ソースは`before`。
- `final_validation`で平面63件と合成6条件×70入力の全出力照合成功。中央値、無関係な追加・削除、所属ID置換、並べ替えを追加検証。
- 先行の通常Release・install、関連143件（平面63・非平面6・曲面74）成功。途中の別ターミナルのビルド終了後に再ビルド・再試験。
- ROSドメイン173・5周期で15チェック成功。試験CPU496159／平面496171は終了コード0、起動コマンドは`ros_smoke.log`。追加プロセスなし、既存3コンテナ・bag・Viewer・ユーザーGNGへの停止操作なし。
- GCC/C++17/-O3、rho再利用ON、CPU固定なし。既存bag・Viewer、途中からユーザーGNG稼働。外部負荷・周波数変動の影響は残存。
- GNG学習・ROS publish・通信・描画・読込・照合は計測外。出力構築は計測内。把持の実機連携は未検証。
- 可視化削減原資料：`artifacts/plane_visual_output_20260928/`、最終`trial2/visual_summary.json`。24/24、予測72秒／実63.93秒、関連141件・ROS各15成功。
- 同先行試験の初期60/61失敗は周期別の鎖棄却数を一定とした試験前提。1周期の回数一致と4／10周期の誤平面なしへ修正、閾値変更なし。
- 時系列分散原資料：`artifacts/plane_boundary_20260928/phases_fast_absorb/phase_summary.json`。15/15、予測45秒／実40.12秒、関連140件・ROS14成功。
- 同先行試験の初期59/60失敗は位置・法線の同時変更による面傾斜との混同。独立試験へ修正後60/60成功、閾値変更なし。
- 先行試験のCPU442943／456982／457044・平面442955／456994／457057は停止済み。初回試作138件・先行135件の結果は各原資料参照。
- 世代付き差分API接続、ライブGNG全体時間、正解付き認識品質、任意の巨大座標・長期間運用は未検証。16bit ID照合は従来どおり、系列切替は明示resetが必要。

```bash
# 最新の起動コマンド。gng_cpu_container内、/ros2_ws/src。再実行時は新しい出力先。
timeout 280 python3 benchmarks/plane_consistency_20260924/profile.py prepare artifacts/plane_direct_20260928/final --baseline-source artifacts/plane_direct_20260928/before/plane_cluster_incremental.cpp --direct-comparison
python3 skills/run-benchmark-batch/scripts/run_batch.py artifacts/plane_direct_20260928/final/cases.json --output artifacts/plane_direct_20260928/final/batch --repeats 5 --timeout-sec 35 --max-total-sec 240 --estimate-sec 3
python3 benchmarks/plane_consistency_20260924/profile.py summarize artifacts/plane_direct_20260928/final
timeout 300 python3 benchmarks/plane_consistency_20260924/profile.py validate artifacts/plane_direct_20260928/final_validation --baseline-source artifacts/plane_direct_20260928/before/plane_cluster_incremental.cpp
# ROSとinstallのsetup.bash読込後、/ros2_wsで実行。
timeout 300 colcon build --packages-select ais_gng --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
timeout 120 ctest --test-dir /ros2_ws/build/ais_gng --output-on-failure -R 'test_plane_cluster_incremental|test_nonplane_component_extractor|test_surface_model'
# /ros2_ws/src。子ノードはfinallyで停止、正確な引数はros_retry.log。
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 PLANE_SMOKE_DELTA=1 PLANE_SMOKE_PHASES=5 PLANE_SMOKE_OUTPUT_DIR=/ros2_ws/src/artifacts/plane_direct_20260928/ros_retry timeout -s INT -k 15 130 python3 benchmarks/plane_consistency_20260924/smoke.py
```
