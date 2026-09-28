# 2026-09-27 - 軽量非平面重点の本番反映

本書は9月27日時点の記録。9月28日の1段拡張の試作・撤去は[続報](ADJACENT.md)。

## 1. 要約

試作`np_points`と同じ、既存最近傍ID＋生成世代の所属照合を組込みAPIへ実装。
`at128.yaml`を`tracking_attention.mode: nearest_nonplane`、配分50%へ変更。保持なし。
粗いセル統計・重心計算・26近傍支持・追加の元点評価を省略し、既存入力セルから点数比例で抽選。
GNGの総学習4,000回、既存の非平面成分抽出、候補トピック`/downsampling/tracking`は維持。

実交差点bag、入力上限20万点・ノード上限2万・60入力×4条件×3試行。
各試行の入力20〜54の平均をさらに3試行平均（ms/入力）。順序は試行ごとに循環：

| 条件 | ログのGNG時間 | ノード全体processing |
| --- | ---: | ---: |
| 追加重点OFF | 39.457 | 63.878 |
| 従来coarse・25%・候補出力OFF | 48.694 | 77.704 |
| 軽量nearest_nonplane・50%・候補出力OFF | 41.267 | 66.285 |
| 軽量nearest_nonplane・50%・候補を購読 | 41.247 | 68.490 |

旧方式比でGNG約7.4 ms減、OFF比では約1.8 ms増。候補配信の全体追加は約2.2 ms。
配分変更によるグラフ形状・後段処理の変化も含み、集計だけの差ではない。
ユーザーの現実行と同じ20万点上限だが、TF・分類等の試験条件が異なるため絶対時間の保証なし。
性能測定12起動＋失効検証1起動が成功。通常配布先2パッケージのReleaseビルドを反映。

## 2. 条件・検証

- 通常CPU CTest 25件、ROS関連5件、外部評価器禁止の製品CPU CTest 22件が成功。
- 軽量規則の追加空間集計なし、固定学習枠、候補取得、voxel OFF、古い世代、未知mode拒否を検証。
- ROSはDOMAIN_ID=183、CPU 4固定。GNG初期化ごとにbag先頭60入力、local座標、voxel 0.5 m。
  平面・非平面抽出ON、人車分類OFF、通常点群可視化OFF。既存ユーザーGNG・Viewerは別domainで稼働。
- 計時はROSノードのGNG欄と全体processing。DDS待ち、起動時間、Python送信・確認は含まない。
  本番乱数・時刻更新は変更なし。性能試行の順序は`production_cases.json`とrunnerへ保存。
- 候補点群は最大133,355点。初回とframe変更・巻戻し・時刻ギャップ後の末尾4入力は0点。
  配信OFF条件のPublisher不在、配信ONの同一ヘッダー受信を確認。Viewer実描画・認識品質は未検証。
- 全ROS試験で通常の`/ros2_ws/build/ais_gng/libais_gng_component_cpu.so`と
  `/ros2_ws/install/gng_cpu/lib/libgng_cpu.so`の実ロードを`/proc/PID/maps`で確認。
- 製品版の初回CTestはテスト有効化不足で対象0件。`GNG_BUILD_BENCHMARKS=ON`へ修正後22件成功。
  既存のコンパイラ警告・Torch kineto警告等は保持。試験後の`git diff --check`成功。
- 公開構造体末尾のmode追加によりAPI利用側も再ビルドが必要。外部任意サンプラーの公開制限は維持。
  ROS既定modeはnearest_nonplane、APIの省略時modeは既存ソース互換のcoarse。機能の宣言既定OFFは維持。
- 稼働中GNGの共有ライブラリ実体を同一内容の別inodeで保護してから通常ビルド。
  旧libの控えは`artifacts/nearest_nonplane_20260927/installed_before.so`。既存プロセスの停止なし。

実行は`docker exec gng_cpu_container bash -lc '…'`内。以下はsource後、workspace `/ros2_ws`から：

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
colcon build --packages-select gng_cpu ais_gng --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Dallow_external_sampler=ON --event-handlers console_direct+
ctest --test-dir /ros2_ws/build/gng_cpu --output-on-failure
ctest --test-dir /ros2_ws/build/ais_gng -R 'test_grasp_attention|test_boundary_attention|test_spatial_sampling|test_nonplane_component_extractor|test_plane_cluster_incremental' --output-on-failure
cd /ros2_ws/src
PYTHONDONTWRITEBYTECODE=1 python3 skills/run-benchmark-batch/scripts/run_batch.py benchmarks/tracking_attention_20260926/production_cases.json --output artifacts/nearest_nonplane_20260927/ros_batch --repeats 3 --start-seed 1 --timeout-sec 150 --max-total-sec 600 --estimate-sec 10
ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 timeout -s INT -k 15 150 taskset -c 4 python3 benchmarks/tracking_attention_20260926/verify.py ros --tracking-mode nearest_nonplane --cell-size .5 --ratio .5 --max-points 200000 --seed 27 --validation --output artifacts/nearest_nonplane_20260927/validation.json
```

runner出力は上書き禁止。再試行時は新しい出力名を指定。実測445.69秒、初期予測120秒は初回実測後に補正。
生ログ・argv・PID・終了コード・cleanupは`artifacts/nearest_nonplane_20260927/ros_batch/`。
集計は同階層`ros_summary.json`、失効は`validation.json`。集計コマンドは`verify.py summarize`。
製品版は同階層`product`へCMake構成（Release、allow_external_sampler=OFF、GNG_BUILD_BENCHMARKS=ON、GNG_ENABLE_FRAME_LOG=OFF）、`-j4`ビルド。
CTestは`LD_LIBRARY_PATH`の先頭にproductを指定し`ctest --test-dir <product> --no-tests=error --output-on-failure`。
製品installは同階層`product_install`へ分離。通常installへ製品版を上書きしない。

全13試験ノード・runner・ビルド・CTestは終了。ROSデーモンの起動・コンテナ再作成なし。
bag PID 293745、Viewer PID 333336、3コンテナを維持。ユーザーGNGは412156→423853への変更を観測。
最終時点の423853は軽量mode・配分0.5、ロード先inodeも新版と一致。エージェントによる既存GNGの停止・再起動なし。
設定・制約の正本：[CPUサンプリング](../../ais_gng_cpu/docs/sampling.md)。過去の合成比較：[比較報告](SAMPLING_COMPARISON.md)。
