# 2026-09-23 Spatial Tree版GNGのボトルネック計測

## 結果

| 処理 | 平均 ms/frame |
|---|---:|
| 木のAABB検索・候補収集（セル照合を含む） | 333.19 |
| 従来順へ戻す候補ソート | 208.12 |
| 候補の距離・ラベル・寿命判定 | 24.65 |
| 検索箱の組立て | 12.88 |
| 木への追加・位置更新・削除の合計 | 1.15 |
| 計測版gng_exec全体 | 652.81 |

検索・収集とソートの合計541.31 msは計測版全体の約82.9%。木の動的更新は主因ではない。上表の内訳は互いに重複せず、全体にはその他処理も含む。

## 理由

getDownSamplingGridは平均79,576.85回/frame、学習側getMinGridは4,000回/frame。合計約83,577回で検索箱作成・木の根からの範囲探索・候補ソート・距離判定を繰り返す。列挙候補は合計約167万件、問い合わせ平均約20件。

従来版は27セルを直接参照。今回の木版は同じ探索範囲を保つため、最近傍専用findNBestではなくquery_aabbの範囲内候補全列挙を使用。根からの再帰走査、セルとの交差判定、葉の座標判定、元グリッドセルによる再照合が必要。

さらに同距離時も従来と同じ評価順を保つためstd::sortを毎回実行。比較関数内でセル添字の除算・剰余・tuple比較を反復。ソート区間全体は実測済みだが、除算単独の寄与率は未計測。木の位置更新は約26,015回でも約1.07 ms/frame。

[検索・ソートの実装](../../../ais_gng_cpu/experimental/gng_spatial_tree/src/cpu/cugng.cpp)と[木の範囲検索](../../../SpatialTree/include/SpatialTree/SpatialTree.hpp)を参照。

## 比較条件と限界

前回と同じbag先頭30フレームを50回入力、先頭10回を除く40回平均。同じat128_snapshot.yaml、乱数・LPF時間刻み固定。両版Release、-O3、-DNDEBUG。CMakeCache.txt・compile_commands.jsonを保存。

計測版の前処理attentionはグリッド92.78 ms→木582.26 ms、学習は5.49→40.57 ms。全近傍探索の包括時間は73.61 ms対591.35 ms。包括時間と上表の内訳は重複するため加算禁止。

タイマーなしの保存ライブラリを再計測すると107.82 ms対614.82 ms。計測版は128.32 ms対652.81 msで、計測負荷・環境変動を含む。上表を未計測版の厳密な内訳と扱わない。各版とも前回およびタイマーなし対照の全50フレームとグラフハッシュ一致。

既存実験版を/tmpへコピーし、そのコピーだけにsteady_clockタイマーを挿入。元cugng・実験版ソース・YAML・インストール先の変更なし。ROS変換・TF・配信・viewerは対象外。既存プロセスと同居したため絶対時間はその負荷を含む。

ソート不要の選択規則と問い合わせ方法の見直しが改善候補。範囲検索だけでも従来版全体を超えるため、ソート削除だけでの逆転は示されていない。これらの改善は未実装・未計測。

## 再現と終了確認

[スクリプト・保存済み集計](../../../benchmarks/gng_spatial_profile_20260923/)。生ログ・ビルド設定の生成ファイルはGit管理外の`artifacts/gng_spatial_profile_20260923/`にローカル保管。以下は旧AABB実装向けの計測手順であり、現行の最近傍版ソースには適用不可。起動コマンド（コンテナ内、全終了済み）:

```bash
python3 /ros2_ws/src/benchmarks/gng_spatial_profile_20260923/instrument.py /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree /tmp/gng_spatial_profile_source
cmake -S /tmp/gng_spatial_profile_source -B /tmp/gng_spatial_profile_build -DCMAKE_BUILD_TYPE=Release -DSPATIAL_TREE_INCLUDE_DIR=/ros2_ws/src/SpatialTree/include -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build /tmp/gng_spatial_profile_build --target gng_grid gng_spatial -j2
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 timeout -s INT -k 5 100 python3 /ros2_ws/src/ais_gng_cpu/experimental/gng_spatial_tree/benchmark.py --library /tmp/gng_spatial_profile_build/libgng_spatial.so --config /ros2_ws/src/benchmarks/gng_spatial_tree_20260923/at128_snapshot.yaml --bag /rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3 --output /ros2_ws/src/artifacts/gng_spatial_profile_20260923/spatial.json 2> /ros2_ws/src/artifacts/gng_spatial_profile_20260923/spatial.profile
python3 /ros2_ws/src/benchmarks/gng_spatial_profile_20260923/summarize.py
```

グリッド計測はlibraryをlibgng_grid.so、出力名をgridへ変更。対照は前回artifactsの各保存ライブラリで実行。計測プロセスは終了、一時コピー・ビルドを削除。既存ROSの停止・再起動なし。
