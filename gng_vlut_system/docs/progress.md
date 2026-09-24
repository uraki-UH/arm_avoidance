# 実施済み作業の記録

実際に行った変更・調査・検証だけの記録。予定は [TASK_LIST.md](TASK_LIST.md)、方針未確定の課題は [TASK_CANDIDATES.md](TASK_CANDIDATES.md)、不採用判断は [reject.md](reject.md) に分離。
時間・依存作業などによる保留作業の状態は [pending.md](pending.md) に分離。
記録単位は「日付 / 対象 / 実施内容 / 結果・検証範囲 / 根拠へのリンク」。既存履歴の一括転記なし。

## 2026-09-24: CPU GNG高速化報告の短縮

- ユーザー指定の短い版へ[高速化報告](gng_cpu_optimization_summary_20260924.md)を置換。「報告の要約」「測定条件」の2節に統一し、段階別の詳細表・実験経緯・目標選択の移行説明を本文から削除。
- 最新比較の処理時間と計測範囲を保持し、計測資料は既存の一覧と最新比較へのリンクへ集約。文書一覧・計測資料一覧の案内も同じ報告へ統一。実装・設定・計測データの変更なし。

## 2026-09-24: 目標選択の参照表再利用とbsp3d差分更新

- 特徴量のhash表再構築と、出力ノード抽出の全ノード走査を受信時の参照表へ置換。重複ID・最後の特徴量・元配列順を保持。bsp3dへの移動・追加・削除の差分反映を実装し、座標変化ごとの全索引破棄を廃止。[現行仕様・影響](releases/2026-09-24_goal_selection_efficiency.md)。
- 同一プロセスで7方式・3入力・7更新条件・候補数3条件を3試行比較。ロボット保存座標10,801点・候補20件で、旧8分木＋元選択に対し静的選択1.221→0.254 ms、人工的な全点移動の更新＋選択2.422→0.714 ms。6回受信＋1選択の処理量は1.955→1.756 ms。[全結果・初期費用・遅い条件・起動コマンド](../../benchmarks/goal_selection_efficiency_20260924/README.md)。
- 最終92,610組の出力一致、Release GTest15件・ASan/UBSan、差分索引1,440範囲の全走査照合、隔離ROSの8ケースが成功。通常Releaseビルド・インストール済み。前段2比較の各37,800組も一致し、最終条件とは別保存。
- 初回比較の共有メッセージ破棄を最後の方式だけが負担する偏りを修正。過去の別プロセス比較の微小な全体増加をbsp3d単独の退化とは断定せず、今回の同一プロセス結果へ判断材料を更新。初期構築増加と、重複の多い合成平面の全点移動で旧版より遅い条件を明記。
- 起動した比較・ビルド・試験・ROSノードは全終了し、専用一時ビルド・ROSログを削除。開始前のROS・bag13プロセスと3コンテナを維持。元CPU GNGの学習・YAML・スレッド構成の変更なし。

## 2026-09-24: 共通SpatialTreeのbsp3dへの移行

- ユーザー指示に従い、目標選択をdoubleのbsp3dへ移行。閉区間AABB検索を追加し、索引再利用・TF逆変換・セル所属判定・順位を保持。共通SpatialTree22ファイル、GNG旧8分木方式、bsp3d内の重複実装と旧生成スクリプトを削除。[仕様・影響](releases/2026-09-24_bsp3d_migration.md)。
- 実GNG18,729座標・3試行で、範囲検索1024回は0.664→0.461 ms、構築は1.371→4.420 ms。目標選択全体は約0.075〜0.116 ms増加。選択結果7,920組は全走査基準と一致。試作の一括構築は遅くなり[不採用](reject.md)。[比較条件・全測定値](../../benchmarks/bsp3d_migration_20260924/README.md)。
- Releaseのbsp3d CTest2件、GNG比較版CTest18件、通常目標選択GTest12件が成功。AABB37,600比較とASan/UBSan成功。通常gng_vlut_systemをビルド・インストール。旧fixture不在・初回GTestリンク不一致を修正し、colconによる計測setup.py誤検出にはbenchmarks/COLCON_IGNOREを追加。実ROSの目標選択による再生試験は未実施。
- 今回の比較・ビルド・試験は全終了し、専用一時ビルドを削除。既存ROS・bag13プロセスと3コンテナの状態を維持。元CPU GNGの学習実装・YAML・起動先の変更なし。

## 2026-09-24: CPU GNG高速化の結果と内訳の文書化

- 本番採用5段階と独立tree・最小版の既存検証を整理し、[高速化比較報告](gng_cpu_optimization_summary_20260924.md)を追加。効果を確認した実装、効果が限定的な実装、採用見送りの理由、方式による品質・機能差を区別。
- 保存済みJSONから同一区間の前後時間を再集計。段階別の全体・詳細内訳、集計条件・元データへのリンクを記載。直近2段階の共通項目は既存report.jsonと照合。累積高速化率や起動待ち短縮の未計測値は実績に含めず、既存履歴を保持。
- SpatialTree整理の追加調査で、旧GNG比較パッケージ内のbsp3d版同居、目標選択のAABB範囲検索、ヘッダ生成元の参照を確認。整理の推奨範囲と未実施の区別を報告へ追記。
- 本書作成によるGNG実装・パラメータ変更、再計測、ROS・再生の新規起動なし。

## 2026-09-24: GNG再起動後の入力待ち調査

- 既存プロセス・ログ・bagのSQLiteを読取調査。GNG PID 128595は起動から0.436秒でコア初期化完了ログ、20.512秒後に最初の点群処理内TF警告。根拠はコンテナ内 `/tmp/roslog/2026-09-24-14-44-39-100631-uraki-ILeNEi-15FX165-128594/launch.log` と `/tmp/roslog/ais_gng_cpu_128595_1790228679169.log`。分類モデル読込完了時刻の独立計測なし。
- 再生中の交差点bagは `/lidar_points` のみ6,005件、先頭付近の1件約4.16 MB、約0.1秒間隔。導入済みros2bagの先読み既定値1,000件と、再生PID 128617のRSS 4,233,580 kBを確認。再生開始前のキュー充填待ちは[Humbleの公式実装](https://github.com/ros2/rosbag2/blob/humble/rosbag2_transport/src/rosbag2_transport/player.cpp#L187)で確認。約4.2 GBの先読みが起動待ちの有力原因との判断。先読み件数変更後の時間比較は未実施。
- `map` と `hesai_lidar` のTF不足も確認。現行処理は変換失敗を捕捉して点群処理を継続するため、警告だけでは起動停止を意味しない。[該当処理](../../ais_gng_cpu/src/ais_gng/src/ais_gng_component.cpp)。実行コード・設定の変更、ROS・再生の新規起動、既存プロセスの停止なし。

## 2026-09-24: CPU GNGの全体走査削減

- 探索用配列の同期再利用、孤立候補管理、実在エッジプールの単一走査を本番採用。保守区間はvoxel 0.1 mで2.184→1.565 ms、0.5 mで1.869→1.284 ms。全体は0.5 mで33.315→32.807 ms、0.1 mでは93.338→94.004 msで改善未確認。[条件・実装・起動コマンド](releases/2026-09-24_gng_incremental_updates.md)。
- 比較用7,630フレーム、5,450組の全出力一致。追加回帰3,696比較とASan／UBSan、本番CTest21件＋API2件、WASM native 1件成功。通常乱数・実時間の実入力30フレームも成功。
- 実際のcbは30パッケージ、58.4秒で成功。公開API・YAML変更なし。既存GNG等12プロセスと旧ライブラリ参照を維持し、原子的に配布。検証プロセス全終了、今回の一時ビルド3箇所を削除。

## 2026-09-24: CPU GNGの追加4候補の検証と3変更の採用

- 探索用連続配列、重点候補コピー削減、クラスタ管理、疎エッジ保存を個別・合成比較。探索・クラスタ管理・疎エッジの3変更を本番採用。全体時間の3試行中央値は0.1 mで101.209→91.937 ms、0.5 mで36.050→32.565 ms。[詳細・起動コマンド・限界](releases/2026-09-24_gng_followup_efficiency.md)。
- 本比較7,630組と事前probe800組で全照合項目一致。学習4,000回・全点照合・YAMLと公開C APIを維持。疎エッジの予約配列は上限2万ノードで約2 GB→1.8 MB。
- 本番CTest20件・API2件・WASM native1件、実時間版の実入力30フレームが成功。cbは30パッケージ56.1秒で成功。重点候補コピー削減は学習側の増加を確認し[採用見送り](reject.md)。
- 検証前の保存記録と終了時はViewer＋bagの9プロセスで一致。初回読取のGNG3プロセスは保存ベースライン時点で既に存在せず、停止操作は実施なし。次回GNG起動で新ライブラリが有効。結果・環境確認は[検証資料](../../benchmarks/gng_followup_efficiency_20260924/README.md)。

## 2026-09-24: 自動承認の既定設定とgoal作業規約

- ユーザーの明示依頼で個人config.tomlをバックアップし、approval_policy=on-request、approvals_reviewer=auto_review、sandbox_mode=workspace-writeを設定。TOML解析と既存設定保持を確認。
- AGENTS.mdへgoal中の自律実行、重複確認の省略、審査回避禁止、破壊的操作禁止を追加。既存チャットへの即時反映は未確認。[公式設定](https://learn.chatgpt.com/docs/sandboxing/auto-review)。

## 2026-09-24: CPU GNGの法線・曲率処理の効率化

- 座標・法線の連続配列参照と隣接差分の再利用を本番へ反映。法線・曲率・ラベル部分は、voxel 0.1 mで5.588→4.697 ms、0.5 mで3.988→3.497 ms。約16%／12%短縮。全体改善は小さく試行間の揺れあり。[内訳・限界・起動コマンド](releases/2026-09-24_gng_normal_efficiency.md)。
- 計2,660フレーム、基準再利用を含む1,410組で法線・曲率・グラフ等が一致。拡張機能90組で観測・統計・重点入力・イベント・差分も一致。学習4,000回、毎フレームの全法線更新を維持。
- Release＋LTOのテスト計70件とインストール済み実時間版30フレームが成功。追加回帰テストの登録漏れ・環境読込漏れは修正し、実行件数を確認。実際の`cb`で全30パッケージが57.3秒で成功。
- 本番ライブラリを原子的に反映。公開API・YAML変更なし。検証プロセス全終了、一時ビルド削除済み。既存ROS関連12プロセスは前後不変。

## 2026-09-24: 基数ソートの複数シード評価と本番CPU採用

- 6シード、voxel幅0.1／0.5 m、各方式200フレームで比較。全体時間108.239→101.569 ms／39.828→35.947 ms、6.16%／9.74%短縮。0.2 m被覆率の平均差+0.0894／−0.0588ポイントは、従来のシード間標準偏差より小さく、本番の入力ソートを基数ソートへ変更。[条件・結果・起動コマンド](releases/2026-09-24_gng_radix_multiseed.md)。
- 計5,040フレームで4,000回学習を維持。2,400組で全元点のセル対応等が一致。観測ON/OFF・同一セルの代表点変更の240フレームでグラフ等が一致。観測角度の変化と学習への影響を切り分け。
- Release＋LTOのテスト計51件、インストール済み実時間版の実点群30フレームが成功。引数なしの実際の`cb`で30パッケージが55.6秒で成功。公開API・YAML変更なし。
- 本番ライブラリを原子的に反映。検証プロセス全終了、一時ビルド削除済み。既存ROS関連9プロセスは前後不変。

## 2026-09-24: 本番CPU相当の基数ソート比較

- 本番の全点照合・重点学習・観測・クラスタを保つ独立コピーで、Boostと32bit安定基数ソートを比較。300フレームで0.1 mは108.996→99.970 ms、0.5 mは39.746→35.396 ms。ソート単体58〜75%短縮、3試行とも改善。[実装・条件・起動コマンド](releases/2026-09-24_gng_radix_production.md)。
- 全1,090組・計2,180フレームで全点のセル対応と学習4,000回を維持。一方、セル内元点順の変更により初回から学習点とグラフが変化。0.2 m被覆率の平均差は+0.361／+0.066ポイント。本番への採用は結果保持条件を満たさないため見送り、比較版を保存。
- Releaseのテスト34件成功。voxel無効の30フレームは全出力一致、比較用Boostは前回本番相当600フレームと一致。本番53ファイル・共有ライブラリ不変。通常colconは30パッケージ、検証コピー混入なし。
- 検証プロセスは全終了、一時ビルド削除済み。既存ROS関連9プロセスは前後不変。

## 2026-09-24: 検証用GNGコピーによるcbの重複エラー修正

- 前回保存した比較用3コピーが通常版とともに`gng_cpu`として検出され、`cb`が失敗する状態をログで確認。`artifacts/COLCON_IGNORE`と比較準備時の除外マーカー生成を追加。
- `colcon list`で通常版1件・保存領域由来0件を確認。実際の引数なし`cb`で全30パッケージが55.5秒で成功。ビルドプロセスは全終了、既存ROS関連9プロセスは不変。[原因・変更・起動コマンド](releases/2026-09-24_colcon_artifacts_exclusion.md)。

## 2026-09-24: 本番CPU GNGの結果保持型の効率化と実験版の基数ソート採用

- 本番へ最小空きID探索の開始位置保持と入力voxel重心の走査統合を適用。ソート・加算順、入力、学習、全点照合、既存機能を維持。独立実験版の基数ソートを標準ONへ変更。[変更・条件・起動コマンド](releases/2026-09-24_gng_production_efficiency.md)。
- 本番変更前後1,190組・計2,380フレームでグラフ、クラスタ、元点対応が一致。うち90組は重点入力・観測・統計・イベント・差分も一致。学習4,000回を維持。連続300フレームで0.1 mは110.877→107.034 ms、0.5 mは52.159→39.348 ms、3試行共通区間の中央値でも短縮。
- Releaseのテスト計68件成功。通常ビルドから本番ライブラリを原子的に反映し、インストール先の実時間・非固定乱数版で実点群30フレームの動作を確認。比較専用APIの混入なし。
- 検証プロセスは全終了、一時ビルド削除済み。外部で既存GNG関連3プロセスが終了、Viewer・bag再生は継続。エージェントによる既存プロセスの停止・再起動操作なし。

## 2026-09-24: 学習量を維持したGNGの実装高速化

- 前回の最小比較版を独立コピーし、空き番号管理2種・LTO・重心走査統合・基数ソート・CPU向け命令選択を計8構成で比較。標準構成は開始位置管理＋LTO＋重心走査統合。本番・コピー元・bsp3dのハッシュ不変を確認。
- 連続300フレームで、生点群tree 11.20→6.43 ms、grid 9.79→5.45 ms。voxel版は31.6〜35.8%短縮。CPU 0固定の確認でもtreeの生点群42.6%、voxel 0.1 m31.5%短縮。[実装・検証・起動コマンド](releases/2026-09-24_gng_runtime_trials.md)。
- 136実行・10,800フレームで学習検索4,000回、入力・候補・原点選択数が一致。基数ソートを除く8,850フレームでグラフ・ノード更新回数が一致。基数ソートはさらに高速だが丸め差と一部構造差があり任意オプションで保持。
- ReleaseのCTest計116件、最終2構成34件の再検証が成功。最終ソースの再ビルドと測定済みライブラリがバイト一致。計測・テスト全終了、一時ビルド削除済み。既存プロセスは作業中に外部で変更され、前後差分を記録。今回の作業による停止・再起動操作なし。

## 2026-09-24: 最小GNGの入力ボクセル・node.grid比較

- 最小tree版の独立コピーへ入力voxelと従来27セル探索の切替を追加。Release 4ライブラリ、CTest 19/19件成功。本番・コピー元ソースと本番ライブラリのハッシュ不変を確認。
- 同じbagの6条件×3試行で、生点群tree 11.30 ms／grid 11.15 ms、入力voxel 0.1 mは26.37／24.98 ms、0.5 mは22.47／21.37 ms。入力整理は生点群約1 ms、voxel約9〜12 ms。グラフ規模と被覆率の差も保存。[実測・条件・起動コマンド](releases/2026-09-24_gng_minimal_comparison.md)。
- 最近傍検索は全900フレームで4,000回、全点事前探索0回。試行間グラフ900/900一致、tree_rawとコピー元の保存済みグラフ50/50一致。
- 計測・テストプロセスは全終了、一時ビルド削除済み。既存ROS関連10プロセスのPID・親PID・コマンド不変。本番差替えなし。

## 2026-09-24: Viewerへの人・車の確定分類表示

- `uraki_ws/ToPoFuzzy-Viewer`で確定分類の所属ノード色、Human／Car件数、Clusters切替を追加。環境GNGの所属添字を受信時にIDへ正規化し、他Graph・幾何ラベル・既存表示設定を保持。
- 所属誤対応の修正前失敗と修正後成功、Frontend13テスト・lint・本番ビルド、Backendビルド・機能3テストの成功を確認。追加の全CTestでは未変更Backendの書式等6チェックが失敗。[仕様・検証範囲・コマンド](releases/2026-09-24_viewer_human_car_display.md)。
- 起動中ViewerのHTTP配信へ変更反映を確認。ROS・bag・Webサーバーの新規起動と既存プロセスの停止／再起動なし。検証プロセスは全終了、実ブラウザの目視確認は未実施。

## 2026-09-24: ボクセル化なしの最小tree版GNG

- 前回実験版の独立コピーに、YAML範囲内の元点から直接4000点を選んで学習する経路を追加。入力ボクセル・観測セル索引・事前照合・重点候補生成・クラスタリングを撤去。本番・コピー元のソースと本番ライブラリのハッシュ一致を確認。
- ReleaseのCTest 5/5件成功。同じbagの3試行平均は元CPU208.30 ms、前回実験版136.32 ms、最小版21.40 ms。ただし原点以外の0.2 m被覆率は76.75%→52.34%、平均ノード数19879.9→8441.0へ低下。機能・品質が同一の比較ではない。[仕様・条件・結果・起動コマンド](releases/2026-09-24_gng_bsp3d_minimal.md)。
- 最小版の連続300フレームは平均18.75 ms、95パーセンタイル26.30 ms。各方式の3試行間グラフ各150/150一致、未使用voxel設定違いのグラフ50/50一致。最近傍探索は学習4000回だけ。再現資料と集計を保存。
- 計測・テストプロセスは全終了。ROS・bagの新規起動、本番への差替えなし。

## 2026-09-23: CPUの人・車分類結果の反映と確定

- 分類器からの返却を生成フレーム番号からクラスタ年齢へ修正。保持期間内の確定へ条件を修正し、期限切れ・クラス切替時の確認回数リセットと重複加算防止を追加。モデル・YAMLしきい値の変更なし。
- 修正前のAPI・分類器テスト失敗を再現。修正後のCPU CTest 13件、既存API 2件、分類器GTest 3件、launch 15件、所属情報のROS回帰検証が成功。
- 通常インストール先の修正版と廊下bagで、非空マップ120フレーム・人の確定ラベル延べ174件を確認。車の確定はCPUテストで確認、実データの認識精度は未評価。[仕様・検証条件・起動コマンド](releases/2026-09-23_gng_cluster_labels.md)。
- 利用者側で再起動されたGNGの使用ライブラリと更新済みファイルのinode一致を確認。検証用GNG・bag・転送・購読・ビルドプロセスは全終了、一時ビルド領域16MBを削除。既存プロセスへの停止・再起動操作なし。

## 2026-09-23: ViewerのHuman・Car分類色

- 共通分類色をHumanは赤紫`#d946ef`、Carは青紫`#8b5cf6`へ変更。通常分類のノード・クラスタ・凡例へ反映。
- Frontend lint、コンテナ内の本番ビルド、既存描画テスト2件が成功。起動中Viewerの配信色をHTTPで確認。ホストのビルド権限エラーは既存コンテナ内の再実行で解消。[変更範囲・検証コマンド](releases/2026-09-23_viewer_human_car_colors.md)。
- 検証プロセスは全終了。ROS・再生・Webサーバーの新規起動、既存プロセスへの停止・再起動操作なし。

## 2026-09-23: クラスタリングOFF時の通信口抑止

- 平面OFF時の可視化起動・非平面出力・保存購読、曲面OFF時の時間購読を抑止。Viewerの補助平面購読を発行元の存在に連動。作業途中のNULL参照によるGNG停止も修正。
- launchテスト15件、Viewer CTest 3件、非平面の実ROS/WS配信、曲面時間ログ、クラスタ所属の回帰検証が成功。隔離ビルド・通常インストールの両方でOFF→平面ON→曲面ON→OFFと、全構成での非空マップ継続更新を確認。[仕様・検証範囲・起動コマンド](releases/2026-09-23_gng_clustering_topics.md)。
- CPU・ViewerバックエンドのビルドとFrontend lint/buildが成功。通常起動先へ反映、検証プロセスは全終了。既存プロセスへの停止・再起動操作なし。通常環境の確認時点ではGNG発行元が0、実bagでの再起動後動作は未確認。

## 2026-09-23: bsp3d版GNGの固定グリッド撤去と照合回数制限

- 既存tree版を独立コピーし、固定ノードグリッドと1セル10ノード制限を撤去。元点による観測寿命確認・重点候補選別・回数制限付き全域探索を分離。元CPU版・既存tree版・bsp3d本体への今回分の編集なし。
- Releaseの短い3試行比較で、入力voxel 0.1 mの平均112.84→76.03 ms、0.5 mは53.46→73.61 ms。連続300フレームの0.1 mでは114.94→83.47 ms（27.4%短縮）、95パーセンタイル128.10→133.61 ms。原点以外の0.2 m被覆率75.71→77.16%、0.4 m被覆率95.36→95.22%。入力からの原点除外なし。
- CTest 19/19件成功。新2版で`node.grid`を0.001・0.5・1.0に変えたグラフ照合200/200フレーム一致。最終整形後と計測時のライブラリSHA-256一致。[仕様・条件・結果・起動コマンド](releases/2026-09-23_gng_bsp3d_sampled.md)、[再現資料](../../benchmarks/gng_bsp3d_sampled_20260923/README.md)。
- テスト・計測プロセスは全終了。通常ROSへの組込みなし。既存ROS・Viewer・再生への停止や再起動操作なし。

## 2026-09-23: CPUクラスタ所属情報の受け渡し修正

- 所属配列を構築しても個数を0で返す箇所を修正。ROSクラスタへの所属転送と人・車分類器の入力経路を復旧。YAML・しきい値・法線計算の変更なし。
- 修正前の回帰テスト失敗、修正後のコアCTest 11件・既存API 2件・分類器GTest 3件の成功を確認。インストール前後の隔離ROSで各5フレーム・最大所属数120・累計推論結果10件を確認。実環境の認識精度は未検証。[原因・変更範囲・起動コマンド](releases/2026-09-23_gng_cluster_members.md)。
- 通常CPUライブラリをReleaseビルドしてインストール先へ反映。検証用プロセスは全終了。既存Viewer・再生・GNGの停止や再起動なし。

## 2026-09-23: 実験生成物と再現資料の分離

- 利用者の承認に基づき、`artifacts/`全体をGit管理対象外へ変更。109件のインデックスを解除し、うち再現コード・設定・集計・照合資料24件を[benchmarks/](../../benchmarks/README.md)へ移管。残る生ログ・生データ85件は実ファイルを保持、SHA-256の前後一致を確認。
- 関連文書と設定参照を更新。再集計の出力先は`artifacts/`とし、Git管理中の集計を上書きしない構成へ変更。移動前後の集計コード2件で出力一致、Python 5件・Shell 5件の構文確認、JSON 9件の読込確認に成功。旧SpatialTreeの保存済み集計には元コードの出力対象外の対照計測項目があり、その値も保持。
- 設定スナップショットのハッシュ維持のため既存の末尾空白も保持。それ以外の対象差分の空白検査に成功。実GNG計測・ROS起動は未実施、既存プロセスへの操作なし。

## 2026-09-23: 残存ビルド生成物のステージ解除

- [.gitignore](../../.gitignore)の除外対象を実験保存先のライブラリ・CMakeキャッシュ・コンパイル情報・ビルドログへ拡張。追加26件をインデックスのみから解除。
- 26件の実ファイルはSHA-256の前後一致を確認。他のステージ済みパスは維持し、計測JSON・再現スクリプト・テストログは除外対象外。ステージ内のバイナリと、除外規則に該当する追跡済み実験ファイルの残存なし。

## 2026-09-23: bsp3dビルドログのGit管理除外

- [.gitignore](../../.gitignore)へbsp3d計測ディレクトリ内のビルドログ除外規則を追加。ステージ済みの`build.log`2件と`sampled_build.log`1件をインデックスから解除。
- `git check-ignore`で3件の除外を確認。実ファイルのSHA-256は前後一致、ソース・計測結果JSON・テストログは維持。ROSプロセスの起動・停止なし。

## 2026-09-23: GNG launchの入力トピック上書き修正

- センサー設定を同一セレクターの辞書へ統合し、明示入力引数の末尾適用へ変更。YAMLの既定トピックへ戻る不具合と、同じ原因のrho・短名優先順位を修正。
- 実ROSのパラメータ解決によるテストへ置換し、修正前5条件の失敗と修正後11件の成功を確認。指定コマンドで実点群の継続計算と非空マップ更新を確認。[原因・検証範囲・起動コマンド](releases/2026-09-23_gng_input_topic_override.md)。
- 検証用launch・子ノード・プローブは全終了、一時ログを削除。既存Viewer・bag再生と、作業中に別ターミナルで起動されたGNGは維持。


## 2026-09-23: bsp3d版GNGの処理別計測

- 一時コピーのRelease計測で、全ボクセル点の照合を含む前処理61.01 msが全体95.78 msの約64%と確認。最近傍探索は平均83,576.85回／フレーム、その約95%が学習前の照合。
- ボクセル処理10.32 ms、法線・ラベル9.86 ms、学習5.10 ms。無計測対照95.37 msと確認し、詳細計時の負荷増大を段階計測・標本計測と区別。全4条件で前回保存グラフと各50/50フレーム一致。[内訳・制約・起動コマンド](designs/gng_bsp3d_profile_20260923.md)。
- 計測セッションは全終了。実装・YAML・稼働中ROSへの今回分の変更なし。

## 2026-09-23: GNG観測APIのヘッダー分離

- 通常CPU版・実験版の観測用構造体と関数宣言を`observation_api.h`へ分離し、実装・ROS利用側・テストのincludeを更新。データ構造・計算処理・設定の変更なし。
- ヘッダー単独利用とinclude順の8コンパイル確認、基本APIの依存除去、CPU・ROSコンポーネントのReleaseビルド、CTest 17件が成功。[変更範囲・互換性・検証コマンド](releases/2026-09-23_gng_observation_api_header.md)。
- 検証プロセスは全終了、一時成果物を削除。通常のインストール先は未更新、既存ROS・再生の停止や再起動なし。

## 2026-09-23: bsp3dを使う独立GNG版の比較

- 独立コピーに`bsp3d::Index`を使う共有ライブラリを追加。GNG処理・YAML・厳密2近傍の条件をそろえ、元CPU版とbsp3d本体への変更なし。
- Releaseテスト27件成功。同一bagの3試行平均で、固定順8分木112.04 ms→bsp3d95.50 ms（約14.8%短縮）。グリッド103.94 ms比でも約8.1%短縮。8分木版とのグラフは150/150フレーム一致。[仕様・実測・起動コマンド](releases/2026-09-23_gng_bsp3d.md)。
- 検証・計測プロセスは全終了。通常ROSへの組込みなし。比較ライブラリ・生JSON・設定・テスト結果を保存。

## 2026-09-23: Spatial Tree版GNGの範囲検索・並べ替え廃止

- 利用者指示に従い、独立コピーを木全体の最近傍2ノード探索へ変更。候補列挙・候補ソート・子セルの並べ替えを廃止し、固定順と第2近傍距離による枝刈りを実装。
- Releaseテスト17件成功。同一bagで旧版617.50 msから112.51 msへ短縮。グリッド版104.26 msに対しては約7.9%長い結果。汎用2近傍版とのグラフは150/150フレーム一致。[挙動変更・計測条件・起動コマンド](releases/2026-09-23_gng_spatial_nearest.md)。
- 検証・計測プロセスは全終了。元GNG・共通SpatialTree・YAMLの今回分の変更なし、ROSへの組込みなし。

## 2026-09-23: Spatial Tree版GNGの処理別計測

- 一時コピーのRelease計測で、範囲検索・候補収集333.19 ms、候補ソート208.12 msが全体652.81 msの約83%と確認。木の追加・移動・削除は合計約1.15 ms。
- タイマーなし対照はグリッド107.82 ms・木614.82 ms。両版とも前回・対照の全50フレームとグラフ一致。実装・YAML変更なし、計測プロセスは終了し一時ビルドを削除。[詳細・起動コマンド](designs/gng_spatial_tree_profile_20260923.md)。

## 2026-09-23: センサー別YAMLからのPl・Curve切替

- センサー別YAMLを共通設定より優先し、平面・非平面設定をCPU直結側、曲面設定を別ノードへ転送。at128.yamlへ既定OFFの2項目を追加。
- launch展開テスト6件が成功。インストール先のソース参照を確認。テスト終了済み、ROSノードの新規起動・既存プロセスの再起動なし。実点群による計算結果は未検証。[設定手順・検証範囲](releases/2026-09-23_gng_clustering_yaml.md)。
- 利用者指定の`plane_clustering`・`curve_clustering`へYAML切替名を変更。内部設定への変換と旧名との優先順位を追加し、拡張後のlaunch展開テスト9件が成功。既存設定値を維持、試験終了済み。

## 2026-09-23: Spatial Tree版GNGの独立比較

- 利用者指示に従い現行GNGを独立ディレクトリへコピー。元cugngへの今回分の一時変更は除去し、従来修正は保持。SpatialTreeのAABB探索・動的索引更新を実装。
- APIテスト16件成功。同じbag・設定の3試行平均でGNG本体110.37 ms対617.02 ms。比較グラフは150/150フレーム一致、今回の木版では高速化なし。[条件・結果・再現手順](designs/gng_spatial_tree_20260923.md)。
- 計測プロセスは終了、一時ビルドを削除。独立ソース・比較ライブラリ・生ログを保存。稼働中GNG・通常ライブラリ・YAMLの変更なし。

## 2026-09-23: CPU GNGボトルネック報告書の整理

- 既存計測記録へ結論、前処理の役割、根拠コード、計算量と二乗メモリの区別、未検証の改善候補・比較条件を追記。READMEからの参照を追加。
- 当時計測と後続のコード・設定・実験版を区別。新規性能計測、ソース・設定変更、ROSプロセスの起動なし。[報告書](designs/gng_runtime_cost_20260923.md)。

## 2026-09-23: CPU GNGの長期記憶寿命の設定化

- `node.static.s1_age_max`をYAML・コア・削除判定・動的ROS更新へ接続。既定100を維持し、固定値判定を廃止。
- CTest 10件、寿命1の追加試験、隔離ROSでの起動・動的変更・無効値拒否が成功。旧ライブラリ参照による初回失敗は参照先修正後に再確認。ビルドとインストール先反映済み、試験ノードは全終了。既存ROSの停止・再起動なし。[変更範囲・検証コマンド](releases/2026-09-23_gng_static_age_parameter.md)。

## 2026-09-23: CPU GNGパラメータ反映修正

- ボクセルサイズ0の間引き無効化、末尾セル欠落・空入力処理、動的学習設定の内部同期、CPU ROS変更の事前検証を実装。YAML値は未変更。
- CTest 7件・既存API 2件・隔離ROS設定変更検証が成功。ライブラリ更新・CPUコンポーネントの再ビルド済み。試験ノードは終了、既存ROSの停止・再起動なし。実bag・viewer未検証。[詳細と起動コマンド](releases/2026-09-23_gng_parameter_application.md)。

## 2026-09-23: CPU GNGの実行時間調査

- 約16万点・約2万ノードの同一入力による比較で、GNG全体103〜107 msのうち全ボクセル近傍照合を含む前処理が74〜76 ms。学習回数4,000→1,000では全体差が通常変動幅内、ノード上限半減では約73 msだが表現量も変化。
- 関数別の標本計測で`getDownSamplingGrid()`が主要因と確認。実運用コード・設定の変更なし。計測プロセスは終了、既存ROSの停止・再起動なし。[条件・実測・制約](designs/gng_runtime_cost_20260923.md)。

## 2026-09-23: CPU GNGの空間偏り修正

- ボクセル順のノード生成で低いZ側が上限を消費する現象を二層点群と実bagで再現。全候補の処理順を分散し、YAML・点群フィルタの変更なしでX上端約45 mから79 mへ改善。
- CTest 5件・イベントAPI・差分APIが成功。共有ライブラリ更新とインストール後比較まで実施。実ブラウザ未検証。試験プロセスは終了、既存ROSの停止・再起動なし。[検証記録](releases/2026-09-23_gng_spatial_coverage.md)。

## 2026-09-23: CPU直結平面クラスタリングの既定無効化

- YAMLへ`plane_cluster.direct_enabled: false`を追加。初期化・更新のスキップ条件と依存する非平面抽出の停止条件を確認。既存プロセスの再起動・性能計測は未実施。[変更範囲と検証](releases/2026-09-23_disable_plane_default.md)。

## 2026-09-23: 曲面検出の既定無効化

- 共通設定の`surface_model.enable`をfalseへ変更。無効時の初期化・更新処理スキップと、コンテナの設定リンクを確認。実行時間比較・既存launchの再起動は未実施。[変更範囲と検証](releases/2026-09-23_disable_curve_default.md)。

## 2026-09-16: L0の元姿勢割合による色表示

- 安全判定用labelを維持し、集約元の状態件数を保存・ROS配信・WS配信・Viewer色／詳細表示へ追加。[仕様と検証コマンド](releases/2026-09-16_l0_state_ratio_colors.md)。
- 依存7パッケージのビルド、C++14件、Frontend3件、lint、本番ビルド成功。初回C++ヘッダー依存不足は修正、既存Viteキャッシュ所有権問題はrunnerと一時出力先で回避。
- L0を150ノード・740エッジで再生成し、静的モデルの件数は安全10,801・危険0・衝突0。旧モデルを退避し、元gng/vlutのSHA-256不変を確認。
- 隔離ドメイン225で静的ROS→WSの件数一致、合成ノードのlabel・座標不変での件数更新を確認。生成・試験プロセスと子ノードは全終了。既存ユーザーノードの停止／再起動なし、実ブラウザ・実GPU描画は未検証。

## 2026-09-16: 簡略版Tmap L0の再生成と単体召喚確認

- 稼働中環境で`/ToPoDualArm/Tmap_vis_L0`がなく、`visualization_gng.enabled: true`に対して保存済み`vis_gng_L0.bin`が非対応の`VIZGNG2`であることを確認。旧モデルは同じディレクトリの`vis_gng_L0_v2_20260916.bin`へバックアップ。
- 現行trainerでL0を再生成し、10,801元ノードから150ノード・740エッジ、連結成分1・孤立0を確認。`vis_gng_L0.bin`は`VIZGNG5`、単体版`vis_gng_static_L0.bin`は`VIZGST1`。保存再読込検証成功。元`gng.bin`・`vlut.bin`への変更なし。
- 通常環境の元Tmapのframeを`ToPoDualArm/base_link`と受信確認。隔離ドメイン224で単体launchを起動し、同frameの150ノード・740エッジを受信。最初のCLI echoは時間切れとなり、明示的reliable/transient-localのPython購読で再確認。
- 生成・検証プロセスは終了、単体launchと検証購読ノードは停止済み。既存のブリッジ・再生の停止／再起動なし。既存ブラウザでの表示は未検証。[起動手順](../README.md#topofuzzy-viewerへのブリッジ)と[単体版の座標指定](TECHNICAL_SPEC.md#135-元gng非依存のstatic召喚)を更新。

コンテナ内で実行した生成・検証起動コマンド（ドメイン224、全停止済み）:

```bash
ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1 ros2 run gng_vlut_system visualization_gng_trainer \
  --input /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin \
  --output-prefix /ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng \
  --layer 0 --target-nodes 150 --iterations 200000 --seed 42 \
  --joint-motion-weight 0 --workspace-motion-sec-per-m 1.0 --workspace-sample-resolution 0.05 \
  --ros-args --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml

ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1 ros2 launch gng_vlut_system visualization_gng_static.launch.py \
  model_path:=/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/vis_gng_static_L0.bin \
  topic_name:=/ToPoDualArm/Tmap_vis_static_L0 \
  frame_id:=ToPoDualArm/base_link
```

## 2026-09-16: Tmap_staticのBBoxによるROI範囲

- ToPoDualArmのROIを静的マップBBox＋各面20 cmへ変更。TF変換、直接／world検索、consumer別設定に対応。[仕様・検証コマンド](releases/2026-09-16_tmap_roi_bounds.md)。
- 対象ノードのビルド・既存単体テスト21件、隔離ROSの3方式とlaunch引数伝達を検証。初回のテスト用JSON引用不足による起動失敗は引用修正後に再検証。
- 試験用ノードは全停止。既存ノードの停止・再起動なし。実環境の表示確認は未実施。

## 2026-09-16: GNG境界ノード近傍の重点学習

- ガウス距離重み、把持重点との混合、時刻・frame失効、CPU重み付きAPIを追加し、`graspnet.yaml`で有効化。[仕様・検証](releases/2026-09-16_boundary_attention.md)。
- 初回リンク失敗後の再ビルドで2パッケージ成功。境界6件・既存把持重点・重点APIテスト成功。合成10万点の境界検索は1回測定25.7 ms。実点群での改善効果は未検証。
- 既存ノードの停止・新規ROSノードの起動なし。ビルド・単体テストは終了済み。

## 2026-09-15: Viewerの点群トピック別表示設定

- Displayタブへ共通／トピック別の切替と共通設定への復帰を追加。点サイズ・不透明度・色・Heatmap範囲を個別管理し、配信停止と設定を分離。[仕様・検証コマンド](releases/2026-09-15_point_cloud_topic_display.md)を記録。
- 新規の状態更新・2点群Renderer検証と既存のストリーム復帰検証、lint、本番ビルドに成功。個別設定変更でgeometry・頂点attributeのversion不変を確認。テスト用JSXと初回Vite権限エラーを回避後に成功。
- 有限の検証コマンドは終了、一時出力は削除済み。ROS・サーバーの起動停止なし。ブラウザの実操作・実GPU描画は未検証。

## 2026-09-15: ROIボクセルとGNGの座標ずれ調査

再調査: ちらつきの申告後も外部TFの子は`ToPoDualArm/base_link`。`/tf`のPublisherは`test_tf_publisher`、`/tf_static`はrosbag再生とロボットの`robot_state_publisher`。6秒の受信で`world -> ToPoDualArm/base_link`の動的TFを121件、`ToPoDualArm/base_footprint -> ToPoDualArm/base_link`の固定TFを1件確認。Viewerの5秒受信で固定姿勢1件・動的姿勢38件、切替1回。親フレームの競合とViewer側での値の上書きを確認したが、周期的なちらつきの全原因までは未確定。既存ノード・ソース・YAMLの変更なし。診断用コールバックは初回2試行でrclpyのMessageInfo非対応により失敗し、メッセージ単体での受信へ変更後に成功。すべての診断プロセス・一時WebSocket接続は終了済み。

再調査時の診断ノード起動コマンド（終了済み）:

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 3s 15s python3 -c '\''import time,json,rclpy
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
rclpy.init(); node=rclpy.create_node("tf_conflict_probe"); records={}; subscriptions=[]
def callback(topic):
 def receive(msg):
  for t in msg.transforms:
   if t.child_frame_id not in ("ToPoDualArm/base_link","ToPoDualArm/base_footprint"):continue
   p,q=t.transform.translation,t.transform.rotation
   key=(topic,t.header.frame_id,t.child_frame_id,p.x,p.y,p.z,q.x,q.y,q.z,q.w)
   records[key]=records.get(key,0)+1
 return receive
try:
 for topic,durability in [("/tf",DurabilityPolicy.VOLATILE),("/tf_static",DurabilityPolicy.TRANSIENT_LOCAL)]:
  subscriptions.append(node.create_subscription(TFMessage,topic,callback(topic),QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=durability)))
 end=time.monotonic()+6
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.1)
 for key,count in records.items():print(json.dumps({"topic":key[0],"parent":key[1],"child":key[2],"xyz":key[3:6],"quaternion":key[6:],"received":count}),flush=True)
finally:node.destroy_node();rclpy.shutdown()
'\'''
```

追記: ロボット座標で格子を構築しworld表示で回転させる意図を確認後、入力frameの末尾一致による自動読み替えを削除。ToPoDualArmの固定TFをOFFに変更し、URDFルートへの外部TF運用をREADMEへ記載。[修正・検証コマンド](releases/2026-09-15_roi_robot_frame.md)を記録。修正前に不具合を再現、修正後は単体テスト21件と隔離ROSの4条件×2姿勢でvoxel ID一致・world復元位置の量子化誤差内一致を確認。ビルド指定とテストのsnapshot要求形式の失敗も修正後に再検証。専用ノードは停止済み、既存ノードの停止・再起動なし。実ブラウザ目視は未検証。

- 実入力`/camera/camera/depth/color/points`と`/topological_map`の`frame_id`が`base_link`であることを受信確認。ROI生成の`resolveSourceFrameId`だけが末尾一致で`ToPoDualArm/base_link`へ読み替える実装を確認。Viewerの既存WebSocketから受信した`/ToPoDualArm/self_filter_roi_voxels`は`ToPoDualArm/base_link`、voxel幅0.02 m。Viewerは両者へ別のTFを適用する構成。
- `world -> ToPoDualArm/base_link`の動的TF（x=0.15、yaw=1.5）と、`world -> ToPoDualArm/base_footprint -> ToPoDualArm/base_link`の固定TFを同時受信。ロボットbaseの親フレーム定義の競合を確認。診断終盤には外部操作によるTF配信プロセスの引数変更を観測。
- 座標系の解釈は環境固定／ロボット追従の意図確認が必要。コード・YAML・既存ノードへの変更なし。ブラウザの手動オフセットや最終描画位置の実測は未実施。
- 初回診断では既存`voxel_msgs`のPython型サポートにundefined symbolエラー。Voxelを除く受信と既存Viewerの読み取り専用接続に切り替えて調査。診断プロセスとWebSocket接続は終了、追加ROSデーモン・診断ノードの残留なし。

ヘッダ・TF診断の起動コマンド（終了済み）:

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 15s python3 -c '\''import time,json,rclpy
from sensor_msgs.msg import PointCloud2
from ais_gng_msgs.msg import TopologicalMap
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
rclpy.init(); node=rclpy.create_node("roi_frame_diagnostic"); seen=set(); subs=[]
def receive(msg,topic):
 key=(topic,msg.header.frame_id)
 if key in seen:return
 seen.add(key); print("HEADER",topic,msg.header,flush=True)
def tf(msg):
 for t in msg.transforms:
  key=(t.header.frame_id,t.child_frame_id,str(t.transform))
  if key not in seen:seen.add(key);print("TF",key,flush=True)
try:
 for kind,topic in [(PointCloud2,"/camera/camera/depth/color/points"),(TopologicalMap,"/topological_map")]:subs.append(node.create_subscription(kind,topic,lambda m,t=topic:receive(m,t),qos_profile_sensor_data))
 subs.append(node.create_subscription(TFMessage,"/tf",tf,qos_profile_sensor_data))
 subs.append(node.create_subscription(TFMessage,"/tf_static",tf,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
 end=time.monotonic()+5
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.1)
finally:node.destroy_node();rclpy.shutdown()
'\'''
```

## 2026-09-15: 把持アテンション選択点のトピック化

- `/downsampling/grasp`へXYZのPointCloud2出力を追加。重点学習ON・購読時のみ点群化、対象失効時は空点群。既存の選択添字・変換処理を再利用し、GNGコア・設定項目の追加変更なし。[仕様・検証起動コマンド](releases/2026-09-15_grasp_attention_cloud.md)を記録。
- Dockerビルド、単体テスト3件、隔離ROSでの出力座標・点数・時刻・TF・空点群・OFF時Publisherなし・購読なし重点学習の検証に成功。Viewer目視・配信負荷測定は未実施。
- 専用CPU・Pythonノードは停止済み、追加デーモン残留なし。既存CPU/Viewerを維持。既存計画系の外部終了を観測したが本作業からの停止操作なし。

## 2026-09-15: 把持アテンションの物体候補AABB化

- ユーザー指定によりノード半径方式をクラスタ別AABB＋余白へ置換。`radius`を`margin`へ変更し、既定OFF・配分率・失効条件・GNGコアを維持。[仕様・実行コマンド](releases/2026-09-15_grasp_attention_aabb.md)を記録。
- Dockerビルド、AABB単体テスト3件、既存GNG APIテスト2対象、隔離ROSの候補内部点・別候補間・所属なし・失効・TF検証に成功。10候補・5000ノード・10万入力点のAABB構築＋抽出は合成データ単回で1.42386 ms。
- 検証用CPU・Pythonノードは終了、既存CPU/ViewerのPIDを維持。実環境での把持成功率・最適余白は未検証。

## 2026-09-15: 把持候補近傍のGNG重点学習

- CPU GNGへ既定OFFの重点入力APIとROS候補購読を追加。総学習回数内での配分、候補失効・TF失敗時の通常処理、重点分の観測統計への非計上を実装。[仕様・起動方法](../../ais_gng_cpu/docs/grasp_attention.md)と[検証コマンド・結果](releases/2026-09-15_grasp_attention.md)を記録。
- Docker Releaseビルド、CTest 3対象、隔離ROSでのON/OFF・TF・時刻・空候補・受信停止の検証に成功。初回ビルドのTF参照範囲とテストの設定順序を修正後に成功。
- 5000候補中心・10万点の近傍検索を合成データで測定。空間ハッシュ43.8318 msからkd-tree20.3293 msへ変更、選択点42178点で一致。重点50%時の通常勝者イベント500回、共分散・観測方向件数への重点分の混入なしを確認。
- 検証用ROS domain 219のCPU・Pythonノードは終了、既存CPU/ViewerのPIDを維持。把持推定器の外部再起動を観測したが本作業からの停止なし。実物把持の成功率改善・実入力での最適設定は未検証。

## 2026-09-15: 非平面成分のViewer Graph化

追記: ユーザー指定により`/nonplane_components`のBbox既定設定を削除し、GUI・独立ビュー選択を対象外へ変更。Graph配信・法線・共分散と把持候補Tmapの既定ONは維持。関連テスト2ファイル・lint・frontend本番ビルド・Docker backendビルドに成功。実ブラウザ操作は未検証。有限コマンドは終了、一時出力は削除済み。ROSノード・サーバーの起動停止なし。

今回の検証コマンド（frontendディレクトリ）:

```bash
node --test tests/inspection_bbox_gate.test.mjs tests/candidate_hover_frame.test.mjs
npm run lint
npm run build -- --configLoader runner --outDir /tmp/nonplane-bbox-hidden-build
```

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 180s colcon build --packages-select topo_fuzzy_viewer --symlink-install --parallel-workers 1'
```

- ROS所属配列を維持し、バックエンドの非平面Marker生成を既存Graphバイナリ配信へ置換。元ID・法線・共分散・成分所属・実エッジを保持し、平面側端点を成分のBboxから除外。[現行仕様・実行コマンド](releases/2026-09-15_nonplane_graph_viewer.md)を記録。
- `/nonplane_components`に既定OFFのBbox切替を追加。他トピックの未指定設定・把持候補の既定ONと既存の並行変更を維持。
- Docker backendビルド・CTest 2対象、隔離ROSでの入力全6到着順・再接続再購読・空成分・購読解除検証、frontend関連5ファイル・lint・本番ビルドに成功。初回のテスト側端点数・再購読漏れ・旧Bbox期待値と一時出力先権限を修正後に再検証成功。
- 実ブラウザの目視と実GNG入力の描画は未検証。専用ROS domain 218・port 19092のノードは終了、一時build出力は削除。既存Viewerの停止・再起動なし。CPU GNGの外部操作によるPID更新を観測したが、本作業からの停止操作なし。

## 2026-09-14: 作業記録の分類

- `progress.md` と `reject.md` を新設し、実施済み作業と不採用判断を分離。
- ローカルスキルと `AGENTS.md`、文書索引を更新。未確定指標への仮の計算式追加を避ける規則を明文化。
- 既存のタスク本文・過去のリリースノートの移動や削除なし。
- `quick_validate.py skills/maintain-project-docs` によるスキル形式検証に成功。

## 2026-09-14: 配信用の暫定評価指標

- リポジトリ内の参照調査で、関節余裕min/meanと推定エネルギー・時間が候補選定に未使用で、配信・表示用であることを確認。
- 4指標の暫定処理と専用パラメータを削除。既存NaN初期値と汎用評価の無効フラグを維持。
- 上方候補の面積比は順位付けに使用中であるため維持。採否条件・軌道選択ロジックの変更なし。
- 変更仕様は [リリースノート](releases/2026-09-14_provisional_candidate_metrics.md) を参照。
- Dockerの`gng_cpu_container`内で対象ノードと追加テストをビルド。テスト用includeパス・静的リンク順の不足を修正後、成功。
- 回帰テスト1件に成功。4指標のNaN・無効フラグ、候補ID・選択状態・姿勢・関節値・経路・位置可操作性の保持を確認。
- 実環境の把持・Viewer画面での検証は未実施。常駐ROSノードの新規起動・既存プロセスの停止なし。ビルド・テストプロセスは終了。

実行コマンド（Docker内、`source /ros2_ws/install/setup.bash` 後）:

```bash
cmake --build /ros2_ws/build/gng_vlut_system --target topological_map_avoidance_node -j2
cmake --build /ros2_ws/build/gng_vlut_system --target test_candidate_metric_availability -j2
ctest --test-dir /ros2_ws/build/gng_vlut_system -R '^test_candidate_metric_availability$' --output-on-failure
```

## 2026-09-14: 保留作業の記録先追加

- `pending.md`を新設し、保留理由・完了済み範囲・再開条件・次の一手の記録項目を整備。
- 作業記録スキル、`AGENTS.md`、文書索引と各台帳の案内を更新。保留と不採用の区別、再開時の移管・二重管理防止を明文化。
- 既存タスク本文と既存の不採用判断は維持。保留項目の自動登録・移動なし。ROSコード・設定の変更なし。

## 2026-09-14: ファジィルール実装の所在確認と設計書

- HTMLの既定ルール・所属関数・IF-THEN評価・JSON編集処理と、別用途の`FuzzyClassifier`を確認。
- ROS把持経路では指標配信と固定的な候補選択を確認。汎用IF-THENエンジンの把持候補選択への接続は見つからず。
- [ROSルールエンジンの実装雛形設計](designs/fuzzy_rule_engine_design.md)を追加。責務・入出力・設定形式・欠損値・ID対応・確認項目を記載。
- 実装・数値境界の確定・ROS起動・動作検証は未実施。既存コードとタスク順序の変更なし。

## 2026-09-14: 隣接平面クラスタの統合拒否調査

- 稼働中のCPU GNGから`/topological_map`と`/plane_clusters`を読み取り、同じ`frame_number`の5更新（9236〜9240）について隣接クラスタ対の統合条件を再計算。
- クラスタ7と10は法線差約5.4〜6.5度・接続4本で、面内広がり比・統合後残差上限・少数側残差の条件を通過。正規化残差約0.30に対して増加判定の許容値が約0.20となり、残差増加条件による拒否を確認。
- 根拠は[統合判定](../../ais_gng_cpu/src/ais_gng/src/topological_plane/plane_cluster_incremental.cpp#L1248)。出力済みクラスタ対の再評価であり、内部の逐次統合全経路や物理的な同一平面性の検証ではない点に留意。
- [CPU起動処理](../../ais_gng_cpu/src/ais_gng/launch/ais_gng.launch.py#L167)で、`plane_params_file`から非平面成分設定だけを抽出し、平面クラスタの統合設定をCPUノードへ渡していないことを確認。
- 調査用ノード`plane_merge_readonly_probe`は終了し、終了後のプロセス一覧で残存なしを確認。既存のGNG・Viewer・ROS daemonの停止や再起動なし。ROSソースコード・設定変更、ビルド、統合条件変更後の検証は未実施。

調査ノードの実行コマンド（終了済み、調査スクリプトは一時ファイル）:

```bash
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 15s python3 -' < /tmp/plane_merge_probe.py
```

## 2026-09-14: 把持ファジールール設計資料の統合

- 入力設計・実装雛形・145件のルール候補・ROS入力指標候補の4資料を [統合設計書](designs/fuzzy_grasp_design.md) へ移管。旧4ファイルはリンク案内のみ。
- 現行実装調査から未実装の改善提案を統合先へ移管。実装説明・数値再現結果・過去のスライド・既存進捗は維持。
- `/grasp_pose_cands`の型、`/plane_clusters`の名称、独立スコアtopicの廃止、関節余裕・推定時間等の未計算状態をソース確認の範囲で整合。全topicの実受信確認なし。
- README・現状資料・評価メッセージ仕様・スライド生成元の参照先を更新。スライドのPowerPoint・PDF再生成は未実施。
- 移管前後の照合でルール145件の本文、入力表67行、指標・仮説等のID100件、JSON雛形1件の保持を確認。
- 文書リンク88件の解決、コードフェンス対応、スライド生成元のPython構文を検査。既存索引の無関係なリンク切れ1件は変更対象外。`git diff --check`に成功。
- ROSコード・設定・評価式・タスク順序の変更なし。常駐プロセスの新規起動・既存プロセスの停止なし。

## 2026-09-14: 疎な接続による平面分割の改善

- 実入力のクラスタ1と54で、幾何条件を通過する一方、直接接続1本のため統合候補から外れるケースを確認。1と19は少数側残差でも拒否となる別ケース。
- 接続1本のノイズ付き同一平面で回帰テスト失敗を確認後、統合用接続数とYAML転送を修正。変更範囲・互換性は[リリースノート](releases/2026-09-14_plane_merge_connections.md)を参照。
- DockerビルドとC++22件・launch2件のテストに成功。同一実入力列の隔離再生で分割数の減少を確認。全箇所の正解判定や修正後Viewerでの目視確認は未実施。
- 調査・比較ノードはすべて終了。プロセス一覧で既存GNG・Viewer・ROS daemonの維持とテストプロセス残存なしを確認。比較用一時スクリプトは終了後削除。

起動コマンド（すべて終了済み）:
```bash
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 15s python3 -' < /tmp/plane_merge_probe.py
docker compose exec -T gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 120s python3 -' < /tmp/plane_merge_replay.py
```

## 2026-09-14: 不採用案の追加テスト削除

- ユーザー依頼により、[不採用の平面クラスタ改善案](reject.md)に追加したC++テスト1件、launchテスト2件、専用補助コードとCMake登録を削除。既存テスト・処理本体の変更なし。
- 削除対象のソース参照残存なしと`git diff --check`を確認。再ビルド・テスト実行・ROSプロセスの操作は未実施。

## 2026-09-14: 上方把持フローの順序修正

- [システム全体フロー](presentations/README.md#システム全体フロー上方把持の順序修正版)をDOT・SVG・PNGで作成。生成コマンドは同READMEに記載、生成処理は終了済み。
- 把持面の寸法確認と、候補姿勢生成後の付属ノード・対象寸法確認を分離。幾何的候補と関節姿勢探索の出力も分離。
- SVG上の主要接続16本と、未接続ファジー評価への入力矢印がないことを機械確認。PNGで文字と接続を目視確認、`git diff --check`に成功。
- ROSコード・設定変更、常駐プロセスの新規起動、既存プロセスの停止なし。

## 2026-09-14: 全体フローのスライド配置調整

- 全体フローのSVGを1920×1080の固定配置へ変更。上段の環境認識・把持候補生成、下段の照合・関節姿勢評価へ整理し、主要文字を26pxへ拡大。
- 元図の22ノード・22接続の保持とSVG・PNG寸法を機械確認。PNGで文字・矢印・背景区分の重なりを目視確認し、修正。
- SVGを配置の正本、DOTを接続の参照用として案内を更新。[PNG再生成コマンド](presentations/README.md#システム全体フロー上方把持の順序修正版)を実行し、生成処理は終了済み。`git diff --check`に成功。
- ROSコード・設定変更、常駐プロセスの新規起動、既存プロセスの停止なし。

## 2026-09-14: 候補ノードの独立3Dビュー

- Topo Fuzzy Viewerへ候補ノードのクリック選択と独立カメラの詳細表示を追加。切り出しは既存`viewer_edit_node`、描画は既存の一括描画を利用。単体HTML・把持推定処理への変更なし。[仕様・操作方法](releases/2026-09-14_candidate_inspection_view.md)を参照。
- DockerのReleaseビルド、追加C++テスト5件、frontend lint・ホスト側buildに成功。frontendコンテナ内buildは既存MCAP依存不足で失敗し、依存が揃っているホストで検証。依存追加なし。
- ROS domain 225とWS port 19001のダミー候補を専用Chromeで受信。ノードクリックから平面25点・非平面5点の表示、主画面と独立した回転・ズーム・平行移動、全体表示、固定保持、明示更新、ウィンドウ移動、ドラッグ誤選択の防止を操作・画像比較で確認。
- サイドバーを含む画面座標と3D領域座標の混同による初期配置のはみ出しを検出・修正。実ユーザー画面・大規模実環境での性能測定は未実施。
- 検証用ROS・HTTP・Chromeの全プロセス終了と専用3ポートのリスナー消滅を確認。既存ROS・frontendのPID、ROS daemon、コンテナ状態を維持。

検証起動コマンド（すべて終了済み、一時スクリプト使用）:
```bash
docker compose exec -T -w /ros2_ws gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout --signal=INT --kill-after=15s 620s python3 -u -' < /tmp/codex-candidate-inspection-ros.py
node /tmp/codex-inspection-browser.cjs
node /tmp/codex-inspection-ui-test.cjs
```

## 2026-09-14: 候補のホバー枠

- 候補ノード・クラスタへのホバー枠を追加。独立ビューと共通の所属解決・AABBを使用し、TF・手動表示変換を適用。[仕様](releases/2026-09-14_candidate_hover_frame.md)を参照。
- Docker Releaseビルド、候補抽出C++テスト6件、frontend lint・ホスト側buildに成功。検証によるTypeScript生成キャッシュの差分は除去。
- domain 225・WS port 19001と専用Chromeで、TF付き枠の描画位置・候補切替・枠消去・クリックとの両立・ドラッグ中の非表示・購読OFFでの取得停止を検証。範囲取得は2.1秒で6件。大規模実環境での負荷計測は未実施。
- 検証用ROS・HTTP・Chromeの終了、専用3ポートの消滅、既存ROSのPID維持を確認。開始前に停止状態だったfrontend・Viewerの起動なし。コンテナ状態を維持。

検証起動コマンド（すべて終了済み、一時スクリプト使用）:
```bash
docker compose exec -T -w /ros2_ws gng_cpu bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout --signal=INT --kill-after=15s 620s python3 -u -' < /tmp/codex-hover-ros.py
node /tmp/codex-hover-browser.cjs
node /tmp/codex-hover-test.cjs
```

## 2026-09-14: 把持候補経路生成と回避実行の分離

- `grasp_joint_candidates.launch.py`の起動先を経路生成専用ノードへ変更。共通処理の移動による実装共有と、追従・退避・近傍追加ペナルティを通らない計画更新を実装。[仕様・検証コマンド](releases/2026-09-14_candidate_path_planner.md)を記録し、RUN_GUIDEと現行仕様・ソース参照を更新。
- Docker ReleaseビルドとC++・既存到達性テストに成功。追加C++テストの補助グラフのインターフェース不足による初回ビルド失敗は、補完後の再ビルド・再実行で解消。
- domain 218で指定launchと実GNG入力を実行。専用ノードの起動、回避ノードとtrialサービスの不在、候補経路・評価・ロボットプレビューの配信、静止時の再探索抑制、明示要求・関節角度変更での更新、空候補時の旧出力消去、関節指令・control claim配信なしを確認。
- 最終ビルドで結合テストを再実行し成功。検証用プロセスはすべて停止済み。既存ROSのPIDとコンテナ状態を維持し、ROS daemonの新規残留なし。実環境でのCPU負荷測定は未実施。

## 2026-09-14: 付属抽出の不要な切替設定の削除

- 参照平面方式へ一本化し、付属抽出の2つの切替設定と旧成分所有者方式を削除。現行ToPoDualArm設定の動作を維持。[仕様・実行コマンド](releases/2026-09-14_fixed_reference_attachment.md)を記録し、資料・スライドを更新。
- Dockerビルド、単体テスト、domain 117のROS結合テスト3ケースに成功。テスト側の候補ID固定前提と未確定出力取りこぼしを修正し、全ケースを再確認。
- 検証用プロセスはすべて終了済み。既存ROSのPIDを維持し、稼働ノードの再起動なし。実機の把持成功率は未検証。

## 2026-09-14: 候補選択のバウンディングボックス化

- ユーザー指定により、初回ホバーとクリックの当たり判定をノードの球から物体全体のAABBへ変更。候補範囲の一括取得と更新中の保持を実装。[仕様・検証コマンド](releases/2026-09-14_candidate_bbox_picking.md)を記録。
- Docker Releaseビルド、C++テスト7件、frontend hover・Marker回帰テスト、lint・buildに成功。テスト一時出力先の権限不足は書込可能なテスト配下への変更で解消。
- domain 226のROS内部RPCで部品合算AABBと詳細取得の一致を確認。検証ノードは停止済みで既存ROSのPID・コンテナ状態を維持。実画面のGPU操作と大規模実入力の負荷計測は未実施。

## 2026-09-14: グラフトピック名のtmap短縮

- 環境入力`/topological_map`を維持し、関連トピックの既定値・launch・設定例を短縮。Viewerの経路判定を新旧名へ対応。[変更一覧・検証コマンド](releases/2026-09-14_tmap_topics.md)を記録。
- 関連ROSノードのDockerビルド、frontend型チェック・名前判定テストに成功。domain 217のViewer検証で新旧5トピックの検出・購読・バイナリ受信を確認。
- 経路結合テストは並行作業中のC++目標選択ノードが未配置のため停止。変換ノードのビルド対象も既存CMakeに存在せず、成功扱いなし。調整待ちの範囲を[pending.md](pending.md)へ記録。
- 検証用プロセスはすべて終了済み。既存ROSノードのPID維持と未再起動を確認。実画面のGPU操作は未検証。

## 2026-09-14: 計画目標選択のCPU張り付き解消

- 高負荷の対象をPython目標選択ノードへ特定。実入力のprofileでROSメッセージの取出し・Pythonオブジェクト展開が約82%を占有することを確認し、C++へ移管。[変更内容・計測条件・起動コマンド](releases/2026-09-14_goal_selector_cpu.md)を記録。
- Docker Releaseビルド、選択C++テスト9件と既存候補評価テストに成功。初回のメッセージ定数所属・整数型不一致は修正後の再ビルドで解消。
- 同一実入力10,801ノード・8候補で選択結果の完全一致を確認。TF移動・復帰、空候補の失効・復帰を検証。実入力6秒のCPU測定は103.27%から12.17%へ低下し、最終版の再測定は11.33%。
- tmap短縮を保持した最終launchの結合テストをdomain 228で実行し成功。C++既定topicの整合・実行ファイル配置・選定map配信まで確認し、対応する結合確認の保留を解消。変換ノードの別のビルド問題は対象外。
- 計測・再生・launchはすべて停止済み。既存ROSのPID・コンテナ状態を維持。稼働中旧版の再起動と実画面確認は未実施。

## 2026-09-14: Tmapへの表記統一

- ユーザー指定の`Tmap`へROS既定名・設定・Viewer判定を統一。環境入力`/topological_map`は維持。[変更・検証コマンド](releases/2026-09-14_tmap_topics.md)を更新。
- 関連3パッケージのDocker再ビルド、frontend型チェック・名前判定、domain 218の経路結合テストに成功。検証プロセスは終了済み。既存ROSへの停止・再起動操作なし。

## 2026-09-14: 静的ロボットマップのホバー枠除外

- `/ToPoDualArm/Tmap_static`をホバー枠・範囲取得から除外。実装は除外条件1行のみ。[仕様・検証](releases/2026-09-14_candidate_bbox_picking.md)を更新。
- ホバー回帰テストとfrontend型チェックに成功。検証コマンドは終了済み、ROSの起動・再起動なし。実画面確認は未実施。

## 2026-09-14: 把持対象ノードのTopologicalMap配信

- ユーザー指定の`/grasp_pose_cands/Tmap`へ出力を変更。同じ候補内の元GNGエッジだけを収録し、候補IDはclusterへ対応。到達性を既存semantic_labelで配信し、Viewerの共通ラベル設定へ接続。[仕様・検証コマンド](releases/2026-09-14_candidate_topological_map.md)を記録。
- Docker Releaseビルド3パッケージ、把持推定・部分グラフC++テスト、Viewer詳細抽出7件、domain 117のROS結合3ケースに成功。候補間エッジ除外、状態更新、遅延購読、空配信を確認。
- frontendのラベル・ホバー回帰テスト、lint・buildに成功。既存ビルド警告あり、実画面GPU描画と大規模入力の負荷測定は未実施。
- 検証launch・子ノードは全停止済み。既存ROSのPIDとコンテナ状態を維持し、ROSデーモンの新規残留なし。稼働中旧版の自動再起動なし。

## 2026-09-14: バウンディングボックス判定の把持候補限定

- ユーザー指定により、ホバー枠・枠内クリック・範囲取得を`/grasp_pose_cands/`配下だけへ限定。非平面成分などの個別除外ではなく、許可するprefixの判定へ変更。[現行仕様・検証](releases/2026-09-14_candidate_bbox_picking.md)を更新。
- 対象外だけの表示時にも範囲取得・枠・クリック選択が発生しない回帰テスト、frontend lint・buildに成功。実画面操作は未検証。
- 検証コマンドは全終了済み。ROSノード・サーバーの新規起動なし。backendは未変更で、既存の全体ビルドへの干渉なし。

## 2026-09-14: 把持候補矢印の上位N件表示

- 既存Marker設定へ表示数スライダーを追加。受信順の先頭N件だけを描画し、0は全件。ROS・対象ノードグラフ・計画への変更なし。[仕様・検証](releases/2026-09-14_candidate_display_limit.md)を記録。
- 描画回帰テスト、lint、型チェックに成功。コマンドは全終了済み。ROS・サーバー起動なし、実画面操作は未検証。

## 2026-09-14: バウンディングボックスのトピック別フラグ化

- ユーザー指定により、名前による対象制限を削除。Graph/Markerのトピック別`enable_bounding_box`とGUIの`Bounding Box`切替を追加し、全トピックの既定をOFFへ統一。[現行仕様・検証](releases/2026-09-14_candidate_bbox_picking.md)を更新。
- 把持候補Graph・非平面成分Graph・任意名Markerの回帰検証に成功。既定OFF、明示ON、OFF時の枠・クリック解除、遅延応答破棄、再ON、全OFF時のタイマー停止を確認。
- frontend lint・buildとDocker backendビルドに成功。検証コマンドは全終了済み。ROSノードの新規起動・停止操作なし。実画面操作は未検証。

## 2026-09-14: 把持対象グラフの表示既定値

- `/grasp_pose_cands/Tmap`のノードサイズ0.008、エッジ表示OFF、ノード不透明度0.5へ変更。他レイヤーと手動設定は維持。[仕様・検証](releases/2026-09-14_candidate_topological_map.md)を更新。
- 既定値回帰テスト・型チェックに成功。検証コマンドは終了済み、ROS起動なし。実画面操作は未検証。

## 2026-09-14: 把持候補ロボットプレビュー未配信の調査

- 21:32:23のlaunchログで`topological_map_path_planner_node`（PID 51618）の起動直後の終了コード-6を確認。目標選択ノードだけが継続し、計画・プレビュー生成ノードは不在。
- 6秒の購読で静的GNG 10,801ノード、把持候補6件（INSIDE 2件）、選定ID `[3425,3485,4105]`を確認。候補評価トピックのpublisherは0。robot pose購読はQoS不一致があり、未受信を停止の根拠には使用せず。
- domain 218で同一設定を再現すると正常初期化し、10秒の上限まで入力待ちを継続。元の異常終了の例外本文は保存ログになく、詳細原因は未確定。コード変更・既存launchの停止や再起動なし。
- 以下の調査用プロセスはすべて終了し、既存ROSのPID維持を確認。

```bash
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 10s /ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/topological_map_path_planner_node --ros-args -r __node:=topological_map_path_planner_node -r __ns:=/ToPoDualArm --params-file /ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml --params-file /tmp/launch_params_pihazn67'
docker exec -i gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 15s python3 -' < /tmp/preview_inputs_probe.py
```

## 2026-09-14: Marker側のバウンディングボックス設定の削除

- ユーザー指定により、法線などのMarker側からボタン・設定・判定経路を削除。Graph側の既定OFF・明示ONと、Marker本体の描画は維持。[現行仕様・検証](releases/2026-09-14_candidate_bbox_picking.md)を更新。
- ホバー・既存Marker描画の回帰テスト、frontend lint・buildに成功。検証コマンドは全終了済み。ROS・サーバー起動なし、実画面操作は未検証。

## 2026-09-14: 把持候補ロボットの基準フレーム修正

- 実配信で通常ロボットの `base_link` と候補の `base_footprint` の不一致を確認。計画ノードもYAMLの `frame_id` を使用するよう修正し、重複した初期化を削除。[仕様・検証コマンド](releases/2026-09-14_candidate_robot_frame.md)を記録。
- 追加回帰検証で修正前の失敗を再現。Docker Releaseビルド、domain 218の既存経路結合テストと全候補フレーム一致、フレーム設定5ケースに成功。実画面描画は未確認。
- 購読プローブ・検証launch・子ノードは全停止済み。テスト用一時ログを削除。既存ROSへの停止・再起動操作なし。
- 最終確認時に、こちらの操作によらない既存Viewerコンテナ・関連launchの停止を確認。検証プロセスとROSデーモンの新規残留なし。外部変更を戻すための再起動は未実施。

## 2026-09-14: 候補ロボットと本体のTF不整合調査

- Docker内の6秒の読み取り専用購読で、候補ロボットのframeIdは`ToPoDualArm/base_footprint`、本体・静的GNG・選定GNGは`ToPoDualArm/base_link`と確認。候補配信も確認済み。
- `world -> base_footprint`は原点・無回転、別配信元の`world -> base_link`は位置`(0.15,0,-0.2)`・yaw `3.2 rad`。URDF由来の`base_footprint -> base_link`も同時配信され、`base_link`の親が競合。候補と本体で異なる変換の適用を確認。
- `world -> graspnet_table`は単位変換、把持候補と対象ノードグラフは`world`。今回の購読範囲では物体入力側の非単位変換なし。
- 調査用コマンドは下記のとおり終了コード0で終了し、プローブ残留なし。既存プロセスの停止・再起動操作、ソース・設定変更なし。調査中に外部からの既存launch再起動を観測したため、全PID維持とは記録せず。実画面での修正後確認は未実施。

```bash
docker exec -i gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 5s 15s python3 -' < /tmp/grasp_frame_probe.py
```

## 2026-09-14: 稼働中処理のCPU負荷測定

- 22:00:50から10秒間の`pidstat`測定。100%は論理CPU 1個相当、ホストは16論理CPU。候補経路計画98.5%、Viewer専用Chrome GPUプロセス92.3%、GraspNet再生70.4%、同Chrome renderer32.0%、ais_gng_cpu26.5%、topofuzzy_bridge24.7%、world_index_to_voxel17.3%、目標選択13.5%、voxel_to_vlut10.0%、上方把持推定9.1%、平面クラスタ3.7%、Viewer gateway3.6%。Chromeの値はGPU使用率ではなくCPU使用率。
- 別時点の`docker stats --no-stream`ではgng_cpu_container244.12%、graspnet_player72.95%、rosbridge_container0.32%、frontend0.10%。frontendコンテナ値にブラウザ描画負荷は含まず。
- コード確認では、計画側に入力不変時の早期returnが存在する一方、`planFromStartCandidates`内でゴール候補ごと・開始候補ごとにDijkstra探索を実行。`topofuzzy_bridge`の占有・危険ボクセル受信は入力不変判定なしで安全評価とdirty化。計画が実際に何を契機に再実行したかの内訳は未測定。
- GraspNet再生は20 Hz・フレーム10〜15のループ設定。コンテナ内`graspnet_player_cpp.cpp`の`publish_frame`は各回RGB/depth画像の読み込み、点群生成、PointCloud2生成を実行し、生成済み点群のキャッシュなし。Viewerは既に`frameloop="demand"`、`dpr={1}`。
- 関数別サンプリングは`perf_event_paranoid=4`によって失敗。権限制限の変更なし、関数別CPU割合・改善率は未確定。ソース・設定の変更、既存プロセスの停止・再起動なし。
- 以下の有限計測コマンドは終了済み。対象主要PIDの継続を確認。ROSノード・サーバーの新規起動なし、失敗したperfの空ファイルは削除。

```bash
pidstat -u -p 1845486,1844918,1844966,1844494,1845334,1754609,1845139,1845484,1845141,1845222,1754613,1841990 2 5
timeout -s INT -k 3s 12s perf record -F 49 -g -p 1845486,1844494 -o /tmp/grasp_cpu_20260914.perf -- sleep 5
```

## 2026-09-14: 候補ロボットの表示数制限

- 候補ロボット欄に既存スライダーを追加。配信順の先頭N件、0は全件。ROS側の評価・選定・配信への変更なし。[仕様・検証コマンド](releases/2026-09-14_candidate_robot_display_limit.md)を記録。
- 候補数・順序更新・全件復帰・空配信・TF維持と既存ロボット色・Markerの回帰検証に成功。frontend lint・本番ビルド・backendビルドに成功。通常ビルドの権限不足・DockerのMCAP依存不足を記録し、依存や権限を変えず検証経路を切替。実画面操作は未検証。
- 検証コマンドは全終了、一時ファイルを削除。ROS・サーバーの新規起動や既存プロセスの停止・再起動操作なし。

## 2026-09-14: 候補ロボットの全件復帰・単体選択GUI

- 「全件に戻す」「先頭N件」「1体選択」を追加。番号スライダーで単体候補を指定し、全件復帰時は件数制限と単体選択を解除。Collisionの表示対象も選択候補へ一致。[現行仕様・検証コマンド](releases/2026-09-14_candidate_robot_display_limit.md)を更新。
- GUIコールバック、候補番号変更、候補数減少・空配信、全件復帰、TF維持と既存ロボット色・Markerの回帰検証に成功。frontend lint・型チェック・本番ビルドに成功。実ブラウザ操作は未検証。
- 全検証コマンド終了、一時出力・テスト用ファイルを削除。既存Frontendサーバーを維持。ROS・サーバーの新規起動、既存プロセスの停止・再起動なし。

## 2026-09-14: 候補経路探索の重複削減

- 安全ゴール間での開始候補別Dijkstra共有と探索配列化を実装。終点例外・隣接危険度ペナルティ有効時は個別探索を維持。[仕様・測定条件・全起動コマンド](releases/2026-09-14_candidate_path_batch.md)を記録。
- Dockerビルド、C++テスト6件、domain 218のROS結合テストに成功。実GNGの5開始候補×8ゴールの全経路一致、追加3回の探索時間中央値1,370.62 msから286.57 msを確認。変更後の個別・共有比較であり、旧版バイナリや稼働全体のCPU改善率とは区別。
- 検証launch・子ノードと有限計測は全終了済み、最終プロセス一覧で残留なし。既存ROSへの停止・再起動操作なし。調査中に外部からの候補計画launch起動を観測し、そのプロセスは維持。

## 2026-09-14: 候補経路のエッジコスト共有

- 1回の計画内で開始候補間の有向エッジ判定・基礎コストを再利用する遅延キャッシュを実装。安全制約・終点別ペナルティは維持し、単一開始候補時はキャッシュ生成なし。[現行仕様・全起動コマンド](releases/2026-09-14_candidate_path_batch.md)を更新。
- Dockerビルド・C++7件・domain 218の既存ROS結合テストに成功。実GNGの全40経路一致、キャッシュ寿命と更新後の再評価を確認。3回の同一実行内比較の中央値は共有方式271.269 ms、追加キャッシュ方式107.392 ms。全体CPU改善率は未測定。
- 検証用launch・子ノード・比較計測はすべて終了済み。既存ROSの停止・再起動操作なし。新たな設定・トピック・メッセージ変更なし。

## 2026-09-14: 固定GNG索引・探索木再利用による候補経路高速化

- 候補専用ノードに固定グラフ索引と再開可能な探索木を追加。通過可否変更時の失効、例外終点別の独立木、CPU数を上限としたジョブ並列化を実装。[仕様・全検証コマンド](releases/2026-09-14_candidate_static_path_index.md)を記録。
- Dockerビルド・C++8件・domain 218のROS結合テストに成功。実GNGの5開始候補×8終点で全40経路一致。各5回の初回探索は安全終点2.99〜3.91 ms、危険終点6.33〜8.49 ms。安全状態更新後も各回10 ms未満、再利用は最大0.240 ms。索引準備31〜41 msは別計測。ROS配信・描画込みの時間とは区別。
- 実運用の6候補が全て危険ラベルだった時点を観測。同時刻スナップショットの取得は候補ID配信元不在で失敗し、危険終点の性能検証は保存済みGNGの制御条件として実施。全体CPU・実運用遅延の改善率は未測定。
- プローブ・検証launch・子ノード・有限ベンチマークはすべて終了済み。既存ROSへの停止・再起動操作なし。トピック・パラメータ追加なし。

## 2026-09-15: 候補軌道計画の計算時間ログ

- 計画更新時の計算時間・目標候補数・到達数をINFOの1行に集約し、起動時の詳細一覧と候補受信ログをDEBUGへ移動。[計測範囲・検証・起動コマンド](releases/2026-09-15_candidate_planning_log.md)を記録。
- Releaseビルド・インストールとdomain 218の既存ROS結合テストに成功。計算時間ログ7件、静止入力での再探索なし、通常ログの簡潔化を確認。検証用launch・子ノードはすべて停止済み。
- Applied the requested English format `dof=7 Plan: 34.35 ms Count: goal=2 reach=2` and removed the separate startup INFO log. Release library rebuilds passed; no ROS processes were started for these wording changes.

## 2026-09-15: Surface clustering cost and incremental updates

- Observed the existing stream and captured 21 synchronized frames with 1,545 nodes. Mean surface extraction was 9.196 ms at about 1.9 Hz; all frames exhausted the 128-fit budget. Exact unchanged-patch count was zero across 200 comparisons. [Measurements, limitations, artifacts, and startup commands](designs/curved_surface_position_fit.md#2026-09-15-live-cost-and-incremental-update-investigation).
- Compared three isolated Release prototypes on identical inputs. Total reductions were 1.4–2.8%; each passed 66 existing tests. [Production adoption deferred](reject.md#2026-09-15-surface-clustering-micro-optimizations-and-exact-patch-cache). No production source or setting changes for this investigation.
- Capture/probe/build/replay/test processes all exited. Existing GNG launch and nodes remained running; no user process was stopped or restarted.

## 2026-09-15: モデル当てはめなしの連続面抽出

- `surface_method:=smooth_graph` を追加。実GNGエッジの距離・法線・接平面条件と影響成分の差分再探索を既存ファイル内に実装。[仕様・比較条件・全起動コマンド](releases/2026-09-15_smooth_surface_graph.md)を記録。
- Releaseビルド・C++74件・隔離ROS検証に成功。実入力21フレームで所属と採用エッジが毎回全探索と一致。従来modelの所属・種別・IDも変更前結果と一致。
- 従来7.407 msに対して新方式1.159 ms。ただし最大成分は平均1,420/1,545ノードへ拡大。差分管理単独は実入力で3.7%の増加、固定入力では判定・所属再探索の省略を確認。分割品質を理由に既定値modelを維持。
- 検証ROSノード・driver・再評価・ビルド・描画はすべて終了、プロセス一覧で残留なし。既存プロセスの停止・再起動操作なし。結果・比較図は `tmp/surface_graph_20260915/` に保存。

## 2026-09-15: 手元の候補経路高速化とリモート更新の統合

- ユーザー承認に基づき、手元のステージ済み10ファイルを`a957cdc`へ保存。最新fetch後の`origin/grasp_new4`（`cf17573`、計測ログ・連続面抽出の2コミット）を統合。ソースは自動マージ、進捗文書の末尾追記競合は双方の記録を保持して解消。
- Docker内で候補計画・曲面抽出のビルド、C++8件と74件、domain 218のROS結合テストに成功。候補ロボットの基準座標、混在入力、静止時の再計画抑止、関節変更・明示要求、空入力のクリアと関節目標配信なしを確認。
- 下記の有限コマンドと検証用launch・子ノードはすべて終了済み。既存ROSへの停止・再起動操作なし。確認中に別の既存シェルからの候補計画launch起動を観測し、そのプロセスは維持。リモートへのpushなし。

```bash
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 360s cmake --build /ros2_ws/build/gng_vlut_system --target test_candidate_metric_availability topological_map_planning -j2 && timeout -s INT -k 5s 120s /ros2_ws/build/gng_vlut_system/test_candidate_metric_availability && timeout -s INT -k 15s 360s cmake --build /ros2_ws/build/ais_gng --target test_surface_model plane_cluster_incremental_node -j2 && timeout -s INT -k 5s 120s /ros2_ws/build/ais_gng/test_surface_model --gtest_color=no'
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 25s 180s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py'
```

結合テスト内の`ros2 run gng_vlut_system safety_monitor_node`と`ros2 launch gng_vlut_system grasp_joint_candidates.launch.py`の全引数は[検証起動コマンド](releases/2026-09-14_candidate_static_path_index.md#verification)と同一。

## 2026-09-15: 把持幅・TCP姿勢・関節姿勢の独立した追加評価

- 既存候補を購読する別ノードと専用出力を追加。局所実点群による幅・接触・グリッパ掃引評価、既存関節候補を初期値にしたIKを実装。[仕様・制限・全起動コマンド](releases/2026-09-15_grasp_candidate_refinement.md)を記録。
- Releaseビルド、新規8件・既存16件のC++検証と隔離ROS結合検証に成功。合成40 mm対象・実URDFで40/46 mmの接触幅・開口とIK補正、既存出力への非干渉、欠損・失効・不正入力時の無効化を確認。
- 指定bagの実点群20フレームを既存GNG・上方候補生成へ投入。追加評価は48更新・最大14候補で平均11.569 ms、最大18.701 ms（IKなし）。観測幅457件、確定接触対0件。実点群での把持成立、補正した腕の衝突・経路は未検証。
- 検証driver・launch・子ノード・有限計測は全終了済み、開始前後のプロセス一覧で残留なし。既存プロセスの停止・再起動操作なし。結果は `tmp/grasp_refinement_20260915/` に保存。

## 2026-09-15: 追加pushされた把持候補補正の再統合

- 最新fetch後の`origin/grasp_new4`（`fd44919`）を前回の統合結果`1a314f7`へ統合。前回の高速化・計測ログ・曲面抽出を保持。ソース・仕様書は自動マージ、進捗文書の追記競合は双方を保持して解消。
- Dockerで専用メッセージ生成、新規ノードと既存計画のReleaseビルド・インストールに成功。C++は新規補正8件、到達性9件、経路計画8件の計25件成功。domain 218で新規補正と既存候補計画のROS結合テストも成功。
- 合成対象の接触幅40 mm・開口46 mm・IK補正、既存トピックへの非干渉、欠損・衝突・TF失効復帰・入力停止・不正入力・空入力の処理を確認。実点群での把持成立は今回の検証対象外。
- 下記コマンドと検証用launch・子ノードはすべて終了済み。一時結果・ROSログの専用ディレクトリを削除し、テスト残留なしを確認。既存ROSの停止・再起動操作、pushなし。

```bash
docker exec -w /ros2_ws gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 240s colcon build --packages-select gng_control_msgs --symlink-install --executor sequential && source /ros2_ws/install/setup.bash && timeout -s INT -k 15s 120s cmake -S /ros2_ws/src/gng_vlut_system -B /ros2_ws/build/gng_vlut_system && timeout -s INT -k 15s 480s cmake --build /ros2_ws/build/gng_vlut_system --target grasp_candidate_refiner_node test_grasp_refinement test_grasp_candidate_reachability test_candidate_metric_availability topological_map_planning -j2 && timeout -s INT -k 5s 120s ctest --test-dir /ros2_ws/build/gng_vlut_system --output-on-failure -R "^(test_grasp_refinement|test_grasp_candidate_reachability|test_candidate_metric_availability)$"'
docker exec -e ROS_DOMAIN_ID=218 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/tmp/grasp_merge_check_Fs4LBp/ros gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash && timeout -s INT -k 10s 120s cmake --install /ros2_ws/build/gng_vlut_system && timeout -s INT -k 20s 120s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_refinement.py --output /tmp/grasp_merge_check_Fs4LBp/result.json && timeout -s INT -k 25s 180s python3 /ros2_ws/src/gng_vlut_system/test/check_grasp_joint_candidates_integration.py'
```

新規検証の子launchは`ros2 launch gng_vlut_system grasp_candidate_refinement.launch.py params_file:=/tmp/grasp_refinement_0vwzlmg0/params.yaml candidate_topic:=/grasp_refinement_test/source seed_topic:=/grasp_refinement_test/seeds point_cloud_topic:=/grasp_refinement_test/points output_topic:=/grasp_refinement_test/result`。既存候補計画テストの子launchは[前回の起動コマンド](releases/2026-09-14_candidate_static_path_index.md#verification)と同一。

## 2026-09-15: 候補独立ビューのXYZ軸切替

- 独立3DビューのXYZ軸を既定OFFとし、チェックボックスを追加。主画面・候補選択・XYZ寸法表示は維持。[仕様・検証コマンド](releases/2026-09-15_inspection_axes_toggle.md)を記録。
- frontendのlint・本番ビルドに成功。ブラウザの実操作は未検証。有限コマンドは終了、一時出力は削除済み。ROS・開発サーバーへの起動停止操作なし。

## 2026-09-15: 候補独立ビューの情報整理

- フッターをノード数・エッジ数・XYZ寸法の1行へ集約し、座標系の説明を削除。`nonplane_components`の表示だけを`nonplane_`へ短縮。[仕様・検証コマンド](releases/2026-09-15_inspection_info_layout.md)を記録。
- HTML出力テスト・lint・本番ビルドに成功。元source不変も確認。実ブラウザ描画は未検証。有限コマンドは終了、一時出力は削除済み。ROS・開発サーバーへの起動停止操作なし。

## 2026-09-15: 独立ビューのBbox切替

- 独立ビューへ既定OFFのBbox表示を追加。受信済みの範囲から線枠を描画し、主画面設定は維持。[仕様・検証コマンド](releases/2026-09-15_inspection_bbox_toggle.md)を記録。
- HTML出力テスト・lint・本番ビルドに成功。実ブラウザの切替・描画は未検証。有限コマンドは終了、一時出力は削除済み。ROS・開発サーバーへの起動停止操作なし。

## 2026-09-15: 主画面Bboxと独立表示の連動

- Bbox OFF時にも残っていたノード・クラスタ・Marker直接選択を、トピック別フラグへ連動。詳細取得の開始・応答でも確認し、OFF後の遅延表示とエラーを抑止。`/grasp_pose_cands/Tmap`だけ既定ONへ変更。[仕様・全検証コマンド](releases/2026-09-15_inspection_bbox_gate.md)を記録。
- 関連frontendテスト5ファイル、lint・本番ビルド、Dockerのbackendビルドに成功。Appの実接続条件と非同期処理、明示OFFの保持、既存ホバー・Marker・独立ビュー内Bbox既定OFFを確認。実ブラウザ操作は未検証。
- 検証コマンドは全終了、一時テスト・build出力を削除。ROSノード・開発サーバーの新規起動や既存プロセスの停止・再起動操作なし。

## 2026-09-15: 接続小平面の合算と未指定Bbox GUIの非表示

- 非平面経由の接続を再利用し、起点の最低ノード数・傾斜角と合算対象の寸法判定を分離。小平面・側面を含む候補Tmapを検証。[仕様・Docker実行コマンド](releases/2026-09-15_grasp_small_plane_membership.md)を記録。
- Bounding Box未指定のGraphではGUIなし、明示falseでは再ON可能とし、通常`/topological_map`を対象外へ変更。[仕様・frontend実行コマンド](releases/2026-09-15_bbox_explicit_controls.md)を記録。
- Docker Releaseビルド・C++回帰検証・CTest、frontend関連4ファイルのテスト・lint・本番ビルドに成功。初回ホバーテストの旧false期待値を更新して再検証成功。実入力・実ブラウザ・実機動作は未検証。
- 有限コマンドは全終了、一時出力は削除済み。既存ROS PIDを維持し、検証プロセスの残留なし。frontendコンテナの稼働期間更新を観測したが、本作業からの起動停止操作なし。

## 2026-09-15: Viewerの配信元停止・再起動追従

- 停止時の旧レイヤー・送信待ち・描画完了待ちを消去し、Streams選択を維持した同名配信元の自動再購読を実装。[変更内容・検証コマンド](releases/2026-09-15_viewer_stream_restart.md)を記録。
- Gatewayビルド、別ドメインでの配信プロセス停止・再起動、未ACKグラフ・点群・QoS変更Marker・即時再起動の復帰検証に成功。Reactフックの状態消去・復帰テストとlintにも成功。
- 通常frontendビルドの依存不足・設定キャッシュ権限による失敗を記録。ホストのTypeScript検査と、設定を直接読み込むViteでの一時ディレクトリへの本番アセット生成に成功。実ブラウザ描画は未検証。
- 検証用Gateway・配信元は終了し、既存Gateway PID 96433を維持。frontendの外部操作による停止・再起動を観測したが、本作業からの起動停止操作なし。

## 2026-09-15: 配信元再起動時の可視化設定保持

- 点群の表示設定を受信バッファと分離し、旧データ削除後の同名トピック復帰時に再適用。グラフの受信ごとの色変更・属性欠落による表示設定の自動OFFを廃止。[仕様・検証コマンド](releases/2026-09-15_viewer_stream_restart.md#表示設定保持の追加検証)を更新。
- 再起動回帰2件、Bbox回帰3件、lint、TypeScript検査、本番アセット生成に成功。非表示・透明度0・手動変換・グラフ設定の保持、新点群バッファへの置換を確認。実ブラウザ描画は未検証。
- 有限テスト・ビルドは終了、一時出力は削除済み。ROS・サーバーの新規起動や既存プロセスの停止操作なし。

## 2026-09-15: 上方把持候補の凸包・回転包含判定

- 平面PCAと複合候補の固定軸を廃止し、2D凸包・支持点切替区間による開口包含判定へ置換。[仕様・検証コマンド](releases/2026-09-15_convex_grasp_footprint.md)を記録。
- Docker Releaseビルド・CTest・AddressSanitizer/UndefinedBehaviorSanitizer検査に成功。回転形状、合算時の再回転、重複点不変性、退化形状、ランダム120件と独立角度走査の整合を確認。
- 32平面・3,072ノードの合成入力で推定時間の中央値0.125 ms（内点あり）・0.620 ms（全点が凸包頂点）を測定。実入力・Viewer・実機把持は未検証。
- 検証プロセスは全終了、一時出力は削除済み。既存プロセスへの起動停止操作なし。外部再起動後の推定器が今回のビルド結果を使用していることを確認。

## 2026-09-15: HTML・Viewerの把持ラベル統合

- HTMLの`/handle_points`専用配信を削除し、把持部位生成を残して`/semantic_points`へ集約。HTMLの出力ラベルを0/1に揃え、縁・蓋・机等と到達性2〜4の衝突を解消。[仕様・検証コマンド](releases/2026-09-15_grasp_label_unification.md)を記録。
- Viewerの「把持ラベル」配下に把持部位・未評価・到達範囲内・到達範囲外を統合。個別色・表示状態と旧設定の移管を確認。
- 実HTML関数の点群生成、ラベル解決・移管、モーダル構造、実Chromeの操作の4検証とlint・型検査・本番アセット生成に成功。HTML→ROS→GNGの実通信は今回未検証。
- `node tests/label_priority_browser.test.mjs`で起動したChrome PID 1060265は停止済み。専用プロファイル・一時ビルド出力を削除。既存ブラウザ・ROSノードの起動停止操作なし。

## 2026-09-15: SpatialTree比較とTmap目標選択索引

- 通常版とSpatialTree2を実Tmapの10,801ノードで比較。同一数値型でのバイナリ一致・検索不一致0を確認し、既存通常版doubleを採用。[仕様・比較結果・全検証コマンド](releases/2026-09-15_static_spatial_index.md)を記録。
- 静的索引の再利用と回転セルの外接箱検索を実装し、既存の選定条件・結果を保持。実Tmap座標と合成候補20件の目標選択中央値は1.351 msから0.485 msへ短縮。
- Docker Releaseビルド、12件の回帰検証、AddressSanitizer/UndefinedBehaviorSanitizer、隔離ROSでの実GNG候補・経路出力確認に成功。実画面・実機把持は未検証。
- 検証プロセスは全終了、一時コピー・実行ファイル・ROSログを削除し、再現用の座標・結果ログを保持。既存目標選択ノードPID 497597を維持。AIS・frontendの外部再起動を観測したが、本作業からの起動停止操作なし。

## 2026-09-15: 把持補正の無表示調査と理由表示

- 実入力は受信・結果配信済みで、観測時4候補の3件が局所点数超過、1件が支持不足と確認。未受信・棄却理由の低頻度INFOと、幅未計算候補の理由Markerを追加。[仕様・全検証起動コマンド](releases/2026-09-15_refinement_visibility.md)を記録。
- Docker Release、幾何8件、隔離ROSの既存幅・IK補正と追加表示テストに成功。実入力で30,000/100,000点予算の別出力も確認し、上限増加だけでは接触対・IK成立0件。判定条件・既定予算は変更なし。
- 実入力プローブの非同期最新同士の照合失敗後、同一時刻照合へ修正して再検証成功。実画面・実機把持は未検証。
- 検証プロセスは全終了、一時プローブ・専用ROSログを削除。既存補正PID 1443642と関連ROS・コンテナの稼働を維持し、起動停止操作なし。結果は`tmp/refinement_visibility_20260915/`へ保存。

## 2026-09-15: 非平面付属部分の棄却条件に関する検討事項の記録

- 接続する非平面部分の把持領域超過による棄却について、不要な可能性があるというユーザー指摘を [TASK_CANDIDATES.md](TASK_CANDIDATES.md) に記録。コード・設定の変更なし。

## 2026-09-15: Viewerの把持補正理由が非表示となる原因の修正

- 実ROSでは理由Marker配信・Gateway購読済みだったが、WS本文欠落と文字描画未対応を確認。トピック非依存の標準文字Markerとして修正。[仕様・全起動コマンド](releases/2026-09-15_viewer_text_markers.md)を記録。
- Dockerビルド、隔離ROSの本文・再起動確認、既存Marker回帰、lint、型検査・本番生成に成功。通常Viteビルドの既存キャッシュ権限エラーはrunnerで回避。
- 実WSデータ5件を専用Chromeで描画し、スクリーンショットを目視確認。把持成立0件は継続し、判定条件は変更なし。実機把持は未検証。
- 検証ノード・Chrome・ポートは全終了、一時スクリプト・ログ・ビルド出力を削除。既存プロセスの外部再起動を観測したが、本作業からの停止・再起動操作なし。結果画像は`tmp/marker_text_20260915/`に保存。

## 2026-09-15: 把持補正Markerの幅・方向描画

- `/grasp_pose_refined/markers`の文字を指内面の線枠と進入矢印へ置換。確認済み幅は実線、観測幅だけは破線、幅未計算は方向のみとし、トピック・判定条件を維持。[仕様・全起動コマンド](releases/2026-09-15_grasp_geometry_markers.md)を記録。
- Docker Release・幾何8件・ROS結合検証に成功。40 mmの幅、回転姿勢、指寸法、破線、衝突色、文字なしを確認。実入力の7形状を専用Chromeで描画し、画像を目視確認。実入力の接触対・IK成立は0件のままで、実機把持は未検証。
- Viewerビルド・lint成功。通常frontendビルドの既存キャッシュ権限エラー後、runner指定の型検査・本番生成に成功。
- 検証ノード・Chromeは全停止、一時ファイル・専用ポートの残留なし。既存プロセスの起動停止操作なし。結果画像・ログは`tmp/grasp_geometry_20260915/`へ保存。

## 2026-09-15: Tmap集約L0の配信復旧

- 旧version 4の集約binとversion 5 readerの不一致を確認。既存trainerで10,801元ノードから150ノード・554エッジを再生成し、同じlaunchでの配信を復旧。[仕様・全検証起動コマンド](releases/2026-09-15_tmap_l0_restore.md)を記録。
- Docker Releaseビルド、保存後再読込、隔離ROSで元・集約マップの同時配信、所属・座標・エッジ・frame一致、Viewer向けWS受信を検証。連結成分1・孤立0。ブラウザ目視は未検証。
- 元GNG・VLUTのSHA256不変を確認し、旧集約binを退避。新しいROSノード・メッセージ・起動時学習は追加なし。
- trainer・検証launchの全子ノード・Gatewayを停止し、専用ROSログ・待受けポートの残留なし。既存プロセスの停止・再起動操作なし。再現用資料は`tmp/tmap_l0_20260915/`へ保持。

## 2026-09-15: 集約L0のガタつきに関する読取調査

- 新旧binの150座標が完全一致し、元coord edge縮約結果をFK補間の空間最近傍対応で上書きする現行処理を確認。新旧エッジ長を集計し、[追加調査](releases/2026-09-15_tmap_l0_restore.md#集約形状の追加調査)に記録。
- 前回の配信検証と形状品質を区別し、仕様書の連続性保証に関する過剰な記述を修正。生成処理・設定・binの変更なし。ROS・常駐プロセスの起動なし。

## 2026-09-15: 集約L0の空間所属・重心・元接続保持

- 空間のみの既定所属、重心描画、元coord edge縮約とFK遷移メタ情報の分離を実装。10,801元ノードから150ノード・740エッジへ再生成。[仕様・全検証起動コマンド](releases/2026-09-15_spatial_tmap_aggregation.md)を記録。
- Docker Release・GTest 5件・隔離ROS/WS検証に成功。全元ノードの一意所属、元トピックの縮約edgeとの完全一致、連結成分1・孤立0を確認。所属点距離RMSは219.2 mmから46.2 mmへ改善。長い元接続は保持。
- 現行Viewer描画で3方向の前後比較を目視確認。元GNG・VLUTのSHA256不変、共有の到達可能ボクセルtrainerのビルド成功を確認。到達可能ボクセルbinの更新は未実施。
- 起動した検証launch・Gateway・Chromeは全停止、専用ログ・profileを削除し、ポート解放を確認。既存通常bridgeの退出を観測したが、本作業から既存プロセスの停止・再起動操作なし。結果は`tmp/tmap_spatial_20260915/`へ保存。

## 2026-09-16: 一ノード周辺の並進・回転空間のFK試験

- 独立したPython一ファイルで、関節角差の小さい順のFK評価と3パネルの3D可視化を実装。ROS・シミュレーション・refineへの統合なし。[条件・結果・起動コマンド](releases/2026-09-16_local_fk_motion_experiment.md)を記録。
- 元GNGノード0の周辺4,169,977通りを評価。GNGと同じ手先+X軸の平行30,175点、向き全体維持2,102点、位置維持の回転566点を確認。各数は元ノードを含み、姿勢残差0.25度・位置残差0.5 mmの許容による分類。
- 解析解・逐次FK・元GNG方向との一致、格子順序・関節限界・辺の補間を検証。向き全体維持グラフは全2,102表示点が連結。軸平行グラフは2,999表示点が元ノードと連結、1表示点が孤立。HTMLのスライダー・追加アニメーションと描画を確認。衝突・大域的な可動限界は未評価。
- Python・検証Chromeは全終了、専用ブラウザprofileを削除。既存プロセスの停止・再起動操作なし。数値・画像・単独HTMLは `tmp/local_fk_motion_20260916/` に保存。
- 保存結果をPlotlyの3Dグラフで確認する表示専用Pythonと単独 `plotly.html` を追加。3分類切り替え・全採用点表示・L2スライダー・投影と視点操作へ対応。[利用方法・描画検証](releases/2026-09-16_local_fk_motion_experiment.md#plotlyによる3dグラフ表示の追加)を記録。ブラウザーの全点数・分類・視点検証と描画確認に成功。検証Python・Chromeは終了、プロセス残留なし、専用profileを削除。
- ユーザーの意図に合わせ、特定軸の平行分類を削除し、FKの並進量・姿勢全体の回転量による[並進的／回転的な2分類](releases/2026-09-16_local_fk_motion_experiment.md#並進成分と回転成分による2分類への修正)へ修正。再評価で2,102／566点、重複4点、混合4,167,313件を確認。旧結果の該当サンプル・辺との一致、座標基底によらない成分の大きさ、補間・ブラウザー描画を検証。旧結果を退避し、Python・Chromeは終了、プロセス残留なし、専用profileを削除。
