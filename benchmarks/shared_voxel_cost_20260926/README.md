# world・ROI・FVGの処理コスト測定（2026-09-26）

## 要約

点群保持に加えてセル別点数集計も共通化し、Docker内の通常配布先へ反映。
同一設定・同一snapshotは複数FVGで共用。有界セルは連続索引、過大範囲はhash。
未使用の履歴・256ラベル配列を撤去し、Marker生成を統合・キャッシュ。
world／ROI／FVG共通のbucket検索で、広域の空bucket列挙も省略。
点数・解像度・更新周期・判定式は維持。CPU時間は全スレッド合計で、経過時間とは異なる。

| 通常launch、近傍ROI、Tmap 5,000ノード、可視化あり | 今回変更前→後 CPU ms/入力 | 変更前→後 ピークRSS MiB |
| --- | ---: | ---: |
| 固定10万点・広域FVG | 46.9 → 23.5 | 154〜158 → 160 |
| 固定20万点・広域FVG | 78.1 → 39.5 | 201〜203 → 205〜221 |
| 固定20万点・近傍FVG | 26.4 → 26.0 | 168〜182 → 174〜177 |
| 動く20万点・広域FVG | 91.5 → 43.9 | 312〜328 → 222〜227 |

固定入力は今回直前の再測定、動的入力の変更前は同日先行測定との比較。
CPU固定割当なし・既存処理と併走のため試行時期の変動あり。連続索引はメモリとの交換条件。
配列上限は`max_dense_voxel_num`、1セル4バイト／バッファ。0でhash、点数切捨てなし。
初期版の固定20万点119.4 ms/入力・約2.1 GiBと前段最適化は[過去集計](summary.json)・
[前段集計](summary_optimized.json)に保持。以下は今回の最終実装。

| 2スレッド計測版、20万点・近傍ROI | CPU ms/入力 |
| --- | ---: |
| FVGなし：world索引＋ROI、可視化なし | 20.1 |
| ＋FVG点数集計、Tmap・可視化なし | 30.4 |
| ＋Tmap、可視化なし | 32.4 |
| ＋worldバケット・FVG Marker配信 | 42.3 |

10万点のFVGなしは10.4 ms/入力。FVGなしではセル集計用配列の確保なし。
20万点・全配信のFVGタイマー平均は前段42.9→15.2 ms、更新後p95は19.1〜20.2 ms。
今回の内訳：点数集計9.41 ms、セル更新1.28 ms、評価＋Marker生成2.71 ms、
Marker配信1.79 ms。Marker生成2.60 msは評価区間の内数で、重複加算不可。
計測区間の移動があるため旧label区間との単純比較不可。近傍FVGのタイマーは0.48 ms。
広域80,411セル、近傍612セル。固定20万点のMarker受信量は約18.6 MiB/sで通信負荷は残存。

通常launchの固定20万点・入力→Marker平均遅延は104.1→74.1 ms、動的は109.2→51.8 ms。
ROIは全条件80/80、FVGは固定80/80、動的79/80。最新フレーム省略と観測末尾の未完了を含む。
ROIまで広域の2スレッド版は前段146.3→107.0 CPU ms/入力、ROI区間81.0→67.5 ms。
ROI平均遅延399→100 ms、今回はROI・FVGとも80/80。ただし広域ROIの余裕は小さい。
異なるTF／解像度のROI占有集計やGNG内部索引を同一セル配列へ統合した変更ではない。

## 条件・検証

- i7-14650HX、24論理CPU、`no_turbo=1`、`powersave`、Humble、Release＋LTO。
  既存bag・Viewer稼働。通常launchは36スレッド。executor制限・allocator設定の本番変更なし。
- 合成入力：固定seed、路面70%・壁面15%・立体点15%、XYZ＋intensity、10 Hz。
  動的入力は全点へx/y/z振幅0.08/0.06/0.02 mの変位、Tmapも同じworld変位。
  GNG学習・VLUT・Viewerブラウザ・入力生成器のCPUは対象外。
- 広域：x/y±12 m、z=-1〜4 m。近傍：x=-1.8〜0.8 m、y=-0.455〜0.355 m、z=0〜2.95 m。
  FVG幅0.1 m、ROI幅0.02 m、bucket幅0.2 m。各座標系の非零TFあり。
- 各条件10フレームwarmup＋40測定×2試行。RSSは約5秒の試行内標本最大、長期上限ではない。
  CPU・RSSは対象プロセスの`/proc`、区間はソースコピーへの計測器挿入。
  可視化OFFはコピー内Marker生成省略。ON時はCDR実購読。ROI出力は全条件有効。
- 共通ストア7件・既存world／ROI21件と[ROS回帰試験](../../fuzzy_voxel_grid/test/README.md)が成功。
  変更前・通常配布先のdense/hashでセル全フィールド・Marker位置／色が一致。
  複数FVGの購読1件、動く点群・空入力・Tmap単独セル・freeze属性保持も確認。
  実bag認識品質・Viewer実描画・長時間・ホストビルドは未検証。
- 初回の動的入力生成遅延はPythonの`array('B')`化で修正。今回のCMake探索失敗は
  `voxel_idx_DIR`指定で修正し再測定。失敗ログも保持。出力条件を緩めた回避なし。

ROS・通常install・FVGのlocal_setupをsource後、workspaceの`src`で実行。

```bash
export SHARED_COST_VARIANT=common_final ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1
python3 benchmarks/shared_voxel_cost_20260926/prepare.py
cmake -S benchmarks/shared_voxel_cost_20260926 -B artifacts/shared_voxel_cost_20260926/common_final/build \
  -DGENERATED_SRC=/ros2_ws/src/artifacts/shared_voxel_cost_20260926/common_final/generated \
  -Dvoxel_idx_DIR=/ros2_ws/install/voxel_idx/share/voxel_idx/cmake
cmake --build artifacts/shared_voxel_cost_20260926/common_final/build -j2
timeout -s INT -k20 300 python3 benchmarks/shared_voxel_cost_20260926/run.py --frames 40 --trials 2
timeout -s INT -k20 120 python3 benchmarks/shared_voxel_cost_20260926/run.py --installed --frames 40 --trials 2
timeout -s INT -k20 90 python3 benchmarks/shared_voxel_cost_20260926/run.py --installed --moving --frames 40 --trials 2
python3 benchmarks/shared_voxel_cost_20260926/summarize.py
```

内部起動は`shared_voxel_cost --ros-args --params-file ... --log-level warn`、
`ros2 launch fuzzy_voxel_grid shared_world_voxel.launch.py ...`。全引数はログのSTART行。
全試験グループと子を終了、既存bag・Viewer・3コンテナのPID／IDを維持。
正本：[今回集計](summary_common_final.json)。生ログ・CSV・ソースhashは
`artifacts/shared_voxel_cost_20260926/common_final/`、変更前は`optimized/before_common_installed.json`。
