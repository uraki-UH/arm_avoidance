# 2026-09-26 - worldバケットとFVGの共通点群経路

## 1. 要約

world索引を`voxel_idx`のROS・fuzzy非依存ライブラリへ移し、
worldとFVGが同じ点群スナップショットを参照できる構成を追加。
worldバケット単独利用にFVGやfuzzy評価を必須化しない。

- FVGのセル内XYZコピーとタイマーによる再走査を除去し、点数・評価情報を保持。
- 同一プロセスの共有launchを追加。worldだけがPointCloud2を購読・world変換・索引化。
- 読取中のスナップショットは不変。旧バッファは参照解放後に再利用。
- 元PointCloud2の共有所有権を保持し、intensityなどの属性を維持。
- world検索bucket、ROI占有セル、FVG評価セルの幅・原点・座標系は独立。
- セル別点数も共通化。同一設定・同一snapshotの複数読者は同じ集計結果を参照。
- 有界セルは連続索引、過大範囲はhash。広域検索は空bucketの列挙を省略。
- FVGの未使用履歴・ラベル集計配列を撤去。表示とfreezeのMarker生成を統合・キャッシュ。

共通化対象は現フレーム点群の保持・検索・設定別セル点数集計。
異なる解像度の集計やROI占有ID、FVG評価状態まで単一配列へ統合する変更ではない。
GNG内部索引・persistent depth固有履歴・新しい重点サンプリング式は対象外。

## 2. 条件・検証

| 項目 | 内容 |
| --- | --- |
| 共有条件 | 同一プロセス・同じ`shared_point_store`。writerは1つ |
| 既定動作 | 空のstore名で独立動作。既存launchを維持 |
| 新規launch | `fuzzy_voxel_grid shared_world_voxel.launch.py` |
| world出力 | `/world_index/buckets`・`/roi_voxels`。YAMLで変更可能 |
| Tmap | 共有点群と同じframe、非未来、`max_tmap_age_sec`以内。既定1秒 |
| TF | センサー→world、world→targetが必要。未接続時の座標流用なし |
| freeze出力 | 判定はworld座標、出力は元入力の座標・header・属性 |
| 配送 | FVG周期で最新1件。遅延時の全フレーム処理保証なし |
| 配列上限 | `max_dense_voxel_num: 8000000`。0でhash。出力の切捨てなし |
| 診断ログ | 全セル要約はDEBUG限定。処理時間の既存設定は維持 |
| ビルド | `voxel_idx`・`gng_vlut_system`・FVGのHumble Release成功 |
| 単体試験 | 共通ストア7件、既存world／ROI回帰21件が成功 |
| ROS試験 | 共有経路、FVG単独、world単独の起動・出力成功 |

異なるTF、異方性セル、非零原点、除外箱、Tmap時刻・frame不一致、
freeze/resumeのintensity保持、空入力・巻戻しを独立計算と照合。
100,000点と空フレームの交互16入力でも全セル件数の一致を確認。
共通化前・隔離ビルド・通常配布先のdense/hashで、セル全フィールド・Marker位置／色が一致。
複数FVGの同一結果・入力購読1件、集計の同時読取・旧結果の不変性も検証。
動く点群、Tmap単独セル更新、狭域bucket検索も照合し、Docker内の通常配布先へ反映。
固定・動的な合成入力でCPU・RSS改善を確認。FVGなしのworld／ROIも再測定。
条件・実測値は[性能測定](../../../benchmarks/shared_voxel_cost_20260926/README.md)に集約。
実bag品質・実Viewer・長期メモリ上限・ホストビルドは未検証。

初回ビルド／起動のAPI・リンク・登録スコープの問題は修正後に再検証。
試験用ROSプロセスは子プロセスを含め全終了。既存bag・Viewer・コンテナを維持。
FVGの`COLCON_IGNORE`は保持し、通常workspaceビルドへの強制追加なし。
`environment_to_vlut.launch.py`全体の共有起動への自動置換は行わない。
共有launchと既存worldノードを同一入力で重ねて起動しないこと。

仕様・利用手順：[FVG README](../../../fuzzy_voxel_grid/README.md)。
再現コマンド・試験範囲・失敗記録：[検証手順](../../../fuzzy_voxel_grid/test/README.md)。
