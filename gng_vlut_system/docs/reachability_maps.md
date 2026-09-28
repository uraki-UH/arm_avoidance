# 経路GNGから独立した左右到達map

## 要約

2026-09-28、max / longの左右到達域をURDFから直接生成し、既存GNG・VLUTと分離。
`reachability_voxel_builder`を再利用し、平均化せず、セルごとに実際の代表関節角を保持。
低差異列で初期探索後、別の乱数列で未登録セルを補完。最後の独立検査中はmapを更新しない。

| モデル・腕 | 到達セル数 | 独立検査のセル一致率：補完前→後 |
| --- | ---: | ---: |
| max 左 | 6,927 | 98.71% → 99.17% |
| max 右 | 6,954 | 98.62% → 99.22% |
| long 左 | 10,789 | 95.87% → 97.75% |
| long 右 | 10,730 | 95.81% → 97.55% |

各腕のセル幅3 cm、初期50,000姿勢・補完20,000姿勢・独立検査10,000姿勢。
一致率の分母は独立検査で自己衝突なし・範囲内となった姿勢数。空間体積の割合ではない。
maxの旧調査と同じ50,000手先点では、最寄り点が5 cm以内の割合が旧24.91% / 26.63%から
新99.928% / 99.956%へ改善。ただし旧クエリには衝突姿勢を含み、経路実行可能率ではない。
[XY比較図](../../artifacts/max_reachability_generation_20260928/coverage.png)。

| 管理場所 | 内容 |
| --- | --- |
| `src/core/reachability/joint_sampling.hpp` | URDF関節制限・低差異サンプル |
| `src/offline_tools/reachability_voxel_builder.cpp` | FCL判定・セル登録・補完・独立検査・保存 |
| `launch/reachability_voxel_builder.launch.py` | 片腕の生成設定 |
| `launch/dual_arm_reachability.launch.py` | 左右の独立配信 |
| `gng_results/<モデル>/reachability/left_arm.bin`・`right_arm.bin` | VIZGST2到達セルmap、代表関節角・空間隣接edge |
| 各`.bin.json` | 関節名・制限・原点・判定条件・補完前後の検査結果 |

到達域生成器で修正した問題：プロファイル名の誤接頭辞、未設定has_limitsフラグへの依存、
肩原点の全身衝突への誤適用、メッシュ非対応の簡易判定。
共通FCL側はメッシュ二重登録を解消し、strict判定と衝突ペア診断を一致。
疎なセル保存の上限は探索箱全体でなく登録セル数65,535へ適用。新旧互換分岐なし。

## 条件・検証

- 各腕のみ可動。他腕・腰・頭・グリッパーはゼロ姿勢。FCLの自己衝突判定を使用。
- 床・環境障害物・動的占有・手先姿勢の要求は対象外。現在の両腕姿勢での動作保証ではない。
- 登録は「セル内に少なくとも1つの証拠姿勢がある」という意味。セル全域・中心の厳密到達を保証しない。
- 未登録セルは未確認。到達不能の断定には使用しない。空間隣接edgeは無衝突の関節軌道ではない。
- 元GNG・VLUTは変更なし。計画・実行ノードの入力にも自動接続しない。
- 全代表姿勢のURDF制限・独立FKを検査。中心との最大距離はmax25.29 mm、long25.56 mmでセル半対角25.98 mm内。
- Release全体ビルド28パッケージ・CTest 3対象成功。FCL一対一登録・姿勢反映・URDF制限の回帰試験を追加。
- ROS_DOMAIN_ID=79でmax / longの左右配信・件数・edge・frameを検査。Viewer実画面の操作は未実施。
- 初期検証で衝突棄却ゼロ→全棄却→制限外サンプルのFK不一致を検出し、上記問題を修正して再生成。
- 初期のsmoke出力は検証失敗時の資料であり利用対象外。採用対象は`gng_results`内の最終4ファイル。
- 環境読込み時の未定義変数エラーを生成スクリプトで修正。SciPy/NumPy既存警告あり、最終解析は成功。
- 生成・解析・配信検査・ビルド・試験は終了。試験配信ノードはSIGINTで停止、既存ノードの停止なし。

Viewerで重ねる場合の例。召喚ロボットと同じ基準フレームを指定。

```bash
ros2 launch gng_vlut_system dual_arm_reachability.launch.py \
  model_dir:=/ros2_ws/src/gng_vlut_system/gng_results/topo_dual_arm_max/reachability \
  namespace:=sim_topo_dual_arm_max frame_id:=sim_topo_dual_arm_max/base_link
```

表示トピックは`/sim_topo_dual_arm_max/reachability_left_arm_Tmap`と`reachability_right_arm_Tmap`。
longはモデルディレクトリ・namespace・frameをlong側へ変更。
生成・検証の正本：[実行スクリプトと結果](../../artifacts/max_reachability_generation_20260928/README.md)。
