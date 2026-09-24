# 2026-09-24 - CPU GNGの全体走査削減

## Summary

探索用配列の同期再利用、孤立ノード候補管理、実在エッジ用プールの単一走査を本番CPUへ反映。保守区間が28〜31%短縮。全体はvoxel 0.5 mで33.315→32.807 ms、0.1 mでは93.338→94.004 msで改善を確認できず。[条件・全測定・起動コマンド](../../../benchmarks/gng_incremental_updates_20260924/README.md)。

## Changed

- 入力照合・学習・距離更新で探索用座標配列を再利用し、通常フレームの全ノード同期を2回から1回へ削減。
- 孤立候補をID順に処理。`eta_decay_rate < 1` の学習係数減衰は従来順序を維持。
- 実在エッジの端点IDを保存し、距離更新で両端の隣接配列の重複走査を除去。

## Added

- 固定dense参照との更新順・寿命・距離・イベント照合テスト。
- [独立比較・再現資料](../../../benchmarks/gng_incremental_updates_20260924/README.md)。

## Fixed

通常フレームの重複同期と、孤立ノード確認・距離更新の余分な全体走査。

## Removed

公開機能の削除なし。移動ノード追跡による距離差分更新の試作は本番不採用。

## Behavior Impact

固定条件で5,450組の全出力一致。入力フィルタ、学習4,000回、ノード寿命・削除順、法線・曲率・ラベル更新を維持。単独CUGNG呼出しは従来経路。内部C++クラス配置の変更に対し、利用パッケージを再ビルド。

## Topics / Params / Messages

変更なし。公開C APIの22シンボルも不変。

## Verification

- 比較用7,630フレーム、うち拡張機能の照合450組。採用版の通常乱数・実時間版30フレームも成功。
- 追加回帰3,696比較、64ビット境界・ID再利用・寿命巻戻り・減衰0.99／1.0を確認。ASan／UBSan成功。
- 本番ReleaseのCTest21件＋追加API2件、WASM native 1件成功。`cb`は30パッケージ、58.4秒で成功。
- `bash benchmarks/gng_incremental_updates_20260924/build.sh before combined sync orphan edges`、`build.sh pooled`、`run_suite.sh before sync orphan edges combined`、`run_suite.sh before_pool pooled`、`verify_boundaries.sh`、`validate_production.sh`を実行。全コマンドの引数・作業ディレクトリは上記再現資料に記載。全検証プロセス終了済み。
- 既存GNG・Viewer・bag再生の12プロセスは前後一致。原子的な配布で稼働中GNGの旧ライブラリ参照を維持。新実装は次回GNG起動時から有効。

## Risk / Notes

保守区間の短縮を全体の改善率として扱わない。0.1 mの全体短縮は未確認。比較元と採用版を交互測定したが、既存GNGを稼働させた状態のため負荷変動を含む。単一bag・固定乱数列の検証であり、別環境の結果を保証しない。追加メモリは候補ビット列とエッジ端点ID。ノード全ペア分の配列は使用しない。
