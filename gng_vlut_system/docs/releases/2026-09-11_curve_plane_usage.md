# 2026-09-11 - 曲面採用に必要な元平面使用率

## Summary / Changed

曲面が使用する現在の元平面ノード数を、元平面の有効所属ノード数で割った使用率の検査。
いずれか一つの元平面で50%以上の使用が既定条件。全元平面から少量ずつ集めただけの曲面は未確定へ変更。

同じ元平面由来の分割パッチは合算。分母は現在フレームの有限座標・有効添字・重複除去後の所属であり、テンプレート全体や過去フレームの点数ではない値。
判定箇所は支持領域分割後の共通採用処理。新規候補・追跡候補・分割された子領域のすべてが対象。棄却時もノード被覆は保持し、曲面モデルと追跡資格のみを解除。

## Topics / Params / Messages

`ais_gng/config/surface_model.yaml`:

```yaml
surface_model.min_plane_usage_ratio: 0.5
```

有効範囲は0〜1、0は検査なし。新規launch引数・topic・message fieldの追加なし。
本体・設定・パラメータ読込への追加は計29行。既存の採用処理への追加であり、専用ノード・別検査モジュールの新設なし。

## Behavior Impact

- 少なくとも一つの元平面に十分な使用率があれば、他の平面の根元を部分使用する候補は許容。
- 元平面を全く含まない曲面も既定では不採用。従来の非平面のみの抽出は使用率を0へ設定して利用可能。表示の元平面数条件は別条件。
- 未確定へ戻した領域のノードはそのフレームでは保持され、別候補への再探索は次フレーム以降。
- 既存曲面処理ノードへの反映には再起動が必要。既存GNG・Viewerの停止・再起動操作なし。

## Verification

`gng_cpu_container`内での有限実行:

```bash
source /ros2_ws/install/setup.bash
timeout --signal=INT --kill-after=5s 180s cmake --build /ros2_ws/build/ais_gng \
  --target test_surface_model plane_cluster_incremental_node replay_surface_models benchmark_surface_merge -j2
timeout --signal=INT --kill-after=5s 60s ctest --test-dir /ros2_ws/build/ais_gng \
  -R '^test_(surface_model|nonplane_component_extractor|plane_cluster_incremental)$' --output-on-failure
timeout --signal=INT --kill-after=10s 30s python3 \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/check_surface_models.py --seconds 8 --graph
```

曲面63件、平面・非平面抽出を含む3テスト実行単位が成功。50%ちょうどの許容、一つだけ核がある場合の許容、全元平面の使用率低下による追跡解除、分割後の各1/6使用の棄却、同一元平面の分割パッチの合算、0による無効化、不正値拒否、根元分離の回帰を確認。

ROS通信検証は専用出力`/surface_model_check_8771`で15フレーム。モデル・TopologicalMap・Marker受信、所属の重複なし、グラフの所属・色整合の検査が成功。最後の観測は171ノード、表示候補2領域。内部処理の平均0.652 ms・p95 0.806 msは今回の観測だけの測定値で、速度上限の保証ではない結果。
検証ノードPID 8797はSIGINTで停止済み、終了コード0。観測用Pythonとビルド・テストも終了済み。

## Risk / Notes

50%は初期設定値であり、実物の正解ラベルに対する最適化済み値ではない設定。複数物体の表面が一つの巨大な元平面に統合された場合、正しい曲面候補でも使用率不足になる可能性。今回の実行検証は通信・整合性の確認であり、Viewer画面の見た目や物体別正解率の検証ではない範囲。
