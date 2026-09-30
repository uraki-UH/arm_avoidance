# 2026-10-01 - 双腕リンク変換の座標整合

- 対象: 肩リンクを各腕のrootに指定した`MultiArmKinematicAdapter`。
- 修正: URDF rootへのglobal base適用、各腕の実FKの先行登録、不足枝だけのゼロ姿勢補完。factoryのglobal base二重加算も解消。
- 原因: 各片腕から全URDF枝を補完して後勝ちで結合する処理。肩座標の全身への誤適用と、他腕の実姿勢の上書き。
- 再現: 保存到達姿勢node 10000で関節限界・TCP一致、衝突用base_link位置のみ右肩へ移動。R_link1/3/4との偽衝突。
- 検証: 既存2件＋新規5件の回帰試験成功。左右非ゼロ姿勢、global base平行移動・回転、外部FK入力、prefix別rootの照合。
- 反映: 通常package build/install成功。関連CTest 3/3対象成功。実検証結果・全起動コマンドは[補完検証記録](../../../benchmarks/gng_coverage_repair_20261001/README.md)へ集約。

衝突検査の全joint原点表と、VLUT生成の固定joint原点表では旧挙動の影響範囲が異なるため、旧VLUT全体の誤配置という判断は保留。生成モデルのVLUTは独立XML FKとの位置照合が対象。
