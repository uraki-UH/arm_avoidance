# 2026-08-03 - Viewer URDF initial load performance

## Summary

ToPoFuzzy Viewer が同一ロボットの候補姿勢を複数生成するときの、URDFメッシュ解析、外観反映、メッシュ配信、JSON転送の重複を削減した。

## Changed

- Viewer は同一URLのメッシュ読み込み結果を共有し、各候補にはObject3D階層とmaterialを複製する。
- メッシュ配信側はpackage解決結果とファイル内容をキャッシュし、更新時刻またはサイズが変わったファイルだけを再読込する。
- 候補プレビューのURDF本文はdescriptionのトップレベルにだけ格納する。
- メッシュHTTP応答を `Cache-Control: no-store` とし、ページリロード時にブラウザ内のSTL応答を破棄する。

## Fixed

- 候補数に比例して同じSTLを再解析していた初期表示遅延を修正した。
- 非同期メッシュ読込を待つために最大7回行っていたロボット全体の外観再走査を廃止し、到着したメッシュだけへ外観を適用するようにした。

## Removed

- `stream.robot.pose` と `robot.instances[]` から重複するURDF本文を削除した。

## Behavior Impact

- ロボット形状、候補姿勢、URDF色、候補別の透過度は変更しない。
- 候補8件ではdescription/pose内のURDF本文が合計18個から1個になり、URDF本文に由来する転送量は約94.4%減る。
- 一度読み込んだメッシュgeometryはViewerページを閉じるまで共有キャッシュに保持される。
- ページリロード後はSTLを再取得・再解析する。backend内部ではファイル内容をキャッシュするため、ファイル未更新時のディスク再読込は行わない。

## Topics / Params / Messages

- topic名、ROS message型、launch引数、parameterの変更はない。
- `stream.robot.description` は `robot.urdf` を保持する。
- `stream.robot.pose` と `robot.instances[]` はトップレベルdescriptionのURDFを参照し、`urdf` fieldを持たない。

## Verification

- `npm run lint`: 変更ファイルのエラーなし。既存の `SharedControls.tsx` 2件、`ellipsoid.tsx` 1件により全体は失敗。
- `npm run build`: Dockerのfrontendコンテナ内で成功。
- `colcon build --packages-select topo_fuzzy_viewer gng_vlut_system --symlink-install`: Dockerのgng_cpuコンテナ内で成功。
- headless Chrome: 1440x900と390x844で接続中のViewerを開き、Three.jsキャンバスにグリッドとロボット形状が描画されることを確認。

## Risk / Notes

- ページリロードごとにSTLのHTTP転送とFrontendでの解析が再実行される。
- backendビルドには今回と無関係な既存warningが3件残っている。
