# 車両照合の検証

## 1. 要約

- [実装・操作・指標](../../ToPoFuzzy-Viewer/doc/VEHICLE_REGISTRATION.md)。合成欠損セダンの姿勢復元、トラック、平面の判定保留、外れ値、入力検証の5ケースが成功。
- 稼働中Macnica交差点bagの `/topological_map` から保存した33ノードのクラスタを照合。一致100%、モデル支持13.6%、未対応86.4%、RMS 0.081 mで判定保留。車両・車種の正解ラベルは未確認。
- 同モデルから作った316ノードの欠損セダンは一致100%、支持41.1%、RMS 0.0247 m。約40.1度の既知姿勢とセダン候補を復元。これは位置合わせ検証であり、未知車種の認識精度ではない。
- Releaseビルド・CTest（内部5ケース）、Frontend lint/build、既存選択UI・路面枠除外の計7テスト成功。実WS＋Chromeで4候補・描画・候補切替・選択変更中の古い結果破棄・実観測の判定保留を確認。
- 路面クラスタ#0の約76 × 101 × 19 mの枠が物体より先に選択される問題を確認。当初は路面・壁のみを除外。その後のユーザー指示で`/topological_map`のBBox表示・選択機能ごと撤去。点群・グラフ自体を維持。
- ログ、画像、全観測の一時保存、PID記録はGit対象外。再現用の小さな[クラスタfixture](../../ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/fixtures/vehicle_intersection_cluster.json)だけ保存。

## 2. 条件・検証

一致距離0.25 m、支持距離0.35 m、各モデル3500点。CPU処理、Yaw＋XYZ、寸法固定。
4モデル比較の実クラスタ約0.8秒、合成観測約3.1〜3.9秒（同時稼働プロセスあり、通信・描画は時間外）。
ホストのVite一時ディレクトリ権限のためFrontendビルドは稼働中`frontend`コンテナで実施。
初回ブラウザ試験は接続開始処理の不足で失敗し、`connect()`追加後に成功。
最終Frontendビルドは別操作のコンテナ再起動と一度競合。復帰後の再実行で成功。

```bash
# ホスト、リポジトリルート
python3 benchmarks/vehicle_registration_20260924/evaluate.py
python3 ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/test_vehicle_registration.py

# ホスト、試験ノードだけを最大180秒起動。既存gatewayが必要
# 終了時はSIGINT、必要に応じてSIGTERM/SIGKILL、waitによる回収
docker exec gng_cpu_container bash -lc 'source /ros2_ws/install/setup.bash; python3 /ros2_ws/src/benchmarks/vehicle_registration_20260924/run_rpc_server_test.py'

# 上の有限時間内に、別端末のfrontendディレクトリから実行
cd ToPoFuzzy-Viewer/frontend
timeout --signal=INT --kill-after=5 100 node tests/vehicle_registration_browser.test.mjs
```

ブラウザ試験内のChromeは`--headless=new --remote-debugging-pipe --user-data-dir=<一時ディレクトリ>`付きで起動し、finallyでプロセス群と専用ディレクトリを回収。
検証スクリプトが起動した追加ノード（PID 152823・152987・158951）とChromeは停止済み。既存bag・GNG・Viewerに停止・再起動の操作なし。
作業中に別操作で再起動された通常Viewer（親PID 159101）配下の照合ノード（PID 159114）は稼働を維持。
初回観測取得は`docker exec gng_cpu_container`内で`timeout --signal=INT --kill-after=3 20 python3`を実行し、`vehicle_registration_capture_20260924`で6フレーム購読後に終了。

通常運用で追加ノードが必要な場合は[起動手順](../../ToPoFuzzy-Viewer/doc/VEHICLE_REGISTRATION.md)を使用。

保存画像（ローカル生成物）: [合成欠損セダン](synthetic_registration.png)、[交差点クラスタ](intersection_registration.png)。
