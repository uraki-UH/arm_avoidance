# 2026-09-24 - Viewerのローカルメッシュ直接表示

## 1. 要約

ToPoFuzzy-ViewerのTSX側に `Data → Mesh Models` を追加。
選択したモデルを点群化せず面付きで表示し、既存の点群・グラフとの比較が可能。

- OBJ・面付きPLY・STL・GLB/glTF・FBXの静止表示、付属MTL・画像・BINのローカル解決。
- 位置・回転・拡大率・単位、表示切替、全体を見る、明るさ補助、削除の操作。
- 入力原本・既存ロボット照明の変更なし。サーバーへのアップロード、ROS配信、WS拡張なし。
- 大容量OBJ用のオフライン軽量化ツールを追加。トラックの30万面・4K画像内包GLBを別フォルダへ生成。

## 2. 条件・検証

| 項目 | 内容 |
| --- | --- |
| 影響 | Frontendのローカル表示機能を追加。ページ再読込でモデル・姿勢設定を解除 |
| 条件 | 取得済みバイクOBJ・PLY、自転車GLB・FBX、ArtecバイクOBJ＋MTL＋PNGを専用headless ChromeのWebGLで描画 |
| 結果 | 上記5モデルの実描画、単位・位置・回転、非表示、削除後のGPU形状・画像解放を確認 |
| 異常系 | 欠損BIN、非有限座標、点だけのPLY、曖昧な参照、大容量ファイルの拒否を確認。欠損MTLは警告＋既定材質 |
| 回帰 | メッシュ・マーカー・候補ロボット・点群表示・再接続のNodeテスト7件成功。ESLint成功 |
| ビルド | コンテナ内FrontendのTypeScript＋Vite、本番Backendのtopo_fuzzy_viewerパッケージとも成功 |
| 制限 | 本体256 MiB・選択合計512 MiB。約1.26 GBのTruck.objは読込前に拒否。圧縮拡張・アニメーション・独自PLYテクスチャは対象外 |
| 軽量トラック | 1,500万→30万面、OBJ 1,261,446,760 bytes→GLB 44,814,004 bytes。画像8192→4096 px。原本OBJ・MTL・画像のSHA256一致 |
| 軽量化の品質 | 元表面の面積比例サンプル2万点から軽量表面への片方向距離。0.001倍換算で平均1.061 mm、rms 1.770 mm、サンプル内最大14.607 mm。全表面の最大誤差・登録精度は未保証 |
| 軽量化の検証 | Python回帰2件、GLB再読込、TSXで内包画像・30万面・座標範囲・100 m未満のカメラ位置・実描画35,049画素・削除時GPU解放を確認。追加後のlint・Frontend/Backendビルド成功 |

トラックは原本の軸・原点を保持し、座標のみ0.001倍。Viewerではm・回転0のまま使用。
倍率は座標の桁と車体寸法からの判断で、配布元単位メタデータの検出ではない。対応行列・帰属・変更内容は生成先のpreview.jsonへ保存。
変換は約314秒で終了。ホストのensurepip不足は、一時venvへ既存pipから依存を導入して回避。システムPythonの変更なし。

初回のFBX描画検証では暗い材質のため画素確認が失敗し、当該モデルだけに作用する明るさ補助を追加後に成功。
ホストのViteビルドはDocker所有の `.vite-temp` への書込権限で失敗し、所有権を変更せず既存コンテナ内でビルド成功。
Viteの大きなchunk警告とBackendの既存コンパイラ警告は残存。
検証は専用ブラウザでの実コンポーネント表示までで、利用者の開いているタブでの操作確認は未実施。

再現コマンド・操作・入力パスは [メッシュ表示ガイド](../../../ToPoFuzzy-Viewer/doc/MESH_MODELS.md) が正本。
追加回帰コマンドはFrontendディレクトリで次のとおり。

```bash
VEHICLE_MODELS=/home/uraki/datasets/vehicle_models node --test tests/local_mesh_loader.test.mjs tests/marker_array_renderer.test.mjs tests/robot_candidate_limit.test.mjs tests/point_cloud_display.test.mjs tests/stream_restart.test.mjs
```

Backendのビルド確認コマンド：

```bash
docker exec gng_cpu_container bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws/src/ToPoFuzzy-Viewer/backend && colcon build --packages-select topo_fuzzy_viewer --symlink-install'
```

試験で起動した専用Chromeは全終了し、専用一時プロファイルも削除済み。
既存Frontend・gateway・Dockerコンテナの停止や再起動なし。新規ROSノード・再生・常駐サーバーの起動なし。
