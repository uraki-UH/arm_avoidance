# 2026-09-09 - ViewerのGPUリセット診断とNVIDIA起動方法

## Summary

`/topological_map` 有効化後のWebGLコンテキスト喪失について、Chrome処理に伴うAMD GPUのpage faultとring resetを確認。NVIDIA描画経路による回避手段の追加。

## Changed

- `RUN_GUIDE.md` に専用Chromeの起動方法を追記。

## Added

- `scripts/open_viewer_nvidia.sh`: PRIME render offloadとANGLE EGL (`--use-angle=gl-egl`) によるChrome起動。`__EGL_VENDOR_LIBRARY_FILENAMES` でNVIDIA EGLドライバを明示指定。
- 既存Chromeへの起動転送を防ぐ専用プロファイル `$XDG_CACHE_HOME/topofuzzy-viewer-nvidia-egl`（未設定時は `~/.cache/topofuzzy-viewer-nvidia-egl`）。

## Fixed

- 通常ウィンドウにおける旧 `--use-angle=gl` の `Invalid visual ID requested` 初期化失敗を、`gl-egl` 経路へ変更。
- AMD GPU障害そのものの修正ではなく、代替描画経路。

## Removed

- なし。

## Behavior Impact

```bash
# ホスト側での実行。frontendとgatewayは通常の方法で起動済み。
bash scripts/open_viewer_nvidia.sh
```

URL指定時は第1引数を使用。NVIDIA GPUまたはGoogle Chromeがない場合はエラー終了。
旧プロファイルのブラウザではGPU初期化失敗後に `--use-gl=disabled` へ移行。旧ウィンドウの再読み込みでは新しい起動設定の適用不可。

## Topics / Params / Messages

- 変更なし。

## Verification

- `journalctl -k`: 2026-09-09 17:29:28 JSTに `amdgpu 0000:65:00.0: [gfxhub] page fault`、Chrome PID 3021205を記録。
- 同17:29:30に `ring gfx_0.0.0 timeout` と `Ring gfx_0.0.0 reset succeeded`。ブラウザ報告の `2026-09-09T08:29:30.953Z` と一致。
- 17:31:34にも別Chromeプロセスでpage fault、17:31:36にring reset。
- `node /tmp/viewer_gpu_probe.cjs` による一時プロファイル・headless Chrome検証。初回は検証用モジュールのimport形式エラー、修正後に下記の描画を確認。
- 実ROS地図から取得した3000ノード・8778エッジのスナップショットを、現行GraphRendererで40秒間・798回更新。ノード座標を微小変更し、ノード・エッジ・法線を表示。クラスタは検証用スナップショットでは空配列。
- renderer: `ANGLE (NVIDIA Corporation, NVIDIA GeForce RTX 5070 Laptop GPU/PCIe/SSE2, OpenGL 4.5.0 NVIDIA 580.159.03)`。
- コンテキスト喪失0回、Geometry数4のまま、最終描画798フレーム。検証中の新規GPU fault/reset記録なし。
- 404取得先は検証ブラウザでは `/favicon.ico`。Viewerコードの取得失敗なし。
- `bash -n scripts/open_viewer_nvidia.sh` 成功。
- 通常ウィンドウでは旧 `gl` 設定の初期化失敗を再現。headless検証結果だけでは通常ウィンドウの動作保証にならないことを確認。
- `env __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json PROBE_ANGLE=gl-egl node /tmp/viewer_gpu_probe.cjs` による通常ウィンドウ検証で、NVIDIA OpenGL ES 3.2選択とViewer初期描画を確認。初回の連続描画試験は途中のページ再読み込みにより中断。
- 通常ウィンドウの再試験では、その時点の実地図659ノード・1024エッジを40秒・765回更新。764描画フレーム、喪失0回、Geometry数4を確認。これは前述のheadless 3000ノード試験とは別条件。
- 通常ウィンドウ検証のChrome PID 3038852、3039934、3040458、3042247、3043738は停止済み。一時プロファイル削除済み。
- 検証ChromeのPID 3028767、3030026は停止済み。専用一時プロファイル削除済み、子プロセスの残存なし。既存ブラウザ・ROSノードの停止なし。

## Risk / Notes

- AMD側の不正アクセスを誘発する個別の描画命令・ドライバ内部経路は未特定。ノード数やDockerのSIGTERMだけを障害原因とする根拠なし。
- requestAnimationFrameはGPU完了通知ではないため、従来ACKの待機時間を「GPU実描画時間」とする説明は不正確。
- 40秒の検証は長時間安定性や全表示設定の保証ではない。
- 元のブラウザにおける404 URLは提示ログだけでは未確定。検証結果との区別。
