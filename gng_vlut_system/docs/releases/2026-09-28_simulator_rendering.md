# 2026-09-28 - 双腕ブラウザシミュレータの描画負荷

## 要約

- 視点の慣性補間（dampingFactor 0.09）を無効化。FPSとは別の入力追従遅れへの対処。

- 標準品質ではEffectComposerを省略して直接描画。preserveDrawingBufferを無効化し、PNG保存は既存の保存直前描画を利用。
- 初期品質を標準へ変更。SSAO無効、ピクセル比1。高精細はメニューから選択可能。
- RGB-Dの非表示点群の色計算を省略。連続取得では取得・表示処理と同じ長さの休止を確保し、描画と入力操作へ時間を配分。
- GPU読み出しを非同期化。描画状態と除外物体の表示は待機前に復元し、同一姿勢の全描画を発行。二重取得とモデル・校正変更前の結果反映を防止。
- 深度用GPUバッファをRGBA float32からR float32へ変更。非対応GPUではRGBAへ自動復帰。出力解像度・精度を維持。
- UI captureはPromiseへ変更、VM・AIとモデルQAの呼び出し元をawait対応。低水準captureは既定同期の互換性を維持。
- 解像度・1フレームの点数・校正は維持。指定Hzは上限で、負荷に応じて取得頻度を低減。
- 読み込み済み画面は再読み込みで反映。VM・AI送信側の頻度制御は今回の対象外。
- URL不要の `scripts/open_dual_arm_nvidia.sh` を追加。専用プロファイル・X11・1440×1000で起動。汎用ランチャーはMarkdownリンクをURLに補正。
- 手順：[README](../../../ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/README.md)。

## 条件・検証

- NVIDIA RTX 4060、Chrome ANGLE / OpenGL ES 3.2、X11、1440×1000の独立ウィンドウ。
- 標準モデル、初期シーン、標準品質。深度848×480、ステレオ可視性、指定10 Hz、点群表示OFF。
- 最新比較は非同期RGBA→非同期R形式。初期化とウォームアップを除外し、順序を交互にした各5回の平均取得時間121.86→70.08 ms（約42.5%短縮）。
- 上記解像度のステレオ取得では、GPU読み出しの生バッファ計算量31,457,280→10,629,120 byte/フレーム。深度3枚とRGB1枚の合計で約66.2%削減。
- 各5秒の連続取得比較：41.74→47.80 fps、描画間隔最大100.1→83.4 ms、95パーセンタイル50.1→33.4 ms。
- 終了時の取得頻度3.75→5.44 Hz。既存ブラウザも起動中で、連続取得比較は各1回。
- 単一成分形式と旧RGBA形式の7配列全要素一致を確認。RGBAへの強制フォールバック経路でも一致。
- 同一静止姿勢でdepth、z16、rgba、xyz、colors、colorValid、pixelsの全要素一致。有効点124,002。非同期取得中のモデルリセット後に古い結果を破棄する検査も成功。
- 同期GPU完了待ちは除去したが、画素計算・プレビュー描画・GPU負荷は残存。操作の引っかかりの完全解消は未達。ROS実接続と動的場面・Longの非同期実測は未検証。
- RGB-D停止時は約60 fps。高精細も静止場面では約59.5 fpsで、初期品質変更単体の効果は未確認。
- GPU renderer文字列とブラウザ内エラーなしを確認。JavaScript構文検査成功。
- GBM警告はNVIDIA描画成功時にも発生。GBM_BACKEND指定でも警告は残り、改善設定としては不採用。
- `--use-angle=gl` はGPU初期化失敗のため不採用。既存の `gl-egl` を維持。サンドボックス無効化・権限変更なし。
- 前回はヘッドレス初期化・通常画面の応答待ちに失敗。今回X11明示の通常画面で上記測定に成功。
- 元MANIFESTを維持。app.js、index.html、rgbd-ui.js、rgbd-core.js、vm-ai.js、qa-models.js、README.mdは意図した差分。
- 試験起動：`node /tmp/topo_gpu_probe_gbm.mjs`、`node /tmp/topo_gpu_probe_gl.mjs`、`node /tmp/topo_gpu_probe_rgbd.mjs`。
- 非同期比較起動：`node /tmp/topo_rgbd_async_test.mjs`。
- 転送量比較：`node /tmp/topo_rgbd_bandwidth_test.mjs`。対応形式確認：`node /tmp/topo_rgbd_format_test.mjs`。
- 試験用Chromeは停止。既存サーバー・ブラウザの停止操作なし。

- ランチャー構文と引数の5ケース検査成功。その後、専用ランチャー・実際の専用プロファイルでChromeを起動し、CDPから100回の視点ドラッグを実行。
- 最新の実ランチャー検査：センサー停止59.49 fps（描画間隔95%値16.8 ms、最大33.4 ms）、RGB-D取得中54.51 fps（95%値33.4 ms、最大50.1 ms）。RGB-D最終取得頻度3.74 Hz。
- ウィンドウ1440×1000指定、実ページ1396×869、描画領域1031×649、標準品質。URL・LOCAL READY・RTX 4060 renderer・視点座標の変化・ページ内エラー0件・画面画像を確認。
- GBM Permission denied、XNNPACK、GCM DEPRECATED_ENDPOINTを記録。警告があっても上記描画は成功。ユーザー提示のQUOTA_EXCEEDEDは今回再現なし。最大化・別シーンでの滑らかさまで保証する結果ではない。
- 再検査時、診断接続が前回のポート番号を読み一度失敗。検査側の起動待ちを修正して成功。アプリの起動失敗とは区別。
- 実行：`node /tmp/topo_actual_launcher_test.mjs`。起動元は `bash scripts/open_dual_arm_nvidia.sh standard --remote-debugging-port=0`。デバッグ引数の転送に対応。
- [ログ・画面・検査スクリプト](../../../artifacts/topo_launcher_check_20260928/)。試験用Chrome停止、既存サーバー維持。

- 実際に使用中のウィンドウを前面化して60 fps表示を確認。FPSだけでは入力遅延を説明できないため慣性設定を修正。構文検査成功。ユーザーが感じるカクつきとの因果と体感改善は未確認。既存ウィンドウは停止・再読み込みなし。

- 標準描画経路比較（各1回）：停止中58.63→59.18 fps、最大描画間隔83.4→33.3 ms。RGB-D中53.51→56.12 fps、最大50.1→33.4 ms。開始視点が異なり、効果の定量断定には不十分。
- 同試験のRGB-D最終取得頻度3.99→2.60 Hz。取得速度の改善を示す結果ではない。ユーザーが感じる遅れの解消は未確認。
- PNGボタンの出力1031×649、194,616 byte、非空画素を確認。試験保存はHTTP手前で捕捉しexportsへ書き込みなし。
- 実行：`node /tmp/topo_render_check.mjs`、`node /tmp/topo_render_png_check.mjs`。[検査ログ](../../../artifacts/topo_render_check_20260928/)。試験Chrome停止。ユーザー操作用Chromeは維持。
