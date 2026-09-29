# 2026-09-16 - 微小関節変化による局所並進・回転のFK可視化

保存済みGNGの1ノード周辺をFKで評価し、並進グラフと回転点群を表示する独立Python試験。当日の最終版（2分類）の記録で、ROS・シミュレーション・refineへの統合なし。

<a id="並進成分と回転成分による2分類への修正"></a>

## 結果と条件

ToPoDualArm10000（10,801ノード）のactiveなノード0を使用。各関節±2度・0.5度刻み・関節差L2上限4度の4,169,977通りを直接FK評価し、全件が関節限界内。IK・ヤコビアン補正・ランダム抽出・限界値への切り詰めなし。

| 分類 | 許容条件 | 点数（原点を含む） | 表示 | サンプル内の最大変化 |
| --- | --- | ---: | --- | --- |
| 並進的 | 相対回転角0.25度以内 | 2,102 | 9,976辺、連結成分1 | 並進9.645 mm |
| 回転的 | 並進量0.5 mm以内 | 566 | 点群のみ | 相対回転角5.531度 |

重複は4件。両条件を超える混合4,167,313件は件数のみ記録し、全保存・描画は未実施。旧3分類の軸平行判定は、その軸まわりの回転を区別できないため削除。

- 座標：元姿勢のFKを基準に並進 `R0ᵀ(p-p0)` [mm]、回転 `R0ᵀR=Rz(yaw)Ry(pitch)Rx(roll)` の相対RPY [deg]。並進量と相対回転角は座標基底によらず、並進点と回転点の独立合成を保証するものではない。
- 辺：表示点の関節空間8近傍から、関節差L2 1.5度・位置差5 mmで選択し、内部7点のFKで姿勢条件を確認。表示上限3,000点を超えて間引く場合も、採用数・最大値・CSV/NPZは全採用サンプルで集計。サンプルIDで対応付け。
- 時間：FK・残差計算10.650秒、画像生成前までの解析12.320秒。この環境での単発の壁時計実測で、一般的な処理性能ではない。

## 検証・制限

- 旧結果の「向き全体維持」「位置維持」と全サンプルID・数値が一致。解析解、逐次FK、軸まわりの回転の計上、座標基底不変性、格子順序・関節限界を検査。
- 辺の17点再補間で最大回転量0.249967度。Plotlyの2分類・全採用点数・回転側の辺なし・視点操作・実描画を確認。
- 未検証：自己衝突・環境衝突・速度・加速度・未探索領域・大域的到達性。有限補間は連続区間全体の保証ではなく、元ノードの自己衝突なしフラグも周辺へ継承しない。
- 読込対象：version 9・little-endian・64 bit Eigen::Index・float配列の一座標層GNGと独立回転関節鎖。汎用インポーターではない。XYZ/RPY投影で座標が重なっても元サンプルは保持。
- 形式：`format_version=2`、CSV分類列は `is_translation_like` / `is_rotation_like`。旧3分類の結果・スクリプトは `tmp/local_fk_motion_20260916/before_motion_separation/` に退避。
- 当日のPython・検証Chromeは終了、残存なし、専用profile削除済み。入力GNG・URDF・通常ブラウザー設定の変更や、既存プロセスの停止なし。

<a id="plotlyによる3dグラフ表示の追加"></a>

## 表示・再現

[計算](../../scripts/inspect_local_fk_motion.py)と[表示](../../scripts/view_local_fk_motion.py)を分離。既定入力は `gng_results/ToPoDualArm10000/gng.bin`（パッケージ内）と `dual_arm_urdf/dual_arm_robot.urdf`、リンクは `L_shoulder_mount` → `L_tcp`。引数の詳細は各スクリプトの `--help`。

`index.html` は単独表示、`plotly.html` は保存済みgraph/CSVから再計算なしで生成。Plotly 4.0.0本体とデータを埋め込み、表示時はサーバー・ネット接続不要（WebGL必須）。視点・投影・2分類・全採用点の切替、L2スライダーと約16秒の順次追加、残差・関節差・ID表示、PNG保存に対応。全点表示でも辺の再構築なし。

<details>
<summary>当時の生成・検証コマンド（ワークスペース直下）</summary>

NumPy・SciPy・Matplotlibを使用。既存結果を残す再実行では出力先を変更。初回生成時のみPlotly本体の取得が必要。

```bash
timeout -s INT -k 5s 120s python3 gng_vlut_system/scripts/inspect_local_fk_motion.py --output-dir tmp/local_fk_motion_20260916
curl -fSL --connect-timeout 10 --max-time 40 https://cdn.plot.ly/plotly-4.0.0.min.js -o tmp/local_fk_motion_20260916/plotly-4.0.0.min.js
python3 gng_vlut_system/scripts/view_local_fk_motion.py tmp/local_fk_motion_20260916
timeout -s INT -k 5s 40s python3 tmp/local_fk_motion_20260916/verify.py
node --check tmp/local_fk_motion_20260916/index_motion_viewer.js
node --check tmp/local_fk_motion_20260916/plotly_motion_viewer.js
timeout -s INT -k 5s 50s google-chrome --headless --no-sandbox \
  --no-first-run --no-default-browser-check --disable-background-networking \
  --use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader \
  --user-data-dir=/tmp/local_fk_motion_components_chrome_20260916 \
  --window-size=1500,1100 --hide-scrollbars --virtual-time-budget=15000 \
  --screenshot=/home/uraki/uraki_ws/tmp/local_fk_motion_20260916/motion_plotly_browser.png \
  --dump-dom file:///home/uraki/uraki_ws/tmp/local_fk_motion_20260916/plotly.html#selftest
```

`verify.py` は当時の結果ディレクトリに保存した検証補助。Chromeはsandbox内で起動失敗後、承認された外側実行で確認。ソフトウェア描画・sandbox無効化は当時の検証用で、通常利用への推奨設定ではない。

</details>

成果物：`tmp/local_fk_motion_20260916/` の `samples.csv/npz`・`graphs.json`・`overview.png`。入力SHA-256・計算条件は `summary.json`、数値検証は `motion_separation_verification.json`、描画確認は `motion_plotly_browser.png` / `motion_plotly_browser_dom.html`。一時領域の記録のため、主要結果は上表にも保持。
