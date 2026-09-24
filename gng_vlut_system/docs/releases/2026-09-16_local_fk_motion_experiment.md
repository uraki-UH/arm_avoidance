# 2026-09-16 - 微小関節変化による局所並進・回転のFK可視化

現在の分類は[並進成分と回転成分による2分類](#並進成分と回転成分による2分類への修正)。以下の3分類と軸平行の記述は初回試験の履歴。

## 1. 要約

保存済みGNGの一ノードを中心に微小な関節角差の組み合わせをFKで評価し、並進空間の3次元グラフと相対回転の点群を表示する、独立したPython試験を追加。ユーザー指定に従い、シミュレーション・ROS・refineへの統合なし。

既存処理・設定・モデルへの変更なし。

- [inspect_local_fk_motion.py](../../scripts/inspect_local_fk_motion.py)：読み込み・FK・分類・辺構築・HTML/画像出力を一ファイルに集約。
- `index.html`：3パネルの回転・拡大可能な3D表示。元ノードからの関節角差L2の上限スライダーと追加アニメーション、点の残差・各関節差の表示。
- `overview.png`、`samples.csv`、`samples.npz`、`graphs.json`、`summary.json`：比較図・再解析用数値・モデル由来と計算条件。

試作中に、HTMLのスライダー刻みによって追加アニメーションが停滞する問題を修正。連続値の進行量と表示スライダー値を分離。

## 2. 条件・検証

実行例（ワークスペース直下、NumPy・SciPy・Matplotlib使用）：

```bash
python3 gng_vlut_system/scripts/inspect_local_fk_motion.py \
  --node-id 0 \
  --output-dir tmp/local_fk_motion_20260916
```

`index.html` は外部JavaScript・サーバー不要。ドラッグで視点回転、ホイールで拡大、スライダーで小さい関節差から点を追加。「順に追加」で約16秒の再生。初期表示は全範囲。

FK評価は `q=q0+Δq` の直接計算のみ。IK・ヤコビアン補正・ランダムサンプリングなし。各関節の差を格子上で列挙し、`||Δq||2` の小さい順に評価。関節限界外は除外し、限界値への切り詰めなし。

基準位置・向きは元の関節角からFKで再計算。並進は `R0ᵀ(p-p0)` のmm表現、回転は `R0ᵀR=Rz(yaw)Ry(pitch)Rx(roll)` の相対RPY [deg]。軸の平行判定はGNGの `ee_direction` と同じ、元TCPの+Xと現在TCPの+Xの同方向性を使用し、反平行を含まない。向き全体の判定は相対回転角を使用。

ROSトピック・パラメータ・メッセージ・launchの追加なし。

| CLI引数 | 既定値・意味 |
| --- | --- |
| `--gng-file` | `gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin` |
| `--urdf` | `dual_arm_urdf/dual_arm_robot.urdf` |
| `--node-id` | `0`。元GNGのID |
| `--root-link` / `--tcp-link` | `L_shoulder_mount` / `L_tcp` |
| `--joint-step-deg` | `0.5`。各関節の格子刻み [deg] |
| `--max-joint-delta-deg` | `2.0`。各関節の正負の差 [deg] |
| `--max-joint-norm-deg` | `4.0`。関節差ベクトルのL2範囲 [deg] |
| `--max-orientation-dev-deg` | `0.25`。軸・向き全体の残差 [deg] |
| `--max-position-dev-mm` | `0.5`。回転候補の位置残差 [mm] |
| `--max-graph-joint-step-deg` | `1.5`。辺の関節差L2範囲 [deg] |
| `--max-graph-dist-mm` | `5.0`。辺の手先位置差 [mm] |
| `--max-display-points` | `3000`。各パネルの表示点数 |
| `--output-dir` | 出力ディレクトリ。必須 |

分類は重複可能で、いずれも原点の1ノードを含む。

1. 軸平行：進入軸の角度が条件内。軸まわりの回転を許容。
2. 向き全体を維持：相対回転角が条件内。
3. 手先位置を維持：位置ずれが条件内。相対RPY空間に配置。

並進グラフの辺は表示対象点の関節空間8近傍から選択し、関節差・位置差を検査後、関節線形補間の内部7点をFK評価して姿勢条件を検査。回転側は点群のみ。表示上限を超える場合は角度差順の配列から等間隔に間引くが、採用数・最大変位・CSV/NPZは全採用サンプルで集計。サンプルIDで表示と全データを対応付け。

ToPoDualArm10000の元GNGは10,801ノード。activeなノード0の保存関節角を使用。保存時の自己衝突なしフラグは参照情報として記録し、周辺サンプルへ継承なし。

各関節±2度、0.5度刻み、L2差4度の範囲で4,169,977通りをFK評価。全件が関節限界内。FKと残差計算は10.911秒、列挙・数値出力・グラフ構築を含む処理は13.508秒。後者は最終HTML/画像生成を除く、この環境での一回の壁時計実測値。

| 分類 | 採用数（原点を含む） | 表示点数 | 辺数 | サンプル内の最大変化 |
| --- | ---: | ---: | ---: | --- |
| 手先+X軸が平行 | 30,175 | 3,000 | 11,122 | 並進30.533 mm |
| 向き全体を維持 | 2,102 | 2,102 | 9,976 | 並進9.645 mm |
| 手先位置を維持 | 566 | 566 | 0 | 相対回転角5.531度 |

このノードでは向き全体を維持する並進分布が薄い面状、回転分布は相対yaw方向に長い形状。向き全体維持の並進グラフは連結成分1で、元ノードから全2,102表示点へ接続あり。軸平行グラフは元ノードから2,999表示点へ接続、1表示点は孤立。間引いた表示点に対する有限近傍の辺構築であり、孤立は運動空間そのものの非連結性の証拠ではない。

解析解のある2関節モデル、実URDFの逐次4×4変換とバッチFK、元GNGの手先方向とFKの+X軸の一致、相対RPYからの回転行列復元、格子の一意性と並び順、関節限界、グラフ添字を確認。辺を構築時より細かい17点へ補間して再評価し、最大角度誤差は軸平行0.249962度、向き全体0.249967度。解析・再検査用スクリプトは結果ディレクトリの `verify.py`。

```bash
timeout -s INT -k 5s 180s python3 gng_vlut_system/scripts/inspect_local_fk_motion.py --output-dir tmp/local_fk_motion_20260916
timeout -s INT -k 5s 40s python3 tmp/local_fk_motion_20260916/verify.py
node --check tmp/local_fk_motion_20260916/viewer.js
timeout -s INT -k 5s 35s google-chrome --headless --disable-gpu --no-sandbox --no-first-run --no-default-browser-check --disable-background-networking --user-data-dir=/tmp/local_fk_motion_chrome_20260916 --window-size=1500,1050 --hide-scrollbars --virtual-time-budget=2500 --screenshot=/home/uraki/uraki_ws/tmp/local_fk_motion_20260916/browser.png --dump-dom file:///home/uraki/uraki_ws/tmp/local_fk_motion_20260916/index.html#selftest
```

Chromeはサンドボックス内で起動失敗後、承認された外側実行で確認。`#selftest` でゼロ角度差の原点のみ表示、全件復帰、追加アニメーション進行、辺表示切替・視点復帰を検証。描画画像を目視確認。Python・Chromeは全終了、専用ブラウザprofileを削除。ROS・シミュレーション・サーバーの新規起動なし、既存プロセスの停止・再起動操作なし。

成果物はワークスペースの `tmp/local_fk_motion_20260916/` に保存。元GNGとURDFのSHA256は `summary.json`、数値検証は `verification.json`、描画確認は `browser.png` と `browser_dom.html`。

**制約**

- 一ノード周辺の有限格子のFK試験。未探索領域・関節差の刻みより細かい解・大域的な到達限界は未評価。
- 軸平行は完全な姿勢維持と区別。完全一致ではなく、明示した角度・位置残差の許容を使用。
- 辺の有限点による姿勢条件検査は、連続区間全体の数学的保証とは区別。自己衝突・環境衝突・速度・加速度は未評価。
- version 9・little-endian・64 bit Eigen::Index・float配列の一座標層GNG、独立した回転関節鎖を対象とする試験用読み込み。他形式の汎用インポーターではない。
- 点群はXYZとRPYへ投影した表現。同一座標へ異なる関節角が対応する場合も、元サンプルを保持。

<a id="plotlyによる3dグラフ表示の追加"></a>

**Plotlyによる3Dグラフ表示の追加**

ユーザーからの3次元グラフ描画ツールでの確認希望に対応し、[view_local_fk_motion.py](../../scripts/view_local_fk_motion.py)を追加。既存の `graphs.json` と `samples.csv` から `plotly.html` を生成し、FKの再計算なし。計算処理と独立して保存結果へ繰り返し適用するための表示専用スクリプト。

[Plotly公式配布](https://plotly.com/javascript/getting-started/)の4.0.0を結果ディレクトリへ取得し、[Scatter3d](https://plotly.com/javascript/3d-scatter-plots/)で描画。ライブラリ本体と数値を一HTMLへ埋め込み、表示時のネット接続・サーバー・追加Pythonライブラリ不要。ブラウザーのWebGLが必要。

- 3種類の結果を大きい一画面で切り替え。既定は3,000表示点の軸平行グラフ。
- 回転・平行移動・拡大縮小、正投影／透視投影、XY／XZ／YZ／斜め視点、PNG保存。
- 関節角差L2のスライダー・順次追加、点の関節角差・姿勢残差・位置残差・元サンプルID表示。
- 全採用点の切り替え。各30,175／2,102／566点をCSVから復元。辺は元の表示用グラフを保持し、全点表示に伴う辺の再構築なし。

生成・表示コマンド：

```bash
curl -fSL --connect-timeout 10 --max-time 40 https://cdn.plot.ly/plotly-4.0.0.min.js -o tmp/local_fk_motion_20260916/plotly-4.0.0.min.js
python3 gng_vlut_system/scripts/view_local_fk_motion.py tmp/local_fk_motion_20260916
xdg-open /home/uraki/uraki_ws/tmp/local_fk_motion_20260916/plotly.html
```

`xdg-open` は利用者向けの表示例で、作業中の実行なし。描画検証には以下を実行：

```bash
node --check tmp/local_fk_motion_20260916/plotly_viewer.js
timeout -s INT -k 5s 50s google-chrome --headless --no-sandbox --no-first-run --no-default-browser-check --disable-background-networking --use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader --user-data-dir=/tmp/local_fk_plotly_chrome_20260916 --window-size=1500,1100 --hide-scrollbars --virtual-time-budget=18000 --screenshot=/home/uraki/uraki_ws/tmp/local_fk_motion_20260916/plotly_browser.png --dump-dom file:///home/uraki/uraki_ws/tmp/local_fk_motion_20260916/plotly.html#selftest
```

CSVと元グラフの採用数一致、JavaScript構文、ブラウザー内のゼロ角度差表示・全点表示・3分類の切り替え・回転側の辺なし・視点変更を確認。`data-selftest="passed"` と `data-ready="true"`、3D描画画像を確認。生成Python・検証Chromeは終了、開始前後のプロセス比較で検証プロセスと新規Chrome補助プロセスの残留なし。専用profileを削除。検証のソフトウェア描画設定は利用者の通常ブラウザー設定への変更なし。

<a id="並進成分と回転成分による2分類への修正"></a>

**並進成分と回転成分による2分類への修正**

ユーザーの意図は、微小な関節変化による動きを並進的・回転的なものへ分けること。特定の手先軸の平行性を独立した分類として扱った初回の解釈を修正。

各 `q0+Δq` のFKから、並進成分 `R0ᵀ(p-p0)` と回転成分 `R0ᵀR` を計算。並進成分の長さと、姿勢全体の相対回転角で分類。回転角の大きさは座標基底や選択した手先軸によらない。

| 現在の分類 | 採用条件 | 描画 | 採用数 |
| --- | --- | --- | ---: |
| 並進的な動き | 相対回転角が0.25度以内 | 並進空間のグラフ | 2,102 |
| 回転的な動き | 並進量が0.5 mm以内 | 相対RPY空間の点群 | 566 |

ほぼ動かない4サンプルは両方へ所属。並進・回転ともに許容値を超えた4,167,313サンプルは混合した動きとして件数を記録し、どちらかへの強制所属なし。混合群の全サンプル保存・描画は未実施。関節格子・範囲・採用許容値は初回から変更なし。

並進的な候補では姿勢全体の回転が小さいため、任意の同じ手先方向ベクトルも元姿勢に対してほぼ平行。方向ベクトルの平行性だけではその方向まわりの回転を区別できないため、特定軸の平行判定・軸誤差計算は削除。2つの成分は同じ関節変化から得たものであり、両空間の任意の点を独立に合成できるという保証ではない。

計算スクリプト・Canvas表示・Plotly表示を2分類へ統一。`format_version=2` の数値形式へ更新し、CSVの分類列は `is_translation_like` / `is_rotation_like`。以前の結果とスクリプトは `tmp/local_fk_motion_20260916/before_motion_separation/` に保持。

4,169,977通りを再評価し、旧結果の「向き全体維持」「位置維持」と新分類の全サンプルID・数値の一致を確認。最大並進9.645 mm、最大相対回転5.531度。並進グラフは2,102点・9,976辺・連結成分1。FKと残差計算10.650秒、画像生成前までの解析12.320秒の一回の実測値。

解析解、特定方向まわりの回転の回転成分への計上、座標基底変更に対する成分の大きさの不変性、逐次FK・関節限界・格子順序、辺の17点補間を検証。辺の最大回転量は0.249967度。Plotlyの全採用数・2分類・回転側の辺なし・視点操作と実描画を確認。

実行コマンド：

```bash
timeout -s INT -k 5s 120s python3 gng_vlut_system/scripts/inspect_local_fk_motion.py --output-dir tmp/local_fk_motion_20260916
python3 gng_vlut_system/scripts/view_local_fk_motion.py tmp/local_fk_motion_20260916
timeout -s INT -k 5s 40s python3 tmp/local_fk_motion_20260916/verify.py
node --check tmp/local_fk_motion_20260916/index_motion_viewer.js
node --check tmp/local_fk_motion_20260916/plotly_motion_viewer.js
timeout -s INT -k 5s 50s google-chrome --headless --no-sandbox --no-first-run --no-default-browser-check --disable-background-networking --use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader --user-data-dir=/tmp/local_fk_motion_components_chrome_20260916 --window-size=1500,1100 --hide-scrollbars --virtual-time-budget=15000 --screenshot=/home/uraki/uraki_ws/tmp/local_fk_motion_20260916/motion_plotly_browser.png --dump-dom file:///home/uraki/uraki_ws/tmp/local_fk_motion_20260916/plotly.html#selftest
```

数値検証は `motion_separation_verification.json`、描画確認は `motion_plotly_browser.png` と `motion_plotly_browser_dom.html`。Python・検証Chromeは終了、プロセス残留なし、専用profileを削除。入力GNG・URDFへの変更なし。
