# 2026-09-02 - HTML基幹GNG-WASM固定

## 概要

`ToPo-FUZZY_Manipulation_v1.html`の点群GNG学習を、埋込みの`gng_wasm_core`だけで実行する構成へ変更。

## 変更

- GNG algorithm選択を`基幹GNG-WASM`固定へ変更
- 初期学習、GNG実行、再生成後の学習、GNG再生操作を`gng_wasm_core/dist/gng_wasm_core.js`の自動読込み経路へ統一
- WASM実行失敗時のJavaScript GNGフォールバックを廃止
- 外部WASMファイルの手動読込みUIを廃止
- 入力点群の範囲とノード上限から`node_grid`を自動算出するWASM設定を追加
- `Node coverage radius [m]`を基幹CUGNGの`node.interval`へ渡すWASM設定を追加
- WASM ABI版数の検査を追加し、旧版WASMでの学習継続を停止
- HTML内に残っていた旧直結WASMの自動実行を無効化
- ABI版数を検査する手動WASM読込みUIと内蔵WASM再読込みUIを復帰
- `graspnet.yaml` の基本学習値を適用するHTMLプリセットを追加
- 大きい点群とグラフJSONをヒープ経由で転送する方式へ変更
- 局所相互近傍グラフによる表示・保存用エッジの再構築を追加
- 初期表示をobject only、法線・クラスタ非表示へ変更

## 挙動

WASMの読込みまたは学習に失敗した場合、学習を停止して画面上に失敗理由を表示する。旧JavaScript GNGは学習・再生の実行経路から呼び出さない。

HTMLはWASM ABI版数`2`を要求する。古い`gng_wasm_core.js`がキャッシュまたは別配置から読まれた場合は、学習結果を表示せず更新を促す。

`node_grid`未指定時の`0.5 m`固定値は、小物点群を少数セルへ集約して1セル当たり10ノードの上限に達する。HTMLは学習点群からセル幅を自動設定し、`0.060 m`未満にはしない。これは基幹CUGNGの周囲1セル最近傍探索が既定の警戒半径`0.1 m`の候補を取りこぼさないための下限であり、物体全域の正しい勝者・第2勝者選択を維持する。

基幹CUGNGは`node.interval`の半径内に勝者がある入力点ではノードを増やさない。HTMLは既定`0.010 m`を全ラベルへ渡し、初期表示の小物点群に対するノード密度を制御する。値を小さくすると点群追従性を優先してノード数が増え、大きくすると抽象化を優先してノード数が減る。

初期表示では物体だけを学習する。机を含む環境GNGは`Table support`から`on table / tabletop`または`on table / tabletop + front edge`を選択して生成する。法線とクラスタはVisibilityから必要時だけ表示する。

GNG-WASMの読込みには、HTMLを`http://`または`https://`で配信し、`gng_wasm_core/dist/gng_wasm_core.js`と対応する`.wasm`ファイルへ到達可能なことが必要。

## API影響

ROS topic、launch引数、message、service、基幹GNGバイナリ形式の変更なし。

## 検証

- `node --check /tmp/topo_fuzzy_first_script.js`
- Docker内Emscriptenによる`bash scripts/build_wasm.sh dist`
- 3,750点のbasket形状入力で`node_grid=0.020 m`、`0.050 m`、`0.080 m`、`0.100 m`を比較し、`0.050 m`以上で探索範囲を満たしつつ全域被覆を確認
- ヘッドレスChromeの初期描画でobject only、約466点、63 node、124 edgeを確認
- `git diff --check`

## 注意

旧JavaScript GNGの初期化、逐次更新、再生ループは削除済み。グラフ可視化と読込み済みグラフの整合化に使う共通処理は残す。
