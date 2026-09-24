# 2026-09-15 - HTMLとViewerの把持ラベル統合

## 1. 要約

HTMLの把持部位付き点群を`/semantic_points`へ集約し、Viewerの把持部位・到達性を「把持ラベル」へ統合。

- ViewerのHANDLE親項目をなくし、「把持ラベル」の子「把持部位」へ移管。未評価・到達範囲内・到達範囲外も同じグループ内で個別設定。
- ノード詳細の名称も統一。不明な数値を剰余演算で既知ラベルへ誤変換しない扱い。
- 子ラベルID・数値・個別色を維持。旧到達性グループOFFは到達性の子だけへ反映。
- 境界との優先順位は統合グループ単位。旧HANDLEと旧到達性が境界の両側にあった場合、旧2親のうち先に指定された位置を採用。

- HTMLの実PointCloud2生成関数を用いた全点保持・ラベル値検査。
- GUIグループ構造と旧表示設定移管の回帰検証。

- HTMLの縁・蓋・机等の数値が候補到達性の2〜4と衝突する定義を削除。HTMLの出力は通常0・把持部位1のみ。
- 数値`semantic_label: 1`も把持部位として維持。

**削除**

HTMLの`handle_points`チェックボックス・トピック欄・配信器のみ削除。把持部位の点生成・抽出機能は維持。
ラベルなし`/topo_points`の任意配信は変更なし。

## 2. 条件・検証

- HTMLの既定ONである`semantic_points`から物体全体と把持部位ラベルを配信。
- 既存の`/handle_points`購読者は`/semantic_points`の`semantic_label == 1`を使用する必要あり。
- HTMLの把持部位指定は、ロボットの到達性・把持成功を保証しない。
- HTMLの再読み込みが必要。既存のブラウザ・ROSノードは本作業から再起動していない。

| 数値 | 意味 | 生成側 |
| --- | --- | --- |
| 0 | 通常 | HTML・GNG |
| 1 | 把持部位 | HTMLからGNGへ継承 |
| 2 | 到達性未評価 | 候補評価側 |
| 3 | 到達範囲内 | 候補評価側 |
| 4 | 到達範囲外 | 候補評価側 |

`PointCloud2.semantic_label`、GNGの`semantic_label`、Viewerの`semanticLabel`の形式・値1〜4は維持。ROS側のソース変更なし。

frontendディレクトリで実行:

```bash
node tests/html_grasp_labels.test.mjs
node tests/boundary_evidence.test.mjs
node tests/boundary_label_modal.test.mjs
node tests/label_priority_browser.test.mjs
npm run lint
./node_modules/.bin/tsc -b
./node_modules/.bin/vite build --configLoader runner --outDir /tmp/viewer-grasp-label-build-RvttrR
```

ラベル付きPointCloud2の座標・全点保持、把持部位の生成維持、0/1だけの符号化、旧表示設定移管、境界との優先順位、GUI構造、実ChromeでのON/OFF・長押し並べ替えを検証。
上記4検証とlint・TypeScript検査・本番アセット生成に成功。全検証プロセスは終了し、一時ビルド出力は削除。
HTML→ROS→GNGの実通信・実入力グラフは今回未検証。GNG側の別作業による変更は保持。

**制約**

専用Chromeは`--headless=new --disable-gpu --remote-debugging-pipe --user-data-dir=/tmp/label-priority-browser-heKyLN`で起動し、PID 1060265と専用プロファイルを終了・削除。
通常ブラウザやROSノードの停止・再起動操作なし。
