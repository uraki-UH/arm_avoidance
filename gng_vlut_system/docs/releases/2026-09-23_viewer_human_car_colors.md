# 2026-09-23 - ViewerのHuman・Car分類色

## 1. 要約

TopoFuzzy ViewerのHumanを赤紫、Carを青紫へ変更。

共通パレットのラベル4を`#d946ef`、ラベル5を`#8b5cf6`へ変更。通常分類によるノード・クラスタ・ラベル一覧・ラベル色モードのvoxelへ共通適用。[配色定義](../../../ToPoFuzzy-Viewer/frontend/src/types/index.ts)。

Human・Carと地形の緑系配色の重複を解消。

## 2. 条件・検証

通常分類色だけの変更。選択中の白色、属性ラベルの優先色、クラスタ単位の識別色は既存規則を維持。反映確認はブラウザの再読み込み。

Frontend lint、コンテナ内の本番ビルド、既存の状態色・クラスタ描画テスト2件が成功。localhost:5173から配信された色定義に両色を確認。新しいテストの追加なし。

実行コマンド:

```bash
cd /home/uraki/uraki_ws/ToPoFuzzy-Viewer/frontend
npm run lint
npm run build
node --test tests/l0_state_colors.test.mjs tests/cluster_graph.test.mjs
cd /home/uraki/uraki_ws
docker compose exec -T frontend npm run build
```

ホストのビルドは既存`node_modules/.vite-temp`の権限で失敗したため、既存Frontendコンテナ内で再実行して成功。権限変更や再インストールなし。ビルド時のチャンク容量警告あり。

検証プロセスは全終了。ROSノード・再生・Webサーバーの新規起動、既存プロセスの停止・再起動なし。

**制約**

実点群のブラウザ画面での目視確認は未実施。Backendの変更・ビルド・mergeは今回未実施。
