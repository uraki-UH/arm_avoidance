# 2026-09-24 - 選択クラスタへの車両モデル照合

## 1. 要約

選択クラスタへ軽自動車相当・セダン・バン・箱型トラックの4モデルを照合するバックエンドと独立ビューを追加。
車両らしい形状への適合と車種の絞り込みを分離し、判定保留、適合度、未対応部分を表示。
セダン以外は寸法仮定の簡易形状。未対応部分は遮蔽・観測不足・形状差を含み、物理的な破損率ではない。

## 2. 条件・検証

| 項目 | 内容 |
| --- | --- |
| 操作 | ユーザー指示により`/topological_map`のBBox・選択有効化ボタンを撤去。このトピックからの独立表示は対象外 |
| 選択 | 旧ON設定でも`/topological_map`の枠描画・ホバー・直接クリック・詳細取得を無効化。他トピックと照合APIは維持 |
| 計算 | 独立ノードでGNGノードと表面モデルのYaw＋XYZ位置合わせ。GNG学習・ラベルの書換えなし |
| 表示 | 観測一致率、モデル支持率、未対応率、一致点RMS。緑・黄・赤の対応表示 |
| 検証 | 欠損姿勢復元・トラック・平面保留・外れ値・入力検証、実WSとChrome、Release・lint・Frontend build成功 |
| 実bag | 33ノードの観測で一致100%・モデル支持13.6%、車両判定保留。車種の正解精度は未評価 |
| 起動 | 次回のviewer_stack起動に自動追加。起動済み旧Viewerには追加ノードの起動が必要 |
| Git | `*.log`と`*.tsbuildinfo`を除外。既存tsbuildinfo 2件は実ファイルを保持して追跡解除 |

[仕様・起動](../../../ToPoFuzzy-Viewer/doc/VEHICLE_REGISTRATION.md)、[条件・結果・再現・停止記録](../../../benchmarks/vehicle_registration_20260924/README.md)。

BBox撤去後もUI回帰7件、lint、Frontend本番ビルド成功。実ブラウザの再試験は未実施。
検証コマンド: frontend内で`npm run lint`、`node --test tests/inspection_bbox_gate.test.mjs tests/candidate_hover_frame.test.mjs tests/cluster_detail_panel.test.mjs`、`frontend`コンテナ内で`npm run build`。全コマンド終了、ROS起動停止なし。
