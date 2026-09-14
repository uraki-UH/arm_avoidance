# 把持ファジィ評価の相談用スライド

## 現行実装の説明用：全12枚

- [PowerPoint：現行入力・集合・全ルール・フロー](./fuzzy_grasp_current_implementation.pptx)
- [PDF：現行入力・集合・全ルール・フロー](./fuzzy_grasp_current_implementation.pdf)
- [初期モデルJSON：HTMLソースから自動抽出](./fuzzy_grasp_current_defaults.json)
- [現行説明用の生成スクリプト](./build_fuzzy_current_slides.py)
- [詳細資料：現行実装](../FUZZY_GRASP_CURRENT_OVERVIEW.md)

把持用11入力・12ルール、前段のクラスタ用2ルール・意味ラベル用4ルール、全メンバシップ関数パラメータ、合成規則、HTML／ROSそれぞれのフローを掲載。
対象は `ToPo-FUZZY_Manipulation_v1.html` のソース初期値。ブラウザーのGUI編集・読込後の状態ではない。
JSONにソースのSHA-256、初期モデル、端点検証値を記録。

```bash
python3 gng_vlut_system/docs/presentations/build_fuzzy_current_slides.py
libreoffice --headless --convert-to pdf --outdir gng_vlut_system/docs/presentations gng_vlut_system/docs/presentations/fuzzy_grasp_current_implementation.pptx
```

生成には下記のPython環境に加えてNode.jsが必要。図形描画は設計案スライドの生成スクリプトと共通。

## 今後の設計案：全8枚

- [PowerPoint：編集用](./fuzzy_grasp_design_draft.pptx)
- [PDF：閲覧・共有用](./fuzzy_grasp_design_draft.pdf)
- [生成スクリプト](./build_fuzzy_grasp_slides.py)
- [元資料：入力情報・制約・集合の設計案](../designs/fuzzy_grasp_input_design.md)

2026-09-14作成、16:9、全8枚。図・表・グラフはPowerPointの編集可能な図形と文字。
未実装の設計案と現行の入力取得状況を区別。メンバシップ関数の数値境界は未確定。

## 構成

1. 目的：つかみやすさをルールで説明
2. 全体像：環境GNG・関節配置GNGと候補評価
3. 入力①：観測・形状の信頼度
4. 入力②：把持品質
5. 入力③：アーム・経路品質
6. ハード制約・未確認・順位付けの区別
7. ファジィ集合の概念図
8. 代表IF–THENルールと相談事項

## 再生成

必要環境: Python 3、python-pptx 1.0.2、Noto Sans CJK JP。PDF変換にはLibreOffice。
以下はリポジトリルートでのコマンド。生成先は本ディレクトリ内の同名ファイル。

```bash
python3 gng_vlut_system/docs/presentations/build_fuzzy_grasp_slides.py
libreoffice --headless --convert-to pdf --outdir gng_vlut_system/docs/presentations gng_vlut_system/docs/presentations/fuzzy_grasp_design_draft.pptx
```

作成時は一時ディレクトリへ依存ライブラリを導入し、専用のLibreOfficeプロファイルでPDF変換。
生成・PDF変換プロセスは終了済み。ROSノードの新規起動なし。
PDF全8ページの目視確認、ページ外図形検査、PDFページ数確認済み。
