# 把持ファジィ評価の相談用スライド

## システム全体フロー：上方把持の順序修正版

- [PNG：閲覧・スライド貼付用](./system_grasp_flow.png)
- [SVG：16:9スライドの配置・文字・矢印の編集元](./system_grasp_flow.svg)
- [DOT：接続関係の参照用](./system_grasp_flow.dot)

2026-09-14作成。把持面の寸法確認、手先位置・姿勢候補の生成、候補姿勢に基づく付属ノード・対象寸法確認を区別。
[上方把持推定器](../../../grasping_system/include/candidate/top_grasp_surface_estimator.hpp)の処理順を参照。付属ノードは参照平面方式で確認し、参照面がない場合は抽出を省略。対象寸法は物体全体の寸法保証ではない。
幾何的候補の出力とロボット側の関節姿勢探索を分離。ファジー評価は設計段階・未接続、軌道生成は今回省略として表示。
図は処理とデータ依存の概要であり、各ROSノードが同期して順番に実行される意味ではない。

スライド用に1920×1080の固定配置へ調整。上段は環境認識と把持候補生成、下段は環境・ロボット照合と関節姿勢評価。
配置の正本はSVG、DOTは接続関係の参照用。DOTから同名SVGを再生成するとスライド配置が失われるため、PNGだけをSVGから書き出す。

PNGの再生成（システムPythonのPyGObject・Cairo・librsvgとNoto Sans CJK JPが必要）:

```bash
/usr/bin/python3 - <<'PY'
import gi
import cairo
gi.require_version("Rsvg", "2.0")
from gi.repository import Rsvg

base = "gng_vlut_system/docs/presentations/system_grasp_flow"
surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 1920, 1080)
handle = Rsvg.Handle.new_from_file(base + ".svg")
viewport = Rsvg.Rectangle()
viewport.x, viewport.y = 0, 0
viewport.width, viewport.height = 1920, 1080
handle.render_document(cairo.Context(surface), viewport)
surface.write_to_png(base + ".png")
PY
```

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
- [元資料：統合設計書・入力情報・制約・集合](../designs/fuzzy_grasp_design.md#inputs)

統合前の設計案から作成したスライド。統合に伴うPowerPoint・PDFの再生成は未実施。最新の設計内容は上記の統合設計書を参照。

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
