# 3Dモデル・依存ライブラリの出典

取得日: 2026-09-28 (JST)。モデルはローカルに同梱し、通常動作時の外部通信は不要です。

## Livox MID-360

- メーカー配布ページ: https://www.livoxtech.com/mid-360/downloads
- 公式原本: https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/mid-360-asm.stp
- `assets/mid360/mid-360-asm.stp`: 配布STEPをそのまま保存。
- `assets/mid360/mid-360.stl`: STEP全形状をOpenCascadeで三角形化したバイナリSTL。原本と同じmm単位・CAD座標。弦誤差の設定0.04 mm、角度0.12 rad。部品の省略なし。
- `convert_mid360.py`: 変換処理。ビルド環境はcadquery-ocp 8.0.1.0.0、VTK 9.6.2。実行時は不要。
- 表示時にCADのY上向きをシミュレータのZ上向きへRx(+90°)で変換し、mm→mへ変換。黒い光学カバー・金属筐体の表示色を追加。STL原本には色はありません。
- シミュレーションの原点はCAD原点をこの変換で写した点です。実機の計測原点・取付誤差を実測校正した値ではありません。GUIの取付XYZ/RPYで補正できます。
- Copyright Livox. 公式配布データであり、独自にオープンライセンスを付与したものではありません。
- 公称仕様: https://www.livoxtech.com/mid-360/specs
- ユーザーマニュアル: https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf

## Ferrari 458 Italia

- モデル作者: **vicent091036**。
- 出典・作者表記: https://github.com/mrdoob/three.js/blob/r180/examples/webgl_materials_car.html
- ファイル: https://raw.githubusercontent.com/mrdoob/three.js/r180/examples/models/gltf/ferrari.glb
- 元作者ページ（three.jsに記載）: https://sketchfab.com/models/57bf6cc56931426e87494f554df1dab6
- 元作者ページは参照不可で、モデル固有のライセンス条件は一次情報による確認未了です。Three.jsのMITライセンスをこのモデルへ拡張解釈していません。受領者による二次配布・公開の前に権利者の条件を確認してください。配布元three.jsの作者表記を保持しています。
- 変更: 軸と中心の変換、ボディ・ガラスのPBR材質、配置・サイズ変更機能。原本GLBは変更せず保存。

## Car Concept

- **Eric Chadwick / Darmstadt Graphics Group GmbH (2024)**。モデル・テクスチャは **CC BY 4.0 International**。
- https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/CarConcept
- https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/CarConcept/glTF-Binary/CarConcept.glb
- ライセンス: https://creativecommons.org/licenses/by/4.0/
- 元形状: Unity FanのCC0 Concept Car 004。モデル内のKhronos / 3D CommerceのロゴはKhronos Groupに帰属。
- ロゴの権利表記: https://github.com/KhronosGroup/glTF-Sample-Assets/blob/main/LICENSES/LicenseRef-LegalMark-Khronos.txt
- 変更: 軸・中心の変換、ボディ色、配置・サイズ変更機能。原本GLBは変更せず保存。
- これはコンセプト車の3Dアセットであり、メーカーの設計CADや実寸公差の保証値ではありません。

## ライブラリ

- Three.js 0.180.0 — MIT。`vendor/three/LICENSE`。
- three-mesh-bvh 0.9.1 — MIT, Garrett Johnson。https://github.com/gkjohnson/three-mesh-bvh 。`vendor/three-mesh-bvh/LICENSE`。モジュールのThree.js importを同梱パスへ変更。
- Draco — Three.js同梱デコーダ。`vendor/three/examples/jsm/libs/draco/README.md`。

車両名・ロゴはモデルを識別する目的で表示し、メーカーの公認・提携を意味しません。

## MID-360実測方向データ（2026-09-28追加）

- Livox公式Indoorサンプル: https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Indoor_sampledata.lvx2
- 配布ページ: https://www.livoxtech.com/mid-360/downloads
- 先頭16 MiBのみ取得し、先頭5秒の1,000,000点スロットの順序・欠損を保持して正規化。座標からの方向抽出のみで、元の室内形状をシーンへ追加していません。
- measured-directions.jsonにファイルハッシュと制約を保存。詳細はSENSOR_FIDELITY.md。

本送付物のファイル位置は app/ 以下です。LivoxのCAD・実測由来データには独自の再配布許諾を付与していません。公式サイトでの公開と、再配布許諾の確認は別です。
