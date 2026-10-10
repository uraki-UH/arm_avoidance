# 3Dモデル・依存ライブラリの出典

取得日: 2026-09-28 (JST)。モデルはローカルに同梱し、通常動作時の外部通信は不要です。

## Livox MID-360

Longの胸部モデル:

- 入力: ユーザー提供`urdf/ToPoDualArm-Max (long)45d.step`。
- 同梱形状: `app/meshes/chest_lidar_mount_45.stl`、`chest_lidar_color_0.stl`〜`chest_lidar_color_3.stl`。ROS用`urdf/topo_dual_arm_max_long/meshes/`と同一のSTL。
- 青色カバー: STEP球面の軸合わせ・隣接面の接合後に三角形化。元の半径・中心・上下境界を維持、面の欠損を回避。再生成処理は元ワークスペースの`scripts/rebuild_chest_lidar_cover.py`。
- 原点・姿勢: 同梱`app/source.urdf`の固定リンク。mm→mのみ、追加の軸回転なし。
- 材質: STEPの面色4種類をlinear RGBで保持。ブラケットはロボットのgreen材質。実機の反射率・透過率の再現は対象外。
- 描画原本: workspaceの`urdf/topo_dual_arm_max_long/`。ROS・ブラウザのvisual・色・STLの共通ソース。ブラウザ同梱分は生成物で、直接の形状修正は原本側。
- 描画キャッシュ: 下記コマンドで原本を同期して再生成。形状ハッシュは`app/assets.json`。同梱データだけの再生成は従来の`python3 tools/rebuild_meshes.py --model long`も使用可能。
- 権利: 提供データの権利条件を継承。新たな再配布許諾の付与なし。

workspaceルートからの描画アセット同期:

```bash
python3 -B ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/tools/rebuild_meshes.py \
  --model long --source-urdf urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf
```

同期範囲はvisual・材質・参照メッシュ。リンク構成一致が必要。ブラウザ用のcollision・関節・慣性は保持し、物理モデル全体の上書きはなし。STL未変更時は既存キャッシュを再利用。配布後の実行時にworkspaceへのアクセスは不要。

ROS Viewerは`robot_viewer_bridge_node`起動時の参照メッシュ更新時刻・サイズをdescriptionに付加。同一URLでも更新情報の変更時は描画キャッシュを再取得。姿勢更新周期でのファイル走査・稼働中の自動監視はなし。原本変更後はブリッジの再起動が必要。キャッシュ修正前のページは初回のみ再読込み。

標準モデルの従来センサ:

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

## Hesai JT128

- 形状: Hesai公式CADの側面コネクタ型。取得日2026-10-10。
- 配布元: [Hesai JT128ダウンロード](https://www.hesaitech.com/product_downloads/jt128/)、[公式CAD ZIP](https://www.hesaitech.com/wp-content/uploads/2025/09/JT128-3D-Model.zip)。
- 原本: `app/assets/jt128/jt128-side-connector.stp`。ZIP内STEPと同一、原本の改変なし。SHA-256は`model.json`。
- 描画: 8部品・100,505三角形。OpenCascade三角形化（弦誤差設定0.06 mm・角度0.16 rad）後、部品別に表示用の簡略化。元CADの公称外形外ソリッド1個（幅約485 mm、`source_part_idx=4`）は表示対象外。細部は表示用の近似。
- 座標: mm→m、Z軸−90°回転、Z方向−47.84 mm。マニュアルFigure 5の座標原点へ描画を合わせ、Y前方をX前方へ変換。点群走査の姿勢・原点の変更なし。
- 材質: 黒い光学カバー、濃灰の筐体、銀色コネクタの外観用PBR。実機反射率の再現ではない。
- 再生成: `python3 -m pip install -r tools/requirements-cad.txt`後に`python3 tools/convert_jt128.py`。CAD変換ライブラリは通常起動時に不要。
- 権利: Copyright Hesai。メーカー配布データの権利条件を継承、独自の再配布許諾の付与なし。

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

## Littlest Tokyo（2026-10-07取得）

- 街並みGLB: `app/assets/environments/littlest_tokyo/LittlestTokyo.glb`。
- 作者・ライセンス: Glen Fox / glenatron、CC BY 4.0。GLB内`asset.extras`とThree.js公式サンプルの作者表記で確認。
- 出典・ハッシュ・表示時の変換: [同梱クレジット](../app/assets/environments/littlest_tokyo/ATTRIBUTION.md)。
- 表示・センサ用の固定環境。物理衝突とアニメーションは未対応。

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
