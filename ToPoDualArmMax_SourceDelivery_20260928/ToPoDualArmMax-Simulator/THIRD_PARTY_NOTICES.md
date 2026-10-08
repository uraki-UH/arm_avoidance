# 第三者ソフトウェアと素材

|対象|版・著作者|条件 / 本文|
|---|---|---|
|React / React DOM / Scheduler|18.3.1 / 18.3.1 / 0.23.2 / Meta (Facebook)|MIT / licenses/REACT-LICENSE.txt|
|Three.js|0.180.0 / Three.js authors|MIT / app/vendor/three/LICENSE|
|three-mesh-bvh|0.9.1 / Garrett Johnson|MIT / app/vendor/three-mesh-bvh/LICENSE。Three.js import先を同梱パスへ変更|
|Draco decoder|Three.js r180同梱バイナリ / Google|Apache-2.0 / licenses/DRACO-LICENSE.txt|
|Littlest Tokyo|Glen Fox / glenatron|CC BY 4.0 / licenses/CC-BY-4.0.txt。[出典と加工内容](app/assets/environments/littlest_tokyo/ATTRIBUTION.md)|
|CarConcept|Eric Chadwick, Darmstadt Graphics Group GmbH, 2024|CC BY 4.0 / licenses/CC-BY-4.0.txt。Khronos / 3D Commerceのロゴ条件: licenses/KHRONOS-LOGO.txt|
|Ferrari 458 Italia|vicent091036|モデル固有の条件は一次情報で確認未了。Three.jsのMITと区別|
|MID-360 CAD / 計測由来方向|Livox|公式公開資料。独自の再配布許諾なし|

モデル原本は変更せず、表示時に軸・位置・寸法・材質を変更しています。MID-360はSTEPからSTLへ変換、LVX2から走査方向へ変換しています。正確な出典URL、加工内容と確認状況は [素材一覧](docs/ASSET_SOURCES.md) に記載しています。ソフトウェアのライセンスと、ロゴ・ブランド・3Dモデルの権利を一括扱いしていません。

## ROS Scene Layersの追加依存

生成JavaScriptに含まれるパッケージとUI CSS。本文は [ROS-VIEWER-LICENSES.txt](licenses/ROS-VIEWER-LICENSES.txt) に同梱しています。

| パッケージ | 同梱版 | ライセンス |
| --- | --- | --- |
|react|18.3.1|MIT|
|scheduler|0.23.2|MIT|
|react-dom|18.3.1|MIT|
|react-reconciler|0.27.0|MIT|
|zustand|3.7.2|MIT|
|scheduler|0.21.0|MIT|
|@react-three/fiber|8.18.0|MIT|
|react-use-measure|2.1.7|MIT|
|its-fine|1.2.5|MIT|
|@babel/runtime|7.29.10|MIT|
|@react-three/drei|9.122.0|MIT|
|three-stdlib|2.36.1|MIT|
|lucide-react|0.562.0|ISC|
|three|0.180.0|MIT|
|urdf-loader|0.12.7|Apache-2.0|
|tailwindcss|4.3.3|MIT|
|fflate|Three.js r180同梱|MIT|

Fiberの本文: https://github.com/pmndrs/react-three-fiber/blob/v8.18.0/LICENSE 。URDF Loaderの本文: https://github.com/gkjohnson/urdf-loaders/blob/v0.12.7/LICENSE 。
