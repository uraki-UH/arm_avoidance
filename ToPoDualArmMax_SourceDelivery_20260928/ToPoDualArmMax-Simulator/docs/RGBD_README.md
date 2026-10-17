# テーブル・物体配置とカメラ（RGB-D／単眼／ステレオ／魚眼）

## 単眼・ステレオ・魚眼カメラ

単眼・ステレオ・魚眼のカラー／モノクロ撮影。RGB-Dとは独立した画像機能。
連続取得中は単発取得・校正変更を禁止。
設定・既定値は[camera-core.js](../app/camera-core.js)。初期校正は汎用の近似値で、特定機種・個体の再現保証なし。
実機との一致には使用モードの内部・外部校正値が必要。保存済み`calibration.json`を再読込可能、ROS YAML・メーカー固有JSONは非対応。

### 校正・座標の契約

- `left` / `right`: 左右別の解像度 `width`・`height` [px]、内部パラメータ `fx`・`fy`・`cx`・`cy` [px]。画素(0,0)は左上画素の中心。単眼・魚眼は `left` のみ使用。
- `distortion_model: plumb_bob`: `d = [k1, k2, p1, p2, k3]`。歪みなしは全係数0。
- `distortion_model: equidistant`: `d = [k1, k2, k3, k4]`。魚眼モードの必須方式。ステレオでも左右に指定可能。
- `max_fov_deg`: 光軸を中心とする有効円錐の全角 [°]。魚眼上限180°、ピンホール上限179°。画像の水平画角とは別。画角外・逆投影不能の画素は透明。
- `mount.translation_m` / `mount.rpy_deg`: URDFの `camera_optical_frame` から左眼への取付補正 [m / °]。光学軸はX右・Y下・Z前、回転合成は `Rz(yaw) × Ry(pitch) × Rx(roll)`。首・腰・ロボット全体の姿勢に追従。
- `right_in_left.translation_m` / `rotation`: 左眼座標で表した右眼の位置 [m]・姿勢。回転は行優先9要素、関係式は `p_left = R × p_right + t`。正のX方向が右側への基線。OpenCVの左→右変換 `R_cv, t_cv` からは `R = R_cvᵀ`、`t = -R_cvᵀ × t_cv` への変換が必要。

投影式の参照: [OpenCVのピンホール・歪みモデル](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)、[OpenCVの魚眼モデル](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)。スキュー、8係数以上の歪み、180°超の魚眼は対象外。

### 保存・再現範囲

画像保存ZIP: `left.png`、ステレオ時の `right.png`、`calibration.json`、`frame.json`、`scene.json`。左右は同じシーン時刻の描画、同じフレーム番号・取得時刻。画像は未平行化の歪み付き画像で、モノクロ選択時は保存画像もモノクロ。`frame.json` の `optical_to_world` / `robot_to_world` は列優先16要素、並進単位m。取得時の校正・関節角度・シーンを保存し、保存操作時の姿勢への置換なし。

Nodeサーバーの保存先: `app/exports/`。静的配信時はブラウザのダウンロード。モデル切替時は校正を保持し、連続取得を停止・旧画像を破棄。

- 再現対象: 幾何的遮蔽、左右の視差、内部・外部パラメータ、レンズ投影
- 未対応: 新カメラのROS画像配信、視差からの深度推定、ステレオ平行化、露光・ノイズ・ローリングシャッター・実機固有の分光特性
- 精度制限: 内部描画と再投影の離散化。モノクロは線形RGB輝度の近似で、実機の感度特性・IR画像は対象外

検証・性能計測の入口は[開発ガイド](DEVELOPMENT.md)。

## 環境と取得状態

点群は取得時の世界座標へ固定し、次フレームで更新。解像度・校正の変更時は旧画像・点群・計測値を破棄して再取得。
保存対象は同じ取得フレームの画像・深度・全点群・姿勢・環境。

テーブルの位置はモデルのURDFルート基準（Longは`base_link`、標準は`base_footprint`）。物体位置は天板中央を原点とするテーブル座標です。物体のZ=0は底面が天板上にある状態です。テーブルを動かすと物体も追従します。落下・接触の計算には物理モードが必要。

## 点群取得の仕組み

深度・点群の設定は[rgbd-core.js](../app/rgbd-core.js)、処理構成は[ARCHITECTURE](ARCHITECTURE.md)を参照。
WebAssemblyを利用できない場合はJavaScript版へ代替。追加インストール不要。
高頻度の取得中は通常ビューの描画頻度が低下する場合あり。

深度用の独立したGPU描画で、各画素に最初に見えるメッシュ表面の**光軸方向の奥行きZ [m]**をfloat32として取得します。カメラからの斜距離ではありません。アンチエイリアスによる深度の混合や点の間引きは行いません。範囲外・未検出画素は0です。近すぎる物体も遮蔽に参加し、その後ろの面が透けて取得されることはありません。

歪み補正済みピンホールモデルで、画素中心`(u,v)`から次の式で点を作ります。

```
X = (u - ppx) * Z / fx
Y = (v - ppy) * Z / fy
point_world = T_depth_optical_to_world * [X,Y,Z,1]
```

光学座標はX右・Y下・Z前、単位m。ピクセル(0,0)は左上画素の中心です。URDFの`camera_optical_frame`に取付補正を適用し、首Yaw・首Pitch・腰Yawの変化を毎回反映します。世界配置への変換にはモデルのURDFルートとロボット配置を使用。カメラ自身の筐体、操作マーカー、グリッド、軌跡、点群表示は撮影対象から除外し、腕・テーブル・物体・床などは遮蔽に参加します。

RGBは深度と別の内部パラメータ・光学中心で描画します。深度点を`depth_to_color`でRGB座標へ変換し、RGBの深度バッファと照合して着色します。RGB視野外・RGB側で隠れた点は残したまま灰色にし、PLYの`color_valid=0`で区別します。表示色によって点のXYZを推定しません。表面のRGBはPBR描画値であり、実センサの分光応答・露光・ホワイトバランスの再現ではありません。

## 公称値と実機校正

初期値はD435i公称値由来の近似。解像度・内部パラメータ・基線・取得範囲は[rgbd-core.js](../app/rgbd-core.js)を参照。
幾何真値はシーン表面のfloat32深度。ステレオ可視性＋Z16近似は右眼の可視性確認と量子化であり、IR画像・視差マッチングの再現ではない区別。

実機と座標・画角を合わせるには、使用解像度の実機内部パラメータ、depth→color外部パラメータ、取付位置・角度の校正値をRGB-D校正JSONへ反映。ここでの回転行列は行優先の9要素です。librealsenseの生の`rotation`配列は列優先なので、転置して読み替える必要があります。`mount.translation`と`mount.rpy`はURDF光学座標から深度光学座標への変換（m、rad、URDFのZYX合成）です。

対応画像は歪み補正済み`model: none`・`coeffs: [0,0,0,0,0]`。非ゼロ歪みは入力時に拒否。
実機とSDK `pyrealsense2`があるPCで、Simulatorルートから任意に校正値を取得:

```bash
python tools/export_d435i_calibration.py
```

出力は`d435i-calibration.json`。同名ファイルがある場合は停止、取付姿勢の校正は対象外。
補助スクリプトの実機接続・動作は未検証。

実機固有の測距ノイズ、欠損、マルチパス、IR投影・ステレオマッチング、透明体・鏡面材質の失敗、RGBローリングシャッター、IMU、USB遅延は再現していません。全物体は深度取得上、不透明なメッシュとして扱います。幾何真値の計算精度を、実機D435iの測距精度や同一の点群取得結果と解釈しないでください。

## 保存ファイル

| ファイル | 内容 |
|---|---|
| `points.ply` | binary little endian。全有効点のXYZ [m]、RGB、color_valid、元のpixel_index。保存時に世界／深度光学座標を選択 |
| `points_display.ply` | 選択中の高さ／距離／RGB色を持つ全点。元のpoints.plyのRGBは変更しません |
| `rgb.png` | RGBカメラの画像。深度画角への引き伸ばしなし |
| `depth.f32` | 上から行優先のfloat32 little endian、光学Z [m]、欠損0 |
| `depth.z16` | 同じ画素配列のuint16 little endian。値×depth_scale=m、欠損0 |
| `frame.json` | フレームID、時刻、単位、内部・外部パラメータ、取得時の光学→世界行列、関節角度 |
| `calibration.json` | 取得時のカメラ設定 |
| `scene.json` | 取得時のテーブルと物体。環境タブの読込でも利用可能 |
| `read_capture.py` | NumPyで深度からXYZを復元し、optical/worldのNPYを出力する例 |

点群は`depth.f32`に対応します。幾何真値モードでは、量子化した`depth.z16`から復元した点と最大0.5 depth_scaleだけZが異なります。世界変換行列はJSON内の名前どおり**列優先16要素**で保存します。外部回転3×3の行優先と混同しないでください。

## 検証と性能

送付版の検証範囲は[HANDOVER](HANDOVER.md)、センサ近似とヒートマップの基準は[SENSOR_FIDELITY](SENSOR_FIDELITY.md)を参照。
連続取得の設定Hzは目標値。GPU・シーン・表示負荷に依存し、実機のフレームレート保証なし。

## 公式参照

- [D435i製品仕様](https://www.realsenseai.com/products/depth-camera-d435i/)
- [RealSenseの投影・逆投影と座標系](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/)
- [RGBへの投影と遮蔽](https://dev.realsenseai.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras/)
- [ROS公式モデルの近似外部パラメータ](https://github.com/realsenseai/realsense-ros/blob/ros2-development/realsense2_description/urdf/_d435.urdf.xacro)
- [SDKの外部回転行列の格納順](https://github.com/realsenseai/librealsense/blob/master/include/librealsense2/h/rs_sensor.h)

同梱配置例: `app/examples/five-objects.json`（環境タブで読込）。旧撮影ZIP・画像は送付物に含めていません。
