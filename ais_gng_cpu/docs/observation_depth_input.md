# depth画像由来の画素番号保持とGNG整数角度表（2026-09-09）

## 構成

```text
depth Image + 同時刻のCameraInfo
  → XYZとpixel_idxを同時生成
  → XYZのGNG学習、元画素番号の引き継ぎ
  → 画素→16ビットyaw/pitch表の参照
  → 第一勝者の角度範囲更新
```

XYZからu/vへ戻す逆投影なし。点群を作った元画素をそのまま使用。
voxel代表点、attention点、一様間引きにも元の入力点番号を引き継ぎ、pixel_idxに接続。
画素番号の全点コピーは不要。元PointCloud2への借用ビューから、選択された学習点だけを参照。
organized入力では元点番号そのものを画素番号として利用。
attention点はXYZの全点複製を省略。
小入力では既存走査中に元入力番号を保存し、大入力では連続区間＋64点単位の索引を使用。
実入力65,536点を境に内部で自動切替。追加のユーザー設定なし。
方向範囲のフレーム初期化は前フレームで更新したノードのみが対象。
[改善版の処理コスト比較](observation_pixel_optimization.md)。
単一区間のノード保持とROS出力version=5は[既存仕様](observation_support.md)のまま。

## 現在の録画への対応

確認したdepthは `/camera/camera/depth/image_rect_raw` の848×480、16UC1。
CameraInfoは `/camera/camera/depth/camera_info`。
既存点群はheight=1、XYZ/RGBだけで画素番号なし。そのメッセージ自体は変更なし。
別系統でdepthから再生成するため、元bagへの書き込み・再生設定の変更は不要。

RealSenseは内部で画素からXYZを生成し、ROSの非organized出力時に点を詰め直す構成。
[SDK実装](https://github.com/realsenseai/librealsense/blob/master/src/proc/pointcloud.cpp)、
[ROS出力実装](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/src/pointcloud_filter.cpp)。
将来の直接カメラ入力では、ordered_pcによる画素順の維持も利用可能。

今回の再生成点群には色なし。正の有限深度を採用し、RGB視野外による除外は未実施。
既存の色付き点群と同じ点数・同じフィルタ結果になる保証はなし。

## 追加した実行系

`depth_pixel_points.py`：

- depthとCameraInfoを取得時刻の完全一致で同期。各入力8フレームの有限バッファ。
- 対応する画像寸法・optical frame、歪みなし、ROIなし、K/P一致の校正だけを許容。
- 16UC1は既定depth_unit=0.001 m、32FC1はm。異なる16UC1単位は明示設定。
- 画像の行余白・big endianに対応。無効深度とfloat32非有限XYZを除外。
- 出力はXYZ各float32とpixel_idx uint32。pixel_idx=v*width+u。point_step=16。
- 同じマスクでXYZと画素番号を選択し、詰め直し後も対応を維持。
- センサから出力座標への固定回転と並進を適用。XYZ用の画素レイもキャッシュ。
- 変換用の一般パラメータ既定値は原点0、回転なし、target_frame空。
- 変換ノードのパラメータは起動時固定。実行途中の校正変更によるキャッシュ不整合の防止。
- max_frame_num=0は連続動作。正の値は指定フレーム数の出力後に終了。

GNG側：

- input.observation_camera_info_topicで高速経路を指定。空なら従来動作。
- CameraInfoは8フレームを保持し、入力点群と同一時刻のものだけを使用。
- pixel_idx、またはu/v（UINT16/UINT32）のフィールドから元画素を取得。
  RGB用の正規化テクスチャUVではなく、CameraInfoに対応するdepth画素の整数座標が対象。
- input.enable_observation_organized=trueならCameraInfoと同寸法の画像状点群も利用。
  「点の順序が元depth画素順」であることは入力側の契約。寸法だけで自動推定しない。
- 一様間引き・行余白の詰め直し前の番号から、選択した入力点の画素番号を取得。
- 既存GNGのXYZ入力は先頭にlittle endian float32のx/y/zを持つ配置が前提。
- 原点・回転・校正・時刻一致・画素番号のいずれかが使えなければ、そのフレームは従来レイ計算。
- 一部だけ無効な画素番号の場合は、その点だけレイ計算。
- 原点自体が不明なら、従来どおり支持記録を無効化。

## 座標系と表の寿命

今回の実装は出力座標系を変えず、表の角度もGNG座標系。
表のキーは画像寸法・焦点距離・主点・カメラからGNGへの回転。
固定カメラでは一度作成後に再利用。並進だけの変化では表の作り直し不要。
回転または校正が変わった場合は表を再構築。回転が毎フレーム変わる用途では再構築コストに注意。
カメラ基準の角度に変更して移動時も表を固定する方式は、今回の出力互換性維持の対象外。

固定原点指定時には、input.observation_camera_rotation=[x,y,z,w]も必要。
TF使用時は、指定センサから点群フレーム、さらにGNGフレームへの回転を同様に適用。
表はray_angles（yaw/pitch各uint16_t、有効yawフラグ、padding込み6 B）の配列。
848×480では2,442,240 B。ノード側は引き続き10 B/ノード。
画素番号の全点一時配列・voxel実測代表点の複製配列は不要。
小入力のattention元番号はuint32_t配列。実入力点数に合わせた再利用領域で、使用部分はattention点数。
大入力では12 B/連続区間と、概ね64点あたり1個のuint32_tおよび末尾番兵の索引。
前フレームの支持ノード番号もuint32_tの可変長配列。
間引き番号は既存の点群選択配列を借用。ノードIDの削除・再利用による重複クリアも許容。

## 起動

Docker内のworkspaceでgng_cpuとais_gngを再ビルド後：

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gng_cpu ais_gng --symlink-install
source install/setup.bash
ros2 launch ais_gng depth_observation.launch.py
```

depth_observation.launch.pyはpointcloud_transformer_cppの
config/realsense_calibration.yamlを読み、同じRx*Ry*Rz順で回転を構築。
今回の原点は(0.434,-0.693,0.279) m、roll/pitch/yawは(-103.8,-28.9,-3.4)度。
既に変換済みの点群を再変換する構成ではなく、optical frameのdepthからの一回の変換。

既定の出力は別名前空間 `/observation_depth`：

- points：XYZと元画素番号。
- camera_info：点群生成に使用した同時刻のCameraInfo。
- topological_map：GNG。
- node_observation_support：整数角度範囲。
- node_observation_lookup_statistics：表参照・フォールバックの件数。

既定launchは最大入力500,000点、最大20,000ノード、学習4,000回。
形状学習経路の検証用に分類・平面／非平面の直接出力はOFF。
実点群試験の間引き設定とは異なり、このlaunch設定での周期保証は未実施。
停止はCtrl+C。元のbag再生を止める必要はなし。
calibration_file、depth_topic、camera_info_topic、namespaceはlaunch引数で指定可能。

## 統計とAPI

node_observation_lookup_statisticsはUInt32MultiArrayで、以下の6要素：

```text
frame_number, stamp_sec, stamp_nanosec, pixel_hit_num, ray_num, table_build_num
```

購読者がいるときだけ出力。ray_num=0かつpixel_hit_num>0が学習時の逆レイ計算なしの状態。
表生成時には全画素の角度計算を実行するため、「全処理で三角関数がない」という意味ではない。

core APIでは、入力設定と原点設定の後にgng_set_observation_pixel_viewを呼び出し、gng_execまで
元点群・間引き番号配列・角度表を生存させること。ビュー構造体自体は値コピー。
原点の再設定・次入力・学習完了・支持設定変更で失効。
旧gng_set_observation_pixelsも互換APIとして維持。連続した画素番号配列を別途用意する場合の入口。
gng_get_observation_lookup_statisticsで参照件数を取得。
元の配列は借用であり、並行更新・再確保は禁止。

## 実点群での検証結果

配信中depthで変換65フレーム、GNG出力60フレームを検査。
別の検証用名前空間とノードで実行し、既存の再生・ノードは変更なし。

- 点数：325,092〜352,989点/フレーム。元depthとXYZ・画素番号の対応を各フレーム約100点で照合。
- GNG入力は20,000点への一様間引き、最大1,024ノード、学習4,000回。
- 60/60フレームで画素表参照。参照件数は3,850〜4,000回/フレーム。
  初期の勝者未成立イベントなどは支持更新対象外。
- レイ計算へのフォールバックは合計0回。角度表の構築は1回だけ。
- Pythonのdepth→点群変換は初回約65.0 ms、その後中央値約26.65 ms。
  この時間は新しい点群生成コストであり、GNGの角度更新時間ではない。
- この追加変換ノード込みで、以前の点群入力より高速になったとの主張は不可。
  本番での負荷削減には生成元のorganized点群利用や変換のC++化が別途候補。

現在の回帰テストはdepth変換、画素ビュー、core API、整数角度範囲。
実トピック検証の再実行は、ビルド済みGNGを指定して以下を使用（Docker内）：

```bash
source /ros2_ws/install/setup.bash
python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/observation_pixel_live_test.py \
  --executable /ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu \
  --output /tmp/observation-pixel-live.json
```

入力depthとCameraInfoの配信が必要。テスト自身のGNG・変換ノードは終了時に停止。
