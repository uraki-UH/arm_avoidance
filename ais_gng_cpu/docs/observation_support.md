# GNGノードの観測方向範囲：16ビットyaw・pitch

CPU版GNGの第一勝者へ、実測点由来のレイ方向範囲を付加する機能。
既定OFF。一学習フレーム単位。XYZ列・方向セル集合の保持はなし。

元画素番号を利用する[depth入力経路](observation_depth_input.md)を追加。
この経路では画素→整数角度表を参照し、学習中のatan2・hypotを省略。
以下のレイ計算は、画素番号・対応表がない場合のフォールバック。

## レイと範囲の構築

```text
ray = measured_point - sensor_origin
yaw   = atan2(ray.y, ray.x)
pitch = atan2(ray.z, hypot(ray.x, ray.y))
```

レイはセンサから実測点への向き。座標系は点群と同じGNG座標系。
yawは+Xを0度、+Yを+90度、pitchはXY平面を0度、+Zを+90度とする仰角。
レイの軸回りのrollは未定義で、記録対象外。
以前の方向セル方式の `origin - point` とは符号が逆で、旧データとの直接比較は不可。

通常学習ではvoxel重心で第一勝者を選択し、そのvoxelの実測代表点1点からレイを計算。
attention側では選択された実測点を使用。ノード位置からの近似ではない。
第二勝者、近傍移動ノード、未選択点、新規追加だけで第一勝者がないイベントは対象外。
点群全点への新たな最近傍割り当てはなし。元の学習点選択・ノード更新処理は維持。

各レイを整数角度へ変換し、勝者ノードのyaw・pitch範囲を逐次更新。
量子化前の角度計算はdouble、保持はuint16_t。
atan2を2回とhypotを使用。角度計算そのものをなくす最適化ではない。

## 16ビットの端点表現

- yaw：[-180, 180]度を65536ビン、1ビン約0.005493164度。
- pitch：[-90, 90]度を65536ビン、1ビン約0.002746582度。
- `min_yaw, max_yaw, min_pitch, max_pitch`：各uint16_t、合計8 B。
- `has_support, has_yaw`：各bool、合計2 B。
- 現環境の`sizeof(angle_range)`は10 B。配列確保・保存数上限・打ち切りなし。
- Node全体のサイズには配置によるpaddingの影響あり。10 Bは支持構造体単体の容量。

端点は「含まれるビン」の番号。角度へ復号する際はminビンの下端とmaxビンの上端を使用。
したがって、実測方向を切り落とさない外側への丸め。
+90度は最終pitchビンに含まれ、上端は+90度。

```text
pitch下端 = -90 + min_pitch * 180 / 65536
pitch上端 = -90 + (max_pitch + 1) * 180 / 65536

yaw始端 = -180 + min_yaw * 360 / 65536
yawビン幅 = ((max_yaw - min_yaw) mod 65536) + 1
yaw終端 = yaw始端 + yawビン幅 * 360 / 65536
```

max+1や差の計算は32ビット以上で実施。uint16_tのままの上端計算は不可。
yawのmin/maxは循環区間の始端/終端であり、単純な数値の大小順ではない。
179度と-179度は境界をまたぐ約2度の区間。全周はyawビン幅=65536で表現可能。
新しいyawが区間外なら、既存区間を残したまま短い拡張側へ端点を更新。同値なら終端側。

真上・真下のレイはpitchのみ更新。yaw範囲が未取得ならhas_yaw=false。
零レイ・非有限レイは無効。無効レイの追加では既存範囲は不変。

### 単一区間としての意味

離れた観測の間も範囲に含む。yawとpitchを別々に囲うため、その組み合わせの穴も区別しない。
これは要求に合わせた方向範囲であり、各方向の観測済みビット集合ではない。
yawの広い・多峰性の分布では、オンライン拡張の結果が入力順に依存する場合あり。
全サンプルを並べ直した最短円弧の保証はなし。取り込み済みのビンの包含は維持。

測距角度間隔による追加膨張やCameraInfoからの分解能自動変更はなし。
今回のカメラ中央の約0.134度/画素より細かい量子化だが、画素の光学的な広がりのモデルではない。
未支持を物体不在の証拠として使用不可。遮蔽判定、可視性表、履歴統合、テンプレート棄却は未実装。

## フレームとノードの寿命

有効なフレームの先頭で全ノードの範囲をクリアし、学習中に再構築。
ノード初期化・再利用時とON/OFF設定変更時もクリア。
センサ原点は入力設定後に一回分を指定。次入力または次の学習で使い回さない。
出力用の原点は学習フレームとともに確定し、次入力用の原点指定とは分離。
原点欠落や空入力のフレームでは以前の範囲を残さない。

## ROS設定

CPU版YAMLのros__parametersへの追加例：

```yaml
node.enable_observation_support: true
input.observation_sensor_frame: camera_depth_optical_frame
```

点群取得時刻のTFでセンサ原点を取得し、必要なら点と同じ変換でGNG座標系へ変換。
点群フレームがセンサフレーム自身なら原点は(0,0,0)。
必要なTFの欠落、不正な座標変換、時刻ゼロでの時刻付き変換では支持を無効化。
複数入力点群の同期経路も支持記録対象外。
移動ロボットは取得時刻のTFを使用。単一フレーム内の点別原点・デスキューは未対応。

既にbase_linkへ変換済みの今回の録画で、基準座標に対してカメラが固定の場合：

```yaml
node.enable_observation_support: true
input.observation_origin: [0.434, -0.693, 0.279]
input.observation_origin_frame: base_link
```

この原点はpointcloud_transformer_cppの既知の変換に対応する位置。
input.observation_sensor_frameとの同時指定は禁止。入力フレームが一致しない場合は無効化。
固定原点は移動するカメラに流用不可。点群へのtransformer回転の再適用はなし。
既存の入力点群からGNG座標への変換がある場合、その変換は原点にも同様に適用。

これらは起動時専用。固定16ビットなので角度分割数や保存数の設定は不要。
旧node.observationのhorizontal_step_deg、vertical_step_deg、max_interval_num、
half_angle_deg、max_direction_num、max_cell_angle_deg、max_block_num、および途中案max_point_numは起動エラー。
旧設定の削除とgng_cpu・ais_gng双方の再ビルドが必要。

## ライブラリAPI

```cpp
gng_setParameter("node.enable_observation_support", 0, 1);
gng_setPointCloud(data, point_num, &config);
gng_observation_input input;
input.origin = sensor_origin_in_gng_frame;
input.has_origin = 1;
gng_set_observation_input(&input);
gng_exec();
const auto frame = gng_get_observation_frame();
const auto range = gng_get_observation_angle_range(node_id);
```

範囲は値返却。無効なノードIDはhas_support=false。
frameは原点、has_origin、frame_number、pixel_hit_num、ray_num。呼び出しの並列実行は不可。
観測入力は一括置換。任意のpixels・angle_table・table_num指定で画素表参照が可能。
pixels未指定はレイ計算。無効なビュー・表は受付結果0かつ有効原点によるレイ計算へ復帰。
nullptrまたは無効原点は観測入力全体の解除。次入力・学習完了・支持設定変更でも失効。
入力構造体は値コピー。参照先の画素・間引き番号・角度表はgng_exec完了まで保持。
次入力の設定では直近出力の原点・参照件数に変更なし。
旧原点・画素配列・画素ビュー設定API、独立した参照件数取得APIは廃止。
gng_observation_frameの構造変更を含むため、ライブラリと利用側の双方の再ビルドが必要。
旧get_observation_cellsと途中案get_observation_pointsのエクスポートはなし。

## ROS出力 version=5

node_observation_supportにstd_msgs/msg/UInt32MultiArrayを出力。QoS深さ1。
購読者不在時は直列化なし。既存TopologicalNodeの定義変更なし。
layout.dim[0].labelはGNG座標系、size/strideは全要素数、data_offsetは0。

```text
先頭9要素：
  version (=5), frame_number, stamp_sec, stamp_nanosec,
  has_origin, origin_x_bits, origin_y_bits, origin_z_bits, node_num

各ノード5要素：
  node_id, node_frame, flags, yaw_endpoints, pitch_endpoints

flags：
  bit0 = has_support, bit1 = has_yaw

yaw_endpoints   = min_yaw   | (uint32_t(max_yaw)   << 16)
pitch_endpoints = min_pitch | (uint32_t(max_pitch) << 16)
```

原点3成分はfloat32のビット表現。角度端点は浮動小数点ではなくuint16_t。
stamp_secはROSのint32秒のビット表現。node_frameはノード生成フレーム。
原点の有効性は全ノードに共通。各ノードの支持の有無とは別。
未支持ノードはflagsと両端点語が0。旧version=1〜4のデコーダは使用不可。

## 検証と再現

通常共有ライブラリのAPIテストと、整数角度範囲の単体テストを使用。
全65536ビンの境界近傍、yawの折り返し・全周・任意順の包含維持、
両極、非有限値、零レイ、フレーム失効、OFF、点群と原点の座標変換を検査。

ROSテストは隔離ドメイン187で実行。既存bag・ノードの停止や変更はなし。
テストスクリプトのfinallyで自身のGNGノードを停止。

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
observation_build_dir=$(mktemp -d /tmp/gng-angle-range-XXXXXX)
cmake -S /ros2_ws/src/ais_gng_cpu/src/gng_cpu -B "$observation_build_dir/core" \
  -DGNG_BUILD_BENCHMARKS=ON -DGNG_ENABLE_FRAME_LOG=OFF \
  -DCMAKE_INSTALL_PREFIX="$observation_build_dir/install"
cmake --build "$observation_build_dir/core" -j 4
cmake --install "$observation_build_dir/core"
LD_LIBRARY_PATH="$observation_build_dir/install/lib:$LD_LIBRARY_PATH" \
  ctest --test-dir "$observation_build_dir/core" --output-on-failure
cmake -S /ros2_ws/src/ais_gng_cpu/src/ais_gng -B "$observation_build_dir/ros" \
  -Dgng_cpu_DIR="$observation_build_dir/install/share/gng_cpu/cmake" -DBUILD_TESTING=ON
cmake --build "$observation_build_dir/ros" -j 4 --target ais_gng_component_cpu ais_gng_cpu test_node_support test_observation_pixels
export LD_LIBRARY_PATH="$observation_build_dir/ros:$observation_build_dir/install/lib:$LD_LIBRARY_PATH"
"$observation_build_dir/ros/test_node_support"
"$observation_build_dir/ros/test_observation_pixels"
PYTHONPYCACHEPREFIX=/tmp/observation-test-python-cache python3 \
  /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/depth_pixel_points_test.py
python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/observation_support_ros_test.py \
  --executable "$observation_build_dir/ros/ais_gng_cpu"
python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/observation_support_ros_test.py \
  --executable "$observation_build_dir/ros/ais_gng_cpu" --transform-cloud
python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/observation_support_ros_test.py \
  --executable "$observation_build_dir/ros/ais_gng_cpu" --fixed-origin
python3 /ros2_ws/src/ais_gng_cpu/src/ais_gng/test/observation_support_ros_test.py \
  --executable "$observation_build_dir/ros/ais_gng_cpu" --fixed-origin --transform-cloud
```

## 関連資料

画素番号付き入力の設定・導入手順は[depth入力](observation_depth_input.md)、
最適化の比較条件・結果は[処理コスト比較](observation_pixel_optimization.md)を参照。
