# ToPoFuzzy-Viewer 実行ガイド

## 開発メモ
@contextScopeItemMention に現状整理と今後の方針を書いています。ひとまずの目標はtopofuzzyviewerで複数アームの場合のweight coordsについて可視化できるようにすることです。またこの際に

### 現状整理
- `offline_urdf_trainer` は、joint-space を1本共有しつつ、coord 側を arm ごとのレイヤーに分ける方向で拡張中です。
- `weight_coord` は1点前提から、`weight_coords` を持てるように広げています。
- `coord edge` は arm ごとに別レイヤーとして扱う方針です。
- `topological_map` 自体は canonical な1本のまま維持し、複数可視化点は `MarkerArray` で補助表示する方針です。
- `robot_viewer_bridge_node` は multi-arm の URDF と `/joint_states` に対応済みです。
- `topodual.yaml` は双腕用の共通設定として、trainer と viewer の両方に使う前提です。

### 今後の予定
- `GrowingNeuralGas` の多層 coord 対応を整理して、コンパイルが通る形に詰める。
- `offline_urdf_trainer` の coord layer ごとの学習を安定化する。
- `topofuzzy_bridge_node` 側にも、必要なら arm layer ごとの補助可視化を追加する。
- `ToPoFuzzy-Viewer` 側で多点 MarkerArray を受ける経路を確認する。
- `topodual.yaml` と既存単腕 YAML の役割分担をもう少し見やすく整理する。

これで一発でfrontendが立ち上がる
-dをつけるとバックグラウウンドで起動し続けて面倒
docker compose --profile manual up  frontend

以下で終了できる-dを押していても終了はできる
docker compose stop frontend

## ロスバグの再生例
ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ --topics /camera/camera/depth/color/points  --loop

## 1. バックエンドのビルドと起動
バックエンド（C++）の修正を反映し、WebSocketゲートウェイを立ち上げます。
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash 
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py
```

## 2. ブリッジノードの起動
`static_only` では、`gng.bin` を静的グラフとしてビューアへ中継します。
この構成では `enable_safety_monitor:=false` にして、`/topological_map` は出しません。
```bash
source install/setup.bash && ros2 launch gng_vlut_system gng_vlut_runtime.launch.py robot_name:=topoarm enable_joint_state_publisher:=true enable_safety_monitor:=false 

ros2 launch gng_vlut_system gng_viewer_bridge.launch.py topic_name:=/topological_map_static robot_base_frame:=base_link gng_frame_id:=base_link 
```

## 3. フロントエンドの起動
Viteデバッグサーバを立ち上げます。ブラウザで可視化画面が表示されます。
```bash
cd ToPoFuzzy-Viewer/frontend && npm install && npm run dev
```

## 4. リアルタイム位置合わせ（キャリブレーション）
センサーとロボットの位置がズレている場合、以下の手順で GUI から調整できます。

1. **変換ノードの起動**:
   ```bash
   # 必要に応じて scipy をインストール: pip install scipy
   python3 scratch/transform_pointcloud.py
   ```

2. **Viewer での操作**:
   - `Edit` タブを開く
   - `TF Calibration (Real-time)` を展開
   - `x, y, z, roll, pitch, yaw` のスライダーを動かして、ロボットモデルと点群を合わせる
   - **点群トピック**: Sources から `/camera/transformed_points` を選択してください。


---

## 🛠 static_only のポイント
- `/topological_map_static` のみを使います。
- `/topological_map` を出す安全監視は止めています。
- ロボット姿勢は `robot_viewer_bridge_node` から `/viewer/internal/stream/robot` に流します。

## 📌 graph の今後の方針
- `topological_map` をそのまま viewer に入れられるなら、基本は direct topic を優先します。
- `/viewer/internal/stream/graph` は viewer 内部の中継レーンです。将来の正規化や差し替えには便利ですが、いまは複製の原因になりやすいです。
。
- static / dynamic を両方使うときは、どちらの経路に一本化するかを先に決めて、二重購読しないようにします。


python3 test_joint_state_publisher.py 
オプションジョイント名をつけたい場合
--prefix topoarm_

python3 test_tf_publisher.py
(--ros-args -p frame_id:=topoarm/base_link)

ros2 launch gng_vlut_system gng_viewer_bridge.launch.py   topic_name:=/topological_map_static   robot_base_frame:=base_link   gng_frame_id:=base_link

realsenseのrosbag  （dynamicの方もtfに対応しているがこれは処理が重くなる原因なのでやめたほうがいいかも）

ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ --topics /camera/camera/depth/color/points --loop

点群の座標変換版トピック
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py 

realsenseにおけるGNG
ros2 launch ais_gng camera_depth_points.launch.py target_frame_id:=world

robotの自己認識ボクセル
ros2 launch gng_vlut_system self_recognition_viz.launch.py 
(marker_frame_id:=world display_mode:=link_local)

座標変換
ros2 run gng_vlut_system self_recognition_filter_node


双腕召喚
ros2 launch gng_vlut_system gng_viewer_bridge.launch.py \
  robot_description_file:=package://gng_vlut_system/urdf/topoarm_description/urdf/topoarm_dual.urdf.xacro
