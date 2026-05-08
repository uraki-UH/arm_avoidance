# ToPoFuzzy-Viewer 実行ガイド

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
- 処理時間の差はありますが、GNG 本体の計算や描画に比べると小さいです。direct topic のほうが 1 hop 少ない分だけ素直、internal stream は 1 hop ぶんだけ余計、という理解で十分です。
- static / dynamic を両方使うときは、どちらの経路に一本化するかを先に決めて、二重購読しないようにします。


python3 test_joint_state_publisher.py 
オプションジョイント名をつけたい場合
--prefix topoarm_

python3 test_tf_publisher.py --ros-args -p frame_id:=base_link

ros2 launch gng_vlut_system gng_viewer_bridge.launch.py   topic_name:=/topological_map_static   robot_base_frame:=base_link   gng_frame_id:=base_link

realsenseのrosbag  （dynamicの方もtfに対応しているがこれは処理が重くなる原因なのでやめたほうがいいかも）

ros2 bag play /rosbag/uraki/rosbag2_2026_04_22-19_10_41/ --topics /camera/camera/depth/color/points --loop

点群の座標変換版トピック
ros2 launch pointcloud_transformer_cpp pointcloud_transformer.launch.py 

realsenseにおけるGNG
ros2 launch ais_gng camera_depth_points.launch.py target_frame_id:=world

robotの自己認識ボクセル
ros2 launch gng_vlut_system self_recognition_viz.launch.py marker_frame_id:=world display_mode:=link_local

realsenseを位置合わせする場合（目安）ロボットの位置を合わせている
position: 0.4340 -0.6930 0.2790
rotation_deg: -103.80 -28.90 -3.40
scale: 1.0000 1.0000 1.0000

座標変換
ros2 run gng_vlut_system self_recognition_filter_node