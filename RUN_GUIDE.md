# ToPoFuzzy-Viewer 実行ガイド

このガイドは、パフォーマンス最適化および起動順序の改善が施された最新の状態での実行手順をまとめたものです。

## 1. バックエンドのビルドと起動
バックエンド（C++）の修正を反映し、WebSocketゲートウェイを立ち上げます。
```bash
colcon build --symlink-install --packages-select topo_fuzzy_viewer && source install/setup.bash && ros2 launch topo_fuzzy_viewer viewer_stack.launch.py
```

## 2. ブリッジノードの起動
`static_only` では、`gng.bin` を静的グラフとしてビューアへ中継します。
この構成では `enable_safety_monitor:=false` にして、`/topological_map` は出しません。
```bash
source install/setup.bash && ros2 launch gng_vlut_system gng_vlut_runtime.launch.py robot_name:=topoarm enable_joint_state_publisher:=true enable_safety_monitor:=false & ros2 launch gng_vlut_system gng_viewer_bridge.launch.py topic_name:=/topological_map_static
```

## 3. フロントエンドの起動
Viteデバッグサーバを立ち上げます。ブラウザで可視化画面が表示されます。
```bash
cd ToPoFuzzy-Viewer/frontend && npm install && npm run dev
```

---

## 🛠 static_only のポイント
- `/topological_map_static` のみを使います。
- `/topological_map` を出す安全監視は止めています。
- ロボット姿勢は `robot_viewer_bridge_node` から `/viewer/internal/stream/robot` に流します。


python3 test_joint_state_publisher.py 
オプションジョイント名をつけたい場合
--prefix topoarm_

python3 test_tf_publisher.py --ros-args -p frame_id:=base_link

ros2 launch gng_vlut_system gng_viewer_bridge.launch.py   topic_name:=/topological_map_static   robot_base_frame:=base_link   gng_frame_id:=base_link

realsenseのrosbag
ros2 launch ais_gng camera_depth_points.launch.py \base_frame_id:=camera_depth_optical_frame
