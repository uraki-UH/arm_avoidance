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
ros2 launch ais_gng camera_depth_points.launch.py target_frame_id:=world

map に直したい場合は `target_frame_id:=map` を使います。入力点群の `header.frame_id` がセンサー座標系で、GNG 側が TF を引いて target frame に変換します。


realsenseを位置合わせする場合（目安）ロボットの位置を合わせている
position: 0.5600 -0.5600 0.2800
rotation_deg: -101.00 -43.00 -3.00
scale: 1.0000 1.0000 1.0000


今後の予定１
位置合わせでtfに直接できるようにしてしまう？
