# ToPoFuzzy-Viewer 実行ガイド

このガイドは、パフォーマンス最適化および起動順序の改善が施された最新の状態での実行手順をまとめたものです。

## 1. バックエンドのビルドと起動
バックエンド（C++）の修正を反映し、WebSocketゲートウェイを立ち上げます。
```bash
colcon build --symlink-install --packages-select topo_fuzzy_viewer && source install/setup.bash && ros2 launch topo_fuzzy_viewer viewer_stack.launch.py
```

## 2. ブリッジノードの起動
ロボットアームと静的グラフ（GNG）のデータをビューアへ中継します。
```bash
source install/setup.bash && ros2 launch gng_vlut_system robot_viewer_bridge.launch.py & ros2 launch gng_vlut_system gng_viewer_bridge.launch.py topic_name:=/topological_map_static
```

## 3. フロントエンドの起動
Viteデバッグサーバを立ち上げます。ブラウザで可視化画面が表示されます。
```bash
cd ToPoFuzzy-Viewer/frontend && npm install && npm run dev
```

---

## 🛠 改善されたポイント
- **低負荷レンダリング**: 静止時のGPU負荷を極限まで抑えています。
- **起動順序の自由化**: どのノードから起動しても、最新のデータが自動的に同期されます。
- **効率的な通信**: URDFなどの重いデータ送信を最小限に抑え、PCの負荷を軽減しました。
- **インタラクティブ性の向上**: スライダーによるノードサイズ変更などが即座に反映されます。
